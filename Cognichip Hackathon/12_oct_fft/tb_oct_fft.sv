`timescale 1ns/1ps
// ============================================================
//  TB-12 — tb_oct_fft
//  1024-point (and 512-point) Memory-Based FFT Testbench
//  Clock : 250 MHz (4 ns period)
//  DUT   : oct_fft
// ============================================================

module tb_oct_fft;

  // --------------------------------------------------------
  //  Parameters
  // --------------------------------------------------------
  localparam int    CLK_PERIOD_NS = 4;           // 250 MHz
  localparam int    N1024         = 1024;
  localparam int    N512          = 512;
  localparam real   TIMEOUT_US    = 500.0;
  localparam int    TIMEOUT_CYCLES = int'(TIMEOUT_US * 1000.0 / CLK_PERIOD_NS);

  // --------------------------------------------------------
  //  DUT Signals
  // --------------------------------------------------------
  logic        clk;
  logic        rst_n;

  // fft_size: 1'b1 = 1024-pt, 2'b00 = 512-pt
  logic  [1:0] fft_size;

  // AXI4-Stream slave (input)
  logic        s_axis_tvalid;
  logic        s_axis_tready;
  logic [31:0] s_axis_tdata;   // {imag[15:0], real[15:0]}
  logic        s_axis_tlast;

  // AXI4-Stream master (output)
  logic        m_axis_tvalid;
  logic        m_axis_tready;
  logic [31:0] m_axis_tdata;   // {imag[15:0], real[15:0]}
  logic        m_axis_tlast;

  // --------------------------------------------------------
  //  DUT Instantiation
  // --------------------------------------------------------
  oct_fft dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .fft_size       (fft_size),
    .s_axis_tvalid  (s_axis_tvalid),
    .s_axis_tready  (s_axis_tready),
    .s_axis_tdata   (s_axis_tdata),
    .s_axis_tlast   (s_axis_tlast),
    .m_axis_tvalid  (m_axis_tvalid),
    .m_axis_tready  (m_axis_tready),
    .m_axis_tdata   (m_axis_tdata),
    .m_axis_tlast   (m_axis_tlast)
  );

  // --------------------------------------------------------
  //  Clock Generation
  // --------------------------------------------------------
  initial clk = 1'b0;
  always #(CLK_PERIOD_NS/2) clk = ~clk;

  // --------------------------------------------------------
  //  Global timeout watchdog
  // --------------------------------------------------------
  int timeout_ctr;
  always @(posedge clk) begin
    if (!rst_n)
      timeout_ctr <= 0;
    else begin
      timeout_ctr <= timeout_ctr + 1;
      if (timeout_ctr >= TIMEOUT_CYCLES) begin
        $display("TB_OCT_FFT: FAIL — TIMEOUT after %.0f µs", TIMEOUT_US);
        $finish;
      end
    end
  end

  // --------------------------------------------------------
  //  Storage for one output frame (max 1024 samples)
  // --------------------------------------------------------
  logic signed [15:0] out_real [0:N1024-1];
  logic signed [15:0] out_imag [0:N1024-1];
  int                 out_count;

  // --------------------------------------------------------
  //  Helper: wait N clock rising edges
  // --------------------------------------------------------
  task automatic wait_clk(input int n = 1);
    repeat (n) @(posedge clk);
  endtask

  // --------------------------------------------------------
  //  AXI4-Stream Send Task  (translate_off real arithmetic)
  // --------------------------------------------------------
  // synthesis translate_off
  task automatic send_tone(input int bin_k, input real amplitude, input int N);
    // N complex samples: real[n] = amplitude * cos(2π*k*n/N), imag[n] = 0
    // Packed as {imag[15:0], real[15:0]}
    real    angle;
    integer sample_val;
    logic signed [15:0] re, im;

    for (int n = 0; n < N; n++) begin
      angle      = 2.0 * 3.14159265358979323846 * real'(bin_k) * real'(n) / real'(N);
      sample_val = int'($floor(amplitude * $cos(angle) + 0.5));

      // Clamp to Q1.15 range [-32768, 32767]
      if (sample_val >  32767) sample_val =  32767;
      if (sample_val < -32768) sample_val = -32768;

      re = signed'(16'(sample_val));
      im = 16'sh0000;

      @(posedge clk);
      s_axis_tvalid <= 1'b1;
      s_axis_tdata  <= {im, re};
      s_axis_tlast  <= (n == N-1) ? 1'b1 : 1'b0;

      // Honour back-pressure
      do @(posedge clk); while (!s_axis_tready);
    end

    s_axis_tvalid <= 1'b0;
    s_axis_tlast  <= 1'b0;
  endtask
  // synthesis translate_on

  // --------------------------------------------------------
  //  AXI4-Stream Send DC Task  (translate_off real arithmetic)
  // --------------------------------------------------------
  // synthesis translate_off
  task automatic send_dc(input real amplitude, input int N);
    logic signed [15:0] re, im;
    integer sample_val;

    sample_val = int'($floor(amplitude + 0.5));
    if (sample_val >  32767) sample_val =  32767;
    if (sample_val < -32768) sample_val = -32768;

    re = signed'(16'(sample_val));
    im = 16'sh0000;

    for (int n = 0; n < N; n++) begin
      @(posedge clk);
      s_axis_tvalid <= 1'b1;
      s_axis_tdata  <= {im, re};
      s_axis_tlast  <= (n == N-1) ? 1'b1 : 1'b0;

      do @(posedge clk); while (!s_axis_tready);
    end

    s_axis_tvalid <= 1'b0;
    s_axis_tlast  <= 1'b0;
  endtask
  // synthesis translate_on

  // --------------------------------------------------------
  //  Collect output frame (up to max_samples)
  // --------------------------------------------------------
  task automatic collect_frame(input int max_samples);
    out_count = 0;
    m_axis_tready <= 1'b1;

    while (out_count < max_samples) begin
      @(posedge clk);
      if (m_axis_tvalid && m_axis_tready) begin
        out_real[out_count] = signed'(m_axis_tdata[15:0]);
        out_imag[out_count] = signed'(m_axis_tdata[31:16]);
        out_count++;
        if (m_axis_tlast) break;
      end
    end

    m_axis_tready <= 1'b0;
  endtask

  // --------------------------------------------------------
  //  Count output samples for exactly N assertion
  // --------------------------------------------------------
  task automatic count_output_samples(input int max_samples, output int cnt);
    cnt = 0;
    m_axis_tready <= 1'b1;

    for (int i = 0; i < max_samples + 10; i++) begin
      @(posedge clk);
      if (m_axis_tvalid && m_axis_tready) begin
        cnt++;
        if (m_axis_tlast) break;
      end
    end

    m_axis_tready <= 1'b0;
  endtask

  // --------------------------------------------------------
  //  Magnitude helper  (synthesis translate_off)
  // --------------------------------------------------------
  // synthesis translate_off
  function automatic real mag_bin(input int k);
    real re_f, im_f;
    re_f = real'(out_real[k]);
    im_f = real'(out_imag[k]);
    return $sqrt(re_f*re_f + im_f*im_f);
  endfunction

  function automatic int peak_bin(input int N);
    real max_mag, m;
    int  peak;
    max_mag = 0.0;
    peak    = 0;
    for (int k = 0; k < N; k++) begin
      m = mag_bin(k);
      if (m > max_mag) begin
        max_mag = m;
        peak    = k;
      end
    end
    return peak;
  endfunction

  function automatic real max_sidelobe(input int peak, input int N);
    real max_sl, m;
    max_sl = 0.0;
    for (int k = 0; k < N; k++) begin
      if (k == peak) continue;
      m = mag_bin(k);
      if (m > max_sl) max_sl = m;
    end
    return max_sl;
  endfunction
  // synthesis translate_on

  // --------------------------------------------------------
  //  Storage for double-buffer test (frame B)
  // --------------------------------------------------------
  logic signed [15:0] outB_real [0:N1024-1];
  logic signed [15:0] outB_imag [0:N1024-1];

  // --------------------------------------------------------
  //  Main Test Sequence
  // --------------------------------------------------------
  // synthesis translate_off
  string fail_msg;
  int    peak_k;
  real   peak_mag, sidelobe, sfdr, expected_peak, dc_mag0, dc_mag1;
  int    sample_cnt;
  // synthesis translate_on

  initial begin : tb_main
    // synthesis translate_off

    // ---- Reset ----
    rst_n         <= 1'b0;
    fft_size      <= 2'b01;   // 1'b1 = 1024-point per spec
    s_axis_tvalid <= 1'b0;
    s_axis_tdata  <= 32'h0;
    s_axis_tlast  <= 1'b0;
    m_axis_tready <= 1'b0;
    fail_msg       = "";

    wait_clk(8);
    rst_n <= 1'b1;
    wait_clk(4);

    // ==============================================================
    //  TEST 1 — Single-tone at bin 50
    // ==============================================================
    $display("TB_OCT_FFT: Starting TEST 1 — Single-tone bin 50");
    fft_size <= 2'b01;  // 1024-point

    fork
      send_tone(50, 16383.0, N1024);
      collect_frame(N1024);
    join

    wait_clk(2);

    begin : t1_check
      // synthesis translate_off
      peak_k        = peak_bin(N1024);
      peak_mag      = mag_bin(peak_k);
      expected_peak = 16383.0 * 512.0;          // amplitude * N/2
      sidelobe      = max_sidelobe(peak_k, N1024);

      if (peak_k !== 50) begin
        $display("TB_OCT_FFT: FAIL — TEST1 Single-tone (peak at bin %0d)", peak_k);
        $finish;
      end

      if (peak_mag < 0.9 * expected_peak) begin
        $display("TB_OCT_FFT: FAIL — TEST1 Single-tone peak magnitude too low: %.1f < %.1f",
                 peak_mag, 0.9*expected_peak);
        $finish;
      end

      if (sidelobe > 0.0) begin
        sfdr = peak_mag / sidelobe;
        if (sfdr < 50.0) begin
          $display("TB_OCT_FFT: FAIL — TEST1 Single-tone SFDR too low: %.2f < 50.0", sfdr);
          $finish;
        end
      end

      $display("TB_OCT_FFT: TEST 1 PASS — peak bin=%0d, mag=%.1f, SFDR=%.1f",
               peak_k, peak_mag, (sidelobe > 0.0) ? peak_mag/sidelobe : 999.9);
      // synthesis translate_on
    end

    wait_clk(4);

    // ==============================================================
    //  TEST 2 — DC input (bin 0)
    // ==============================================================
    $display("TB_OCT_FFT: Starting TEST 2 — DC input");
    fft_size <= 2'b01;

    fork
      send_dc(1000.0, N1024);
      collect_frame(N1024);
    join

    wait_clk(2);

    begin : t2_check
      // synthesis translate_off
      peak_k   = peak_bin(N1024);
      dc_mag0  = mag_bin(0);
      dc_mag1  = mag_bin(1);

      if (peak_k !== 0) begin
        $display("TB_OCT_FFT: FAIL — TEST2 DC input (peak at bin %0d)", peak_k);
        $finish;
      end

      if (dc_mag0 <= 10.0 * dc_mag1) begin
        $display("TB_OCT_FFT: FAIL — TEST2 DC leakage: mag[0]=%.1f not > 10*mag[1]=%.1f",
                 dc_mag0, 10.0*dc_mag1);
        $finish;
      end

      $display("TB_OCT_FFT: TEST 2 PASS — DC mag[0]=%.1f, mag[1]=%.1f, ratio=%.1f",
               dc_mag0, dc_mag1, dc_mag1 > 0.0 ? dc_mag0/dc_mag1 : 999.9);
      // synthesis translate_on
    end

    wait_clk(4);

    // ==============================================================
    //  TEST 3 — Back-to-back frames (double-buffer)
    // ==============================================================
    $display("TB_OCT_FFT: Starting TEST 3 — Back-to-back frames");
    fft_size <= 2'b01;

    // Send frame A (bin 10) then immediately frame B (bin 200) with no gap.
    // Collect both frames concurrently.
    fork
      begin : send_ab
        // Frame A — bin 10
        for (int n = 0; n < N1024; n++) begin
          real angle_a;
          integer sv_a;
          logic signed [15:0] re_a, im_a;
          angle_a = 2.0 * 3.14159265358979323846 * 10.0 * real'(n) / real'(N1024);
          sv_a    = int'($floor(16383.0 * $cos(angle_a) + 0.5));
          if (sv_a >  32767) sv_a =  32767;
          if (sv_a < -32768) sv_a = -32768;
          re_a = signed'(16'(sv_a));
          im_a = 16'sh0000;
          @(posedge clk);
          s_axis_tvalid <= 1'b1;
          s_axis_tdata  <= {im_a, re_a};
          s_axis_tlast  <= (n == N1024-1) ? 1'b1 : 1'b0;
          do @(posedge clk); while (!s_axis_tready);
        end
        // Frame B — bin 200, NO gap (tvalid stays high)
        for (int n = 0; n < N1024; n++) begin
          real angle_b;
          integer sv_b;
          logic signed [15:0] re_b, im_b;
          angle_b = 2.0 * 3.14159265358979323846 * 200.0 * real'(n) / real'(N1024);
          sv_b    = int'($floor(16383.0 * $cos(angle_b) + 0.5));
          if (sv_b >  32767) sv_b =  32767;
          if (sv_b < -32768) sv_b = -32768;
          re_b = signed'(16'(sv_b));
          im_b = 16'sh0000;
          @(posedge clk);
          s_axis_tvalid <= 1'b1;
          s_axis_tdata  <= {im_b, re_b};
          s_axis_tlast  <= (n == N1024-1) ? 1'b1 : 1'b0;
          do @(posedge clk); while (!s_axis_tready);
        end
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
      end

      begin : collect_ab
        // Collect Frame A
        collect_frame(N1024);

        // Stash Frame A results
        for (int k = 0; k < N1024; k++) begin
          outB_real[k] = out_real[k];   // temp reuse for stash
          outB_imag[k] = out_imag[k];
        end

        // Collect Frame B into out_real/out_imag
        collect_frame(N1024);
      end
    join

    wait_clk(2);

    begin : t3_check
      // synthesis translate_off
      // Check Frame A (stashed in outB_*)
      real max_a, m;
      int  pk_a, pk_b;
      real sl_a, sl_b;

      max_a = 0.0;
      pk_a  = 0;
      for (int k = 0; k < N1024; k++) begin
        real re_f, im_f;
        re_f = real'(outB_real[k]);
        im_f = real'(outB_imag[k]);
        m = $sqrt(re_f*re_f + im_f*im_f);
        if (m > max_a) begin
          max_a = m;
          pk_a  = k;
        end
      end

      pk_b  = peak_bin(N1024);  // Frame B in out_real/out_imag

      // Sidelobe of frame A around bin 10
      sl_a = 0.0;
      for (int k = 0; k < N1024; k++) begin
        real re_f, im_f;
        if (k == pk_a) continue;
        re_f = real'(outB_real[k]);
        im_f = real'(outB_imag[k]);
        m = $sqrt(re_f*re_f + im_f*im_f);
        if (m > sl_a) sl_a = m;
      end

      if (pk_a !== 10) begin
        $display("TB_OCT_FFT: FAIL — TEST3 Back-to-back Frame A (peak at bin %0d)", pk_a);
        $finish;
      end

      if (pk_b !== 200) begin
        $display("TB_OCT_FFT: FAIL — TEST3 Back-to-back Frame B (peak at bin %0d)", pk_b);
        $finish;
      end

      // No contamination: Frame A SFDR >= 50
      if (sl_a > 0.0 && (max_a / sl_a) < 50.0) begin
        $display("TB_OCT_FFT: FAIL — TEST3 Frame A contaminated, SFDR=%.2f", max_a/sl_a);
        $finish;
      end

      // No contamination: Frame B SFDR >= 50
      sl_b = max_sidelobe(pk_b, N1024);
      if (sl_b > 0.0 && (mag_bin(pk_b) / sl_b) < 50.0) begin
        $display("TB_OCT_FFT: FAIL — TEST3 Frame B contaminated, SFDR=%.2f",
                 mag_bin(pk_b)/sl_b);
        $finish;
      end

      $display("TB_OCT_FFT: TEST 3 PASS — Frame A peak=%0d, Frame B peak=%0d", pk_a, pk_b);
      // synthesis translate_on
    end

    wait_clk(4);

    // ==============================================================
    //  TEST 4 — Output sample count = exactly 1024
    // ==============================================================
    $display("TB_OCT_FFT: Starting TEST 4 — Output sample count");
    fft_size <= 2'b01;

    fork
      send_tone(75, 8000.0, N1024);
      count_output_samples(N1024 + 64, sample_cnt);
    join

    wait_clk(2);

    if (sample_cnt !== N1024) begin
      $display("TB_OCT_FFT: FAIL — TEST4 Output count (got %0d, expected %0d)",
               sample_cnt, N1024);
      $finish;
    end
    $display("TB_OCT_FFT: TEST 4 PASS — output sample count = %0d", sample_cnt);

    wait_clk(4);

    // ==============================================================
    //  TEST 5 — fft_size = 2'b00 (512-point mode)
    // ==============================================================
    $display("TB_OCT_FFT: Starting TEST 5 — 512-point mode");
    fft_size <= 2'b00;   // 512-point per spec
    wait_clk(4);

    fork
      send_tone(20, 16383.0, N512);
      begin
        count_output_samples(N512 + 64, sample_cnt);
        // Re-collect into out_real/out_imag for peak check
      end
    join

    // Re-run with data capture enabled
    fft_size <= 2'b00;
    wait_clk(4);

    fork
      send_tone(20, 16383.0, N512);
      collect_frame(N512);
    join

    wait_clk(2);

    begin : t5_check
      // synthesis translate_off
      int pk5;
      pk5 = peak_bin(N512);

      if (sample_cnt !== N512) begin
        $display("TB_OCT_FFT: FAIL — TEST5 512-pt output count (got %0d, expected %0d)",
                 sample_cnt, N512);
        $finish;
      end

      if (pk5 !== 20) begin
        $display("TB_OCT_FFT: FAIL — TEST5 512-pt mode (peak at bin %0d)", pk5);
        $finish;
      end

      $display("TB_OCT_FFT: TEST 5 PASS — 512-pt mode, count=%0d, peak bin=%0d",
               sample_cnt, pk5);
      // synthesis translate_on
    end

    // ==============================================================
    //  All Tests Passed
    // ==============================================================
    wait_clk(4);
    $display("TB_OCT_FFT: PASS");
    $finish;

    // synthesis translate_on
  end : tb_main

endmodule : tb_oct_fft