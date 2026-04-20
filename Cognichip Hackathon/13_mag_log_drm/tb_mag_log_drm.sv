`timescale 1ns/1ps
// ============================================================
//  TB-13 — tb_mag_log_drm
//  Magnitude + Log LUT + Dynamic Range Mapping Testbench
//  Clock  : 250 MHz (4 ns period)
//  DUT    : mag_log_drm
// ============================================================

module tb_mag_log_drm;

  // --------------------------------------------------------
  //  Parameters
  // --------------------------------------------------------
  localparam int    CLK_PERIOD_NS  = 4;           // 250 MHz
  localparam real   TIMEOUT_US     = 50.0;
  localparam int    TIMEOUT_CYCLES = int'(TIMEOUT_US * 1000.0 / CLK_PERIOD_NS);

  // Pipeline latency when cordic_en = 0 (power → log → DRM)
  // Adjust PIPE_LAT to match your DUT's actual pipeline depth.
  localparam int    PIPE_LAT       = 5;

  // DRM window used throughout (cordic_en=0 tests)
  localparam logic [31:0] DRM_MIN  = 32'd100;
  localparam logic [31:0] DRM_MAX  = 32'd60000;

  // CORDIC phase tolerance in Q1.15 LSBs
  localparam int    PHASE_TOL      = 100;

  // --------------------------------------------------------
  //  DUT Port Declarations
  // --------------------------------------------------------
  logic        clk;
  logic        rst_n;

  // Configuration
  logic [31:0] drm_min;
  logic [31:0] drm_max;
  logic        cordic_en;

  // AXI4-Stream slave  {imag[15:0], real[15:0]}
  logic        s_tvalid;
  logic        s_tready;
  logic [31:0] s_tdata;

  // AXI4-Stream master
  // When cordic_en=0 : m_tdata[7:0]  = DRM byte
  // When cordic_en=1 : m_tdata[31:16]= phase Q1.15, [7:0]= DRM byte
  logic        m_tvalid;
  logic        m_tready;
  logic [31:0] m_tdata;

  // --------------------------------------------------------
  //  DUT Instantiation
  // --------------------------------------------------------
  mag_log_drm dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .drm_min   (drm_min),
    .drm_max   (drm_max),
    .cordic_en (cordic_en),
    .s_tvalid  (s_tvalid),
    .s_tready  (s_tready),
    .s_tdata   (s_tdata),
    .m_tvalid  (m_tvalid),
    .m_tready  (m_tready),
    .m_tdata   (m_tdata)
  );

  // --------------------------------------------------------
  //  Clock Generation
  // --------------------------------------------------------
  initial clk = 1'b0;
  always  #(CLK_PERIOD_NS/2) clk = ~clk;

  // --------------------------------------------------------
  //  Timeout Watchdog
  // --------------------------------------------------------
  int timeout_ctr;
  always @(posedge clk) begin
    if (!rst_n)
      timeout_ctr <= 0;
    else begin
      timeout_ctr <= timeout_ctr + 1;
      if (timeout_ctr >= TIMEOUT_CYCLES) begin
        $display("TB_MAG_LOG: FAIL — TIMEOUT after %.0f µs", TIMEOUT_US);
        $finish;
      end
    end
  end

  // --------------------------------------------------------
  //  Helpers
  // --------------------------------------------------------
  task automatic wait_clk(input int n = 1);
    repeat (n) @(posedge clk);
  endtask

  // Drive one complex sample; honour back-pressure
  task automatic send_sample(
    input logic signed [15:0] re,
    input logic signed [15:0] im
  );
    @(posedge clk);
    s_tvalid <= 1'b1;
    s_tdata  <= {im, re};
    do @(posedge clk); while (!s_tready);
    s_tvalid <= 1'b0;
  endtask

  // Collect one output beat; m_tready stays high outside this task
  task automatic collect_one(output logic [31:0] d);
    m_tready <= 1'b1;
    do @(posedge clk); while (!m_tvalid);
    d = m_tdata;
    @(posedge clk);
    m_tready <= 1'b0;
  endtask

  // Send one sample and wait for the output, return the full output word
  task automatic send_and_collect(
    input  logic signed [15:0] re,
    input  logic signed [15:0] im,
    output logic [31:0]        out_word
  );
    fork
      send_sample(re, im);
      collect_one(out_word);
    join
  endtask

  // --------------------------------------------------------
  //  Test state
  // --------------------------------------------------------
  logic [31:0] result;
  logic  [7:0] drm_byte;
  logic signed [15:0] phase_out;

  // --------------------------------------------------------
  //  Reset + global config
  // --------------------------------------------------------
  initial begin : tb_main
    // synthesis translate_off

    // --- initialise driven signals ---
    rst_n     <= 1'b0;
    drm_min   <= DRM_MIN;
    drm_max   <= DRM_MAX;
    cordic_en <= 1'b0;
    s_tvalid  <= 1'b0;
    s_tdata   <= 32'h0;
    m_tready  <= 1'b0;

    wait_clk(8);
    rst_n <= 1'b1;
    wait_clk(4);

    // ==================================================================
    //  TEST 1 — Zero input → zero (or DRM minimum) output
    // ==================================================================
    $display("TB_MAG_LOG: Starting TEST 1 — Zero input");

    send_and_collect(16'sd0, 16'sd0, result);
    drm_byte = result[7:0];

    if (drm_byte !== 8'd0) begin
      $display("TB_MAG_LOG: FAIL — TEST1 Zero input (got 8'd%0d, expected 8'd0)", drm_byte);
      $finish;
    end
    $display("TB_MAG_LOG: TEST 1 PASS — zero-input DRM byte = %0d", drm_byte);
    wait_clk(2);

    // ==================================================================
    //  TEST 2 — Known magnitude: real=1000, imag=0
    //           power = 1000² = 1_000_000
    //           DRM: log2(1_000_000) ≈ 19.93
    //           log2(DRM_MIN=100)   ≈  6.64
    //           log2(DRM_MAX=60000) ≈ 15.87
    //           scaled ≈ 255*(19.93-6.64)/(15.87-6.64) — clamps to 255
    //           But DRM window is [100,60000] in linear power space;
    //           log(1_000_000) > log(60_000) so output = 255.
    //           Expect non-zero and in [0,255].
    // ==================================================================
    $display("TB_MAG_LOG: Starting TEST 2 — Known magnitude (real=1000)");

    send_and_collect(16'sd1000, 16'sd0, result);
    drm_byte = result[7:0];

    if (drm_byte === 8'd0 || drm_byte > 8'd255) begin
      $display("TB_MAG_LOG: FAIL — TEST2 Known magnitude (drm_byte=%0d out of (0,255])",
               drm_byte);
      $finish;
    end
    $display("TB_MAG_LOG: TEST 2 PASS — real=1000 → DRM byte = %0d", drm_byte);
    wait_clk(2);

    // ==================================================================
    //  TEST 3 — Monotonicity of log LUT
    //           8 samples with strictly increasing real magnitude,
    //           imag = 0.  Output bytes must be non-decreasing.
    // ==================================================================
    $display("TB_MAG_LOG: Starting TEST 3 — Log LUT monotonicity");

    begin : t3_block
      logic signed [15:0] magnitudes [0:7];
      logic         [7:0] outputs    [0:7];
      logic        [31:0] tmp;

      magnitudes = '{16'sd100, 16'sd200, 16'sd500, 16'sd1000,
                     16'sd2000, 16'sd5000, 16'sd10000, 16'sd16000};

      for (int i = 0; i < 8; i++) begin
        send_and_collect(magnitudes[i], 16'sd0, tmp);
        outputs[i] = tmp[7:0];
        $display("  real=%5d → DRM byte = %0d", magnitudes[i], outputs[i]);
      end

      for (int i = 1; i < 8; i++) begin
        if (outputs[i] < outputs[i-1]) begin
          $display("TB_MAG_LOG: FAIL — TEST3 Monotonicity violated at i=%0d: out[%0d]=%0d < out[%0d]=%0d",
                   i, i, outputs[i], i-1, outputs[i-1]);
          $finish;
        end
      end
    end

    $display("TB_MAG_LOG: TEST 3 PASS — log LUT is monotonically non-decreasing");
    wait_clk(2);

    // ==================================================================
    //  TEST 4 — DRM clamp at maximum
    //           {real=16383, imag=16383} → power ≈ 2*16383² ≈ 536,806,018
    //           >> DRM_MAX (60000) → should saturate to 255
    // ==================================================================
    $display("TB_MAG_LOG: Starting TEST 4 — DRM clamp at maximum");

    send_and_collect(16'sd16383, 16'sd16383, result);
    drm_byte = result[7:0];

    if (drm_byte !== 8'd255) begin
      $display("TB_MAG_LOG: FAIL — TEST4 DRM max clamp (got %0d, expected 255)", drm_byte);
      $finish;
    end
    $display("TB_MAG_LOG: TEST 4 PASS — near-max input saturates to 255");
    wait_clk(2);

    // ==================================================================
    //  TEST 5 — DRM clamp at minimum
    //           real=1, imag=0 → power = 1, far below DRM_MIN (100)
    //           → must clamp to 0
    // ==================================================================
    $display("TB_MAG_LOG: Starting TEST 5 — DRM clamp at minimum");

    send_and_collect(16'sd1, 16'sd0, result);
    drm_byte = result[7:0];

    if (drm_byte !== 8'd0) begin
      $display("TB_MAG_LOG: FAIL — TEST5 DRM min clamp (got %0d, expected 0)", drm_byte);
      $finish;
    end
    $display("TB_MAG_LOG: TEST 5 PASS — tiny input clamps to 0");
    wait_clk(2);

    // ==================================================================
    //  TEST 6 — Pipeline latency = PIPE_LAT cycles (cordic_en = 0)
    //           Send one non-zero sample flanked by zeros.
    //           m_tvalid should rise exactly PIPE_LAT cycles after
    //           the cycle on which s_tvalid is sampled high.
    // ==================================================================
    $display("TB_MAG_LOG: Starting TEST 6 — Pipeline latency check (%0d cycles)", PIPE_LAT);

    begin : t6_block
      int   valid_sent_cycle;
      int   valid_recv_cycle;
      int   latency;
      int   cycle_ctr;

      // Hold off m_tready so we can observe the exact cycle m_tvalid rises
      m_tready      <= 1'b0;
      cycle_ctr      = 0;
      valid_sent_cycle = -1;
      valid_recv_cycle = -1;

      // Drive one non-zero sample
      @(posedge clk);
      s_tvalid <= 1'b1;
      s_tdata  <= {16'sh0000, 16'sd4000};   // real=4000, imag=0
      @(posedge clk);
      // Sample accepted on this edge (assume tready=1 after reset)
      valid_sent_cycle = 0;
      s_tvalid <= 1'b0;

      // Count cycles until m_tvalid rises
      for (int i = 1; i <= PIPE_LAT + 4; i++) begin
        @(posedge clk);
        if (m_tvalid && valid_recv_cycle < 0)
          valid_recv_cycle = i;
      end

      if (valid_recv_cycle < 0) begin
        $display("TB_MAG_LOG: FAIL — TEST6 m_tvalid never asserted");
        $finish;
      end

      latency = valid_recv_cycle - valid_sent_cycle;

      if (latency !== PIPE_LAT) begin
        $display("TB_MAG_LOG: FAIL — TEST6 Pipeline latency mismatch: got %0d, expected %0d",
                 latency, PIPE_LAT);
        $finish;
      end

      // Drain the output
      m_tready <= 1'b1;
      @(posedge clk);
      m_tready <= 1'b0;
    end

    $display("TB_MAG_LOG: TEST 6 PASS — pipeline latency = %0d cycles", PIPE_LAT);
    wait_clk(4);

    // ==================================================================
    //  TEST 7 — CORDIC phase output (cordic_en = 1)
    //           Phase encoded as Q1.15 signed integer:
    //             +π  →  32767
    //             +π/2 → 16384
    //              0   →  0
    //             -π/2 → -16384
    //
    //  7a: {real=+10000, imag=0} → angle = 0°  → phase ≈ 0 ±100 LSBs
    //  7b: {real=0, imag=+10000} → angle = 90° → phase ≈ 16384 ±100 LSBs
    // ==================================================================
    $display("TB_MAG_LOG: Starting TEST 7 — CORDIC phase");

    cordic_en <= 1'b1;
    wait_clk(4);   // allow config to propagate

    // --- 7a: angle = 0° ---
    send_and_collect(16'sd10000, 16'sd0, result);
    phase_out = signed'(result[31:16]);

    if (phase_out > $signed(16'(PHASE_TOL)) ||
        phase_out < $signed(-16'(PHASE_TOL))) begin
      $display("TB_MAG_LOG: FAIL — TEST7a CORDIC 0° phase=%0d (tolerance ±%0d)",
               phase_out, PHASE_TOL);
      $finish;
    end
    $display("TB_MAG_LOG: TEST 7a PASS — 0° phase = %0d (expected ≈0)", phase_out);
    wait_clk(2);

    // --- 7b: angle = 90° (imag=+10000, real=0) ---
    send_and_collect(16'sd0, 16'sd10000, result);
    phase_out = signed'(result[31:16]);

    begin : t7b_check
      // synthesis translate_off
      int expected_90;
      int lo;
      int hi;
      expected_90 = 16384;
      lo          = expected_90 - PHASE_TOL;
      hi          = expected_90 + PHASE_TOL;

      if (int'(phase_out) < lo || int'(phase_out) > hi) begin
        $display("TB_MAG_LOG: FAIL — TEST7b CORDIC 90° phase=%0d (expected %0d ±%0d)",
                 phase_out, expected_90, PHASE_TOL);
        $finish;
      end
      $display("TB_MAG_LOG: TEST 7b PASS — 90° phase = %0d (expected ≈16384)", phase_out);
      // synthesis translate_on
    end

    // Restore cordic_en
    cordic_en <= 1'b0;
    wait_clk(4);

    // ==================================================================
    //  All Tests Passed
    // ==================================================================
    $display("TB_MAG_LOG: PASS");
    $finish;

    // synthesis translate_on
  end : tb_main

endmodule : tb_mag_log_drm