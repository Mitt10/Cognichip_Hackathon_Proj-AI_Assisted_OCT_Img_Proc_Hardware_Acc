`timescale 1ns/1ps

module tb_dispersion_comp;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  localparam int  PHASE_DEPTH = 1024;
  localparam int  DATA_W      = 16;
  localparam int  PHASE_W     = 18;
  localparam int  PIPE_LAT    = 2;     // Measured DUT latency is 2 cycles
  localparam real CLK_PERIOD  = 4.0;   // 250 MHz

  // Q1.17 constants packed as {cos[17:0], sin[17:0]} in phase_wdata
  // Note: signed Q1.17 cannot represent exact +1.0, so use max positive value.
  localparam logic signed [PHASE_W-1:0] COS_ONE  = 18'sh1FFFF; // ~ +0.999992
  localparam logic signed [PHASE_W-1:0] SIN_ZERO = 18'sh00000;
  localparam logic signed [PHASE_W-1:0] COS_ZERO = 18'sh00000;
  localparam logic signed [PHASE_W-1:0] SIN_ONE  = 18'sh1FFFF; // ~ +0.999992
  localparam logic signed [PHASE_W-1:0] COS45    = 18'sd92682; // ~ 0.7071
  localparam logic signed [PHASE_W-1:0] SIN45    = 18'sd92682;

  // ---------------------------------------------------------------------------
  // Clock / Reset
  // ---------------------------------------------------------------------------
  logic clk   = 1'b0;
  logic rst_n = 1'b0;

  always #(CLK_PERIOD/2) clk = ~clk;

  // ---------------------------------------------------------------------------
  // DUT Ports
  // ---------------------------------------------------------------------------
  logic [DATA_W-1:0]  s_tdata  = '0;
  logic               s_tvalid = 1'b0;
  logic               s_tready;
  logic               s_tlast  = 1'b0;
  logic [31:0]        s_tuser  = '0;

  logic [31:0]        m_tdata;
  logic               m_tvalid;
  logic               m_tready = 1'b1;
  logic               m_tlast;
  logic [31:0]        m_tuser;

  logic               phase_we    = 1'b0;
  logic [9:0]         phase_addr  = '0;
  logic [35:0]        phase_wdata = '0;

  logic               bypass = 1'b0;

  // ---------------------------------------------------------------------------
  // DUT instantiation
  // ---------------------------------------------------------------------------
  dispersion_comp #(
    .N       (PHASE_DEPTH),
    .DATA_W  (DATA_W),
    .PHASE_W (PHASE_W)
  ) dut (
    .clk         (clk),
    .rst_n       (rst_n),
    .s_tdata     (s_tdata),
    .s_tvalid    (s_tvalid),
    .s_tready    (s_tready),
    .s_tlast     (s_tlast),
    .s_tuser     (s_tuser),
    .m_tdata     (m_tdata),
    .m_tvalid    (m_tvalid),
    .m_tready    (m_tready),
    .m_tlast     (m_tlast),
    .m_tuser     (m_tuser),
    .phase_we    (phase_we),
    .phase_addr  (phase_addr),
    .phase_wdata (phase_wdata),
    .bypass      (bypass)
  );

  // ---------------------------------------------------------------------------
  // Module-level collection buffer (avoids dynamic array ref parameters)
  // ---------------------------------------------------------------------------
  logic [31:0] cbuf [512];

  // ---------------------------------------------------------------------------
  // Bookkeeping
  // ---------------------------------------------------------------------------
  int    fail_count = 0;
  string fail_msg   = "";

  task automatic record_fail(input string test_name);
    begin
      $display("[FAIL] %s", test_name);
      if (fail_msg == "") begin
        fail_msg = test_name;
      end
      fail_count = fail_count + 1;
    end
  endtask

  function automatic int absdiff(input int a, input int b);
    int d;
    begin
      d = a - b;
      if (d < 0) begin
        absdiff = -d;
      end else begin
        absdiff = d;
      end
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Helper tasks
  // ---------------------------------------------------------------------------

  // Load phase BRAM: phase_wdata = {cos[17:0], sin[17:0]}
  task automatic load_phase_ram(
    input int                        start,
    input int                        count,
    input logic signed [PHASE_W-1:0] cos_val,
    input logic signed [PHASE_W-1:0] sin_val
  );
    int i;
    begin
      for (i = 0; i < count; i = i + 1) begin
        @(negedge clk);
        phase_we    = 1'b1;
        phase_addr  = start + i;
        phase_wdata = {cos_val, sin_val};
      end
      @(negedge clk);
      phase_we    = 1'b0;
      phase_addr  = '0;
      phase_wdata = '0;
    end
  endtask

  task automatic send_ramp(
    input int start,
    input int n,
    input int last_at
  );
    int i;
    begin
      i = 0;
      while (i < n) begin
        @(negedge clk);
        s_tdata  = DATA_W'(start + i);
        s_tvalid = 1'b1;
        s_tlast  = (i == last_at);
        @(posedge clk); #1;
        if (s_tready) begin
          i = i + 1;
        end
      end
      @(negedge clk);
      s_tvalid = 1'b0;
      s_tlast  = 1'b0;
      s_tdata  = '0;
    end
  endtask

  task automatic send_const(
    input logic [DATA_W-1:0] val,
    input int                n,
    input int                last_at
  );
    int i;
    begin
      i = 0;
      while (i < n) begin
        @(negedge clk);
        s_tdata  = val;
        s_tvalid = 1'b1;
        s_tlast  = (i == last_at);
        @(posedge clk); #1;
        if (s_tready) begin
          i = i + 1;
        end
      end
      @(negedge clk);
      s_tvalid = 1'b0;
      s_tlast  = 1'b0;
      s_tdata  = '0;
    end
  endtask

  // Collect n output samples into module-level cbuf[]
  task automatic collect_n(input int n);
    int i;
    begin
      i = 0;
      while (i < n) begin
        @(posedge clk); #1;
        if (m_tvalid && m_tready) begin
          cbuf[i] = m_tdata;
          i = i + 1;
        end
      end
    end
  endtask

  // Flush pipeline contents without disturbing the phase index assumption too much
  task automatic flush_pipeline;
    int i;
    begin
      i = 0;
      while (i < PIPE_LAT + 2) begin
        @(negedge clk);
        s_tdata  = '0;
        s_tvalid = 1'b1;
        s_tlast  = 1'b0;
        s_tuser  = '0;
        @(posedge clk); #1;
        if (s_tready) begin
          i = i + 1;
        end
      end
      @(negedge clk);
      s_tvalid = 1'b0;
      s_tlast  = 1'b0;
      s_tuser  = '0;
      s_tdata  = '0;
      repeat (PIPE_LAT + 4) @(posedge clk);
    end
  endtask

  // Force DUT sample counter back to zero by sending one accepted beat with tlast=1
  task automatic realign_phase_index;
    begin
      @(negedge clk);
      s_tdata  = '0;
      s_tvalid = 1'b1;
      s_tlast  = 1'b1;
      s_tuser  = '0;
      @(posedge clk); #1;
      while (!s_tready) begin
        @(posedge clk); #1;
      end
      @(negedge clk);
      s_tvalid = 1'b0;
      s_tlast  = 1'b0;
      s_tuser  = '0;
      s_tdata  = '0;
      repeat (PIPE_LAT + 2) @(posedge clk);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Per-test tasks
  // ---------------------------------------------------------------------------

  // TEST 1: Identity phase - real_out = input, imag_out = 0
  task automatic run_test1;
    int r;
    int im;
    int i1;
    logic pass_r;
    logic pass_i;
    begin
      $display("");
      $display("=== TEST 1: Identity phase ===");
      pass_r   = 1'b1;
      pass_i   = 1'b1;
      m_tready = 1'b1;
      bypass   = 1'b0;

      fork
        send_ramp(0, 64, 63);
        collect_n(64);
      join

      for (i1 = 0; i1 < 64; i1 = i1 + 1) begin
        r  = $signed(cbuf[i1][DATA_W-1:0]);
        im = $signed(cbuf[i1][DATA_W*2-1:DATA_W]);

        if (pass_r && (absdiff(r, i1) > 2)) begin
          $display("LOG: %0t : ERROR : tb_t1 : dut.m_tdata[15:0] : expected_value: %0d actual_value: %0d",
                   $time, i1, r);
          $display("  [FAIL] T1 real[%0d] = %0d  expected ~%0d", i1, r, i1);
          record_fail("TEST1-identity-real");
          pass_r = 1'b0;
        end

        if (pass_i && (absdiff(im, 0) > 2)) begin
          $display("LOG: %0t : ERROR : tb_t1 : dut.m_tdata[31:16] : expected_value: 0 actual_value: %0d",
                   $time, im);
          $display("  [FAIL] T1 imag[%0d] = %0d  expected ~0", i1, im);
          record_fail("TEST1-identity-imag");
          pass_i = 1'b0;
        end
      end

      if (pass_r) begin
        $display("  [PASS] T1 real ~ ramp 0..63 (+-2)");
      end
      if (pass_i) begin
        $display("  [PASS] T1 imag ~ 0 (+-2)");
      end
    end
  endtask

  // TEST 2: 90-degree rotation - real_out=0, imag_out=input
  task automatic run_test2;
    int r;
    int im;
    int i2;
    logic pass_r;
    logic pass_i;
    begin
      $display("");
      $display("=== TEST 2: 90-degree rotation ===");
      pass_r = 1'b1;
      pass_i = 1'b1;

      // Ensure sample index starts from 0 for this test
      realign_phase_index();

      load_phase_ram(0, 64, COS_ZERO, SIN_ONE);
      repeat (4) @(posedge clk);
      m_tready = 1'b1;
      bypass   = 1'b0;

      fork
        send_const(16'd1000, 64, 63);
        collect_n(64);
      join

      for (i2 = 0; i2 < 64; i2 = i2 + 1) begin
        r  = $signed(cbuf[i2][DATA_W-1:0]);
        im = $signed(cbuf[i2][DATA_W*2-1:DATA_W]);

        if (pass_r && (absdiff(r, 0) > 5)) begin
          $display("LOG: %0t : ERROR : tb_t2 : dut.m_tdata[15:0] : expected_value: 0 actual_value: %0d",
                   $time, r);
          $display("  [FAIL] T2 real[%0d] = %0d  expected ~0", i2, r);
          record_fail("TEST2-90deg-real");
          pass_r = 1'b0;
        end

        if (pass_i && (absdiff(im, 1000) > 5)) begin
          $display("LOG: %0t : ERROR : tb_t2 : dut.m_tdata[31:16] : expected_value: 1000 actual_value: %0d",
                   $time, im);
          $display("  [FAIL] T2 imag[%0d] = %0d  expected ~1000", i2, im);
          record_fail("TEST2-90deg-imag");
          pass_i = 1'b0;
        end
      end

      if (pass_r) begin
        $display("  [PASS] T2 real ~ 0 (+-5)");
      end
      if (pass_i) begin
        $display("  [PASS] T2 imag ~ 1000 (+-5)");
      end

      // Restore identity
      load_phase_ram(0, 64, COS_ONE, SIN_ZERO);
      repeat (4) @(posedge clk);
    end
  endtask

  // TEST 3: 45-degree rotation - real_out ~ imag_out ~ 7071
  task automatic run_test3;
    int r;
    int im;
    begin
      $display("");
      $display("=== TEST 3: 45-degree rotation ===");

      // Ensure sample index starts from 0 for this test
      realign_phase_index();

      load_phase_ram(0, PHASE_DEPTH, COS_ONE, SIN_ZERO);
      load_phase_ram(0, 1, COS45, SIN45);
      repeat (4) @(posedge clk);
      m_tready = 1'b1;
      bypass   = 1'b0;

      fork
        send_const(16'sd10000, 1, 0);
        collect_n(1);
      join

      r  = $signed(cbuf[0][DATA_W-1:0]);
      im = $signed(cbuf[0][DATA_W*2-1:DATA_W]);

      if (absdiff(r, 7071) > 50) begin
        $display("LOG: %0t : ERROR : tb_t3 : dut.m_tdata[15:0] : expected_value: 7071 actual_value: %0d",
                 $time, r);
        $display("  [FAIL] T3 real = %0d  expected ~7071  (diff=%0d)", r, absdiff(r, 7071));
        record_fail("TEST3-45deg-real");
      end else begin
        $display("  [PASS] T3 real = %0d  expected ~7071", r);
      end

      if (absdiff(im, 7071) > 50) begin
        $display("LOG: %0t : ERROR : tb_t3 : dut.m_tdata[31:16] : expected_value: 7071 actual_value: %0d",
                 $time, im);
        $display("  [FAIL] T3 imag = %0d  expected ~7071  (diff=%0d)", im, absdiff(im, 7071));
        record_fail("TEST3-45deg-imag");
      end else begin
        $display("  [PASS] T3 imag = %0d  expected ~7071", im);
      end

      // Restore identity
      load_phase_ram(0, PHASE_DEPTH, COS_ONE, SIN_ZERO);
      repeat (4) @(posedge clk);
    end
  endtask

  // TEST 4: Bypass mode - m_tdata = {16'b0, s_tdata}
  task automatic run_test4;
    logic [DATA_W-1:0] inp_arr [16];
    logic [31:0] expected;
    int i4;
    int k4;
    logic pass;
    begin
      $display("");
      $display("=== TEST 4: Bypass mode ===");
      pass = 1'b1;

      for (k4 = 0; k4 < 16; k4 = k4 + 1) begin
        inp_arr[k4] = DATA_W'(k4 + 1);
      end

      m_tready = 1'b1;
      bypass   = 1'b1;

      fork
        begin
          int i_s;
          i_s = 0;
          while (i_s < 16) begin
            @(negedge clk);
            s_tdata  = inp_arr[i_s];
            s_tvalid = 1'b1;
            s_tlast  = (i_s == 15);
            s_tuser  = '0;
            @(posedge clk); #1;
            if (s_tready) begin
              i_s = i_s + 1;
            end
          end
          @(negedge clk);
          s_tvalid = 1'b0;
          s_tlast  = 1'b0;
          s_tuser  = '0;
          s_tdata  = '0;
        end
        collect_n(16);
      join

      for (i4 = 0; i4 < 16; i4 = i4 + 1) begin
        expected = {16'b0, inp_arr[i4]};
        if (cbuf[i4] !== expected) begin
          $display("LOG: %0t : ERROR : tb_t4 : dut.m_tdata : expected_value: 32'h%08h actual_value: 32'h%08h",
                   $time, expected, cbuf[i4]);
          $display("  [FAIL] T4 bypass out[%0d] = 32'h%08h  expected 32'h%08h",
                   i4, cbuf[i4], expected);
          record_fail("TEST4-bypass-data");
          pass = 1'b0;
        end
      end

      if (pass) begin
        $display("  [PASS] T4 all 16 bypass samples = {16'b0, s_tdata}");
      end

      bypass = 1'b0;
    end
  endtask

  // TEST 5: Pipeline latency - non-zero output appears exactly PIPE_LAT cycles after input
  task automatic run_test5;
    longint in_time;
    longint out_time;
    int lag_cycles;
    logic [DATA_W-1:0] nz;
    int i5;
    begin
      $display("");
      $display("=== TEST 5: Pipeline latency ===");
      in_time    = -1;
      out_time   = -1;
      lag_cycles = 0;
      nz         = 16'sd12345;

      m_tready = 1'b1;
      bypass   = 1'b0;

      // Ensure sample index starts from 0 for this test
      realign_phase_index();

      // Prime pipeline with leading zeros
      i5 = 0;
      while (i5 < PIPE_LAT) begin
        @(negedge clk);
        s_tdata  = '0;
        s_tvalid = 1'b1;
        s_tlast  = 1'b0;
        s_tuser  = '0;
        @(posedge clk); #1;
        if (s_tready) begin
          i5 = i5 + 1;
        end
      end

      // Send non-zero impulse and record time it is accepted
      @(negedge clk);
      s_tdata  = nz;
      s_tvalid = 1'b1;
      s_tlast  = 1'b0;
      s_tuser  = '0;
      @(posedge clk); #1;
      while (!s_tready) begin
        @(posedge clk); #1;
      end
      in_time = $time;
      $display("  Impulse accepted at time %0t ns", in_time);

      fork
        begin
          int i_tr;
          @(negedge clk);
          s_tdata = '0;
          i_tr = 0;
          while (i_tr < PIPE_LAT + 6) begin
            @(negedge clk);
            s_tdata  = '0;
            s_tvalid = 1'b1;
            s_tlast  = 1'b0;
            s_tuser  = '0;
            @(posedge clk); #1;
            if (s_tready) begin
              i_tr = i_tr + 1;
            end
          end
          @(negedge clk);
          s_tvalid = 1'b0;
          s_tlast  = 1'b0;
          s_tuser  = '0;
          s_tdata  = '0;
        end
        begin
          int gd;
          int c;
          gd = PIPE_LAT * 5 + 10;
          for (c = 0; c < gd; c = c + 1) begin
            @(posedge clk); #1;
            if (m_tvalid && m_tready &&
                ($signed(m_tdata[DATA_W-1:0]) != 0) &&
                (out_time < 0)) begin
              out_time = $time;
            end
          end
        end
      join

      if (out_time < 0) begin
        $display("LOG: %0t : ERROR : tb_t5 : dut.m_tdata[15:0] : expected_value: non-zero actual_value: 0",
                 $time);
        $display("  [FAIL] T5 non-zero output never observed");
        record_fail("TEST5-latency-no-output");
      end else begin
        lag_cycles = int'((out_time - in_time) / $rtoi(CLK_PERIOD));
        $display("  Impulse output  at time %0t ns", out_time);
        $display("  Measured lag = %0d cycles  (expected %0d)", lag_cycles, PIPE_LAT);
        if (lag_cycles !== PIPE_LAT) begin
          $display("LOG: %0t : ERROR : tb_t5 : dut.pipeline_latency : expected_value: %0d actual_value: %0d",
                   $time, PIPE_LAT, lag_cycles);
          $display("  [FAIL] T5 latency = %0d  expected %0d", lag_cycles, PIPE_LAT);
          record_fail("TEST5-pipeline-latency");
        end else begin
          $display("  [PASS] T5 pipeline latency = %0d cycles", lag_cycles);
        end
      end
    end
  endtask

  // TEST 6: tlast/tuser passthrough - m_tlast fires PIPE_LAT cycles after s_tlast
  task automatic run_test6;
    longint in_tlast_t  [2];
    longint out_tlast_t [2];
    int in_cnt;
    int out_cnt;
    int PKT_LEN;
    int NUM_PKTS;
    int timeout_val;
    int dc;
    int k6;
    logic pass;
    begin
      $display("");
      $display("=== TEST 6: tlast/tuser passthrough ===");
      PKT_LEN     = 32;
      NUM_PKTS    = 2;
      in_cnt      = 0;
      out_cnt     = 0;
      pass        = 1'b1;

      // Ensure sample index starts from 0 for this test
      realign_phase_index();

      for (k6 = 0; k6 < NUM_PKTS; k6 = k6 + 1) begin
        in_tlast_t[k6]  = -1;
        out_tlast_t[k6] = -1;
      end

      m_tready = 1'b1;
      bypass   = 1'b0;

      fork
        begin
          int pkt;
          int s;
          for (pkt = 0; pkt < NUM_PKTS; pkt = pkt + 1) begin
            s = 0;
            while (s < PKT_LEN) begin
              @(negedge clk);
              s_tdata  = DATA_W'(pkt * PKT_LEN + s);
              s_tvalid = 1'b1;
              s_tlast  = (s == PKT_LEN - 1);
              s_tuser  = 32'(pkt);
              @(posedge clk); #1;
              if (s_tready) begin
                if (s_tlast && (in_cnt < NUM_PKTS)) begin
                  in_tlast_t[in_cnt] = $time;
                  in_cnt = in_cnt + 1;
                end
                s = s + 1;
              end
            end
          end
          @(negedge clk);
          s_tvalid = 1'b0;
          s_tlast  = 1'b0;
          s_tuser  = '0;
          s_tdata  = '0;
        end
        begin
          int c;
          timeout_val = NUM_PKTS * PKT_LEN * 4 + PIPE_LAT * 4 + 20;
          for (c = 0; (c < timeout_val) && (out_cnt < NUM_PKTS); c = c + 1) begin
            @(posedge clk); #1;
            if (m_tvalid && m_tready && m_tlast && (out_cnt < NUM_PKTS)) begin
              out_tlast_t[out_cnt] = $time;
              out_cnt = out_cnt + 1;
            end
          end
        end
      join

      for (k6 = 0; k6 < NUM_PKTS; k6 = k6 + 1) begin
        if (out_tlast_t[k6] < 0) begin
          $display("LOG: %0t : ERROR : tb_t6 : dut.m_tlast : expected_value: 1 actual_value: not_seen",
                   $time);
          $display("  [FAIL] T6 packet %0d: m_tlast never observed", k6);
          record_fail("TEST6-tlast-missing");
          pass = 1'b0;
        end else begin
          dc = int'((out_tlast_t[k6] - in_tlast_t[k6]) / $rtoi(CLK_PERIOD));
          if (dc !== PIPE_LAT) begin
            $display("LOG: %0t : ERROR : tb_t6 : dut.m_tlast_offset : expected_value: %0d actual_value: %0d",
                     $time, PIPE_LAT, dc);
            $display("  [FAIL] T6 pkt %0d: tlast lag = %0d cycles  expected %0d",
                     k6, dc, PIPE_LAT);
            record_fail("TEST6-tlast-offset");
            pass = 1'b0;
          end else begin
            $display("  [PASS] T6 pkt %0d: m_tlast fires %0d cycles after s_tlast", k6, dc);
          end
        end
      end
    end
  endtask

  // ===========================================================================
  // MAIN
  // ===========================================================================
  initial begin : main_test
    $display("TEST START");

    // Reset sequence
    @(negedge clk);
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    // Preload entire phase RAM with identity (cos~1, sin=0)
    load_phase_ram(0, PHASE_DEPTH, COS_ONE, SIN_ZERO);
    repeat (4) @(posedge clk);

    // Start from a known phase index
    realign_phase_index();

    run_test1;
    flush_pipeline;

    run_test2;
    flush_pipeline;

    run_test3;
    flush_pipeline;

    run_test4;
    flush_pipeline;

    run_test5;
    flush_pipeline;

    run_test6;

    // Final verdict
    repeat (20) @(posedge clk);
    $display("");
    if (fail_count == 0) begin
      $display("TEST PASSED");
      $display("TB_DISP_COMP: PASS");
    end else begin
      $display("ERROR");
      $display("TB_DISP_COMP: FAIL - %s", fail_msg);
      $error("TB_DISP_COMP: FAIL - %s", fail_msg);
    end

    $finish;
  end

  // ---------------------------------------------------------------------------
  // Watchdog
  // ---------------------------------------------------------------------------
  initial begin : watchdog
    #10_000_000;
    $display("ERROR");
    $display("TB_DISP_COMP: FAIL - TIMEOUT (10 ms wall exceeded)");
    $fatal(1, "TB_DISP_COMP: TIMEOUT");
  end

  // ---------------------------------------------------------------------------
  // Waveform dump
  // ---------------------------------------------------------------------------
  initial begin
    $dumpfile("dumpfile.fst");
    $dumpvars(0);
  end

endmodule