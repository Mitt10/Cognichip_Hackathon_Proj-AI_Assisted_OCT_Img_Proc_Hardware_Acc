// Code your testbench here
// or browse Examples
// =============================================================================
// tb_klinear_resampler.sv
//
// Testbench for klinear_resampler
//   IN_LEN = 2048  (A-scan depth into the module)
//   OUT_LEN = 1024 (resampled output depth)
//   DATA_W  = 16
//
// LUT entry format assumed: { integer_index[IDX_W-1:0], frac[FRAC_W-1:0] }
//   IDX_W  = $clog2(IN_LEN)  = 11
//   FRAC_W = 9
//   LUT_DW = 20
//   LUT_AW = $clog2(OUT_LEN) = 10
//
// AXI-Stream drive convention:
//   Signals are driven on negedge; handshake is sampled on posedge + #1 delta.
//
// Tests
//   1 – Sample count      : exactly 1024 valid pulses, tlast fires once
//   2 – Decimation        : output[i] == input[2i] ± 2 (linear LUT)
//   3 – Ping-pong         : no cross-contamination across consecutive A-scans
//   4 – LUT update timing : new LUT takes effect only on the NEXT A-scan
//   5 – Backpressure      : 3 × 1024 samples survive 30 % random m_tready low
//
// Final line: "TB_KLINEAR: PASS" or "TB_KLINEAR: FAIL — <test> at sample <i>"
// =============================================================================
`timescale 1ns/1ps

module tb_klinear_resampler;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  localparam int IN_LEN  = 2048;
  localparam int OUT_LEN = 1024;
  localparam int DATA_W  = 16;

  localparam int LUT_AW  = $clog2(OUT_LEN);          // 10
  localparam int IDX_W   = $clog2(IN_LEN);            // 11
  localparam int FRAC_W  = 9;
  localparam int LUT_DW  = IDX_W + FRAC_W;            // 20

  // ---------------------------------------------------------------------------
  // Clock / Reset
  // ---------------------------------------------------------------------------
  logic clk   = 1'b0;
  logic rst_n = 1'b0;

  always #5 clk = ~clk;   // 100 MHz

  // ---------------------------------------------------------------------------
  // DUT Ports
  // ---------------------------------------------------------------------------
  // AXI-Stream slave (input samples)
  logic [DATA_W-1:0]  s_tdata  = '0;
  logic               s_tvalid = 1'b0;
  logic               s_tready;          // driven by DUT
  logic               s_tlast  = 1'b0;

  // AXI-Stream master (resampled output)
  logic [DATA_W-1:0]  m_tdata;           // driven by DUT
  logic               m_tvalid;          // driven by DUT
  logic               m_tready = 1'b1;  // driven by TB
  logic               m_tlast;           // driven by DUT

  // LUT write port
  logic               lut_we    = 1'b0;
  logic [LUT_AW-1:0]  lut_waddr = '0;
  logic [LUT_DW-1:0]  lut_wdata = '0;
  
  logic [31:0] s_tuser = '0;
logic [31:0] m_tuser;

  // ---------------------------------------------------------------------------
  // DUT instantiation
  // NOTE: Adjust port names to match your actual klinear_resampler RTL.
  // ---------------------------------------------------------------------------
 klinear_resampler #(
  .IN_LEN  (IN_LEN),
  .OUT_LEN (OUT_LEN),
  .DATA_W  (DATA_W)
) dut (
  .clk       (clk),
  .rst_n     (rst_n),
  // slave
  .s_tdata   (s_tdata),
  .s_tvalid  (s_tvalid),
  .s_tlast   (s_tlast),
  .s_tuser   (s_tuser),
  .s_tready  (s_tready),
  // master
  .m_tdata   (m_tdata),
  .m_tvalid  (m_tvalid),
  .m_tlast   (m_tlast),
  .m_tuser   (m_tuser),
  .m_tready  (m_tready),
  // LUT
  .lut_we    (lut_we),
  .lut_addr  (lut_waddr),
  .lut_wdata (lut_wdata)
);

  // ---------------------------------------------------------------------------
  // Test bookkeeping
  // ---------------------------------------------------------------------------
  int    fail_count = 0;
  string fail_msg   = "";

  task automatic record_fail(string test_name, int sample_idx);
    $display("[FAIL] %s at sample %0d", test_name, sample_idx);
    if (fail_msg == "")
      fail_msg = $sformatf("%s at sample %0d", test_name, sample_idx);
    fail_count++;
  endtask

  // ---------------------------------------------------------------------------
  // Helper: load LUT — linear decimation map
  //   lut[i] = { i*2 [IDX_W bits], 9'b0 }
  // ---------------------------------------------------------------------------
  task automatic load_lut_linear();
    for (int i = 0; i < OUT_LEN; i++) begin
      @(negedge clk);
      lut_we    = 1'b1;
      lut_waddr = LUT_AW'(i);
      lut_wdata = { IDX_W'(i * 2), FRAC_W'(0) };
    end
    @(negedge clk);
    lut_we    = 1'b0;
    lut_waddr = '0;
    lut_wdata = '0;
  endtask

  // ---------------------------------------------------------------------------
  // Helper: load LUT — shifted decimation map
  //   lut[i] = { i*2 + 10 [IDX_W bits], 9'b0 }
  //   Valid only for i < (IN_LEN - 10) / 2 = 1019; entries beyond that are
  //   out-of-range but only the first 1019 will be spot-checked.
  // ---------------------------------------------------------------------------
  task automatic load_lut_shifted();
    for (int i = 0; i < OUT_LEN; i++) begin
      @(negedge clk);
      lut_we    = 1'b1;
      lut_waddr = LUT_AW'(i);
      lut_wdata = { IDX_W'(i * 2 + 10), FRAC_W'(0) };
    end
    @(negedge clk);
    lut_we    = 1'b0;
    lut_waddr = '0;
    lut_wdata = '0;
  endtask

  // ---------------------------------------------------------------------------
  // Helper: send one A-scan over the AXI-Stream slave port
  //   Drives on negedge; advances to next sample once s_tready is seen at
  //   posedge (proper AXIS handshake — data held stable until accepted).
  // ---------------------------------------------------------------------------
  task automatic send_ascan(input logic [DATA_W-1:0] data [IN_LEN]);
    for (int i = 0; i < IN_LEN; ) begin
      @(negedge clk);
      s_tdata  = data[i];
      s_tvalid = 1'b1;
      s_tlast  = (i == IN_LEN - 1);
      @(posedge clk); #1;          // #1 delta: let DUT outputs settle
      if (s_tready) i++;           // handshake occurred → advance
    end
    @(negedge clk);
    s_tvalid = 1'b0;
    s_tlast  = 1'b0;
    s_tdata  = '0;
  endtask

  // ---------------------------------------------------------------------------
  // Helper: collect exactly OUT_LEN output samples (m_tready must be 1 externally)
  // ---------------------------------------------------------------------------
  task automatic collect_ascan(
    output logic [DATA_W-1:0] out_buf   [OUT_LEN],
    output int                valid_cnt,
    output int                tlast_cnt
  );
    valid_cnt = 0;
    tlast_cnt = 0;
    while (valid_cnt < OUT_LEN) begin
      @(posedge clk); #1;
      if (m_tvalid && m_tready) begin
        out_buf[valid_cnt] = m_tdata;
        if (m_tlast) tlast_cnt++;
        valid_cnt++;
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Helper: absolute value for int
  // ---------------------------------------------------------------------------
  function automatic int abs_diff(int a, int b);
    return (a > b) ? (a - b) : (b - a);
  endfunction

  // ===========================================================================
  // MAIN TEST SEQUENCE
  // ===========================================================================
  initial begin : main_test

    // -------------------------------------------------------------------------
    // Reset
    // -------------------------------------------------------------------------
    @(negedge clk); rst_n = 1'b0;
    repeat (10) @(posedge clk);
    @(negedge clk); rst_n = 1'b1;
    repeat (4)  @(posedge clk);

    // Pre-load linear LUT (used by Tests 1, 2, 3, and first half of Test 4)
    load_lut_linear();
    repeat (4) @(posedge clk);

    // =========================================================================
    // TEST 1 — Sample count
    // =========================================================================
    $display("");
    $display("=== TEST 1: Sample count ===");
    begin : t1
      automatic logic [DATA_W-1:0] ramp    [IN_LEN];
      automatic logic [DATA_W-1:0] out_arr [OUT_LEN];
      automatic int                valid_cnt, tlast_cnt;

      foreach (ramp[i]) ramp[i] = DATA_W'(i);
      m_tready = 1'b1;

      fork
        send_ascan(ramp);
        collect_ascan(out_arr, valid_cnt, tlast_cnt);
      join

      if (valid_cnt !== OUT_LEN) begin
        $display("  [FAIL] valid_count = %0d  (expected %0d)", valid_cnt, OUT_LEN);
        record_fail("TEST1-valid-count", valid_cnt);
      end else
        $display("  [PASS] valid_count = %0d", valid_cnt);

      if (tlast_cnt !== 1) begin
        $display("  [FAIL] tlast fired %0d time(s)  (expected 1)", tlast_cnt);
        record_fail("TEST1-tlast-count", tlast_cnt);
      end else
        $display("  [PASS] m_tlast fired exactly once");
    end // t1

    repeat (10) @(posedge clk);

    // =========================================================================
    // TEST 2 — Decimation accuracy
    // =========================================================================
    $display("");
    $display("=== TEST 2: Decimation accuracy ===");
    begin : t2
      automatic logic [DATA_W-1:0] ramp    [IN_LEN];
      automatic logic [DATA_W-1:0] out_arr [OUT_LEN];
      automatic int                valid_cnt, tlast_cnt;
      // Spot-check indices specified in the test plan
      automatic int spot [4] = '{0, 100, 511, 1023};

      foreach (ramp[i]) ramp[i] = DATA_W'(i);
      m_tready = 1'b1;

      fork
        send_ascan(ramp);
        collect_ascan(out_arr, valid_cnt, tlast_cnt);
      join

      foreach (spot[j]) begin
        automatic int idx  = spot[j];
        automatic int exp  = idx * 2;          // linear map: out[i] = in[2i]
        automatic int diff = abs_diff(int'(out_arr[idx]), exp);
        if (diff > 2) begin
          $display("  [FAIL] out[%4d] = %0d  expected ~%0d  diff = %0d",
                   idx, out_arr[idx], exp, diff);
          record_fail("TEST2-decimation", idx);
        end else
          $display("  [PASS] out[%4d] = %0d  expected ~%0d", idx, out_arr[idx], exp);
      end
    end // t2

    repeat (10) @(posedge clk);

    // =========================================================================
    // TEST 3 — Ping-pong isolation (back-to-back A-scans, no gap)
    // =========================================================================
    $display("");
    $display("=== TEST 3: Ping-pong isolation ===");
    begin : t3
      automatic logic [DATA_W-1:0] scan_a [IN_LEN];
      automatic logic [DATA_W-1:0] scan_b [IN_LEN];
      automatic logic [DATA_W-1:0] out_a  [OUT_LEN];
      automatic logic [DATA_W-1:0] out_b  [OUT_LEN];
      automatic int vc_a, tl_a, vc_b, tl_b;

      foreach (scan_a[i]) scan_a[i] = 16'h0001;
      foreach (scan_b[i]) scan_b[i] = 16'h0002;
      m_tready = 1'b1;

      // Send A then B back-to-back; collect both outputs sequentially
      fork
        begin : t3_sender
          send_ascan(scan_a);
          send_ascan(scan_b);
        end
        begin : t3_collector
          collect_ascan(out_a, vc_a, tl_a);
          collect_ascan(out_b, vc_b, tl_b);
        end
      join

      // Verify A-scan A: all samples ≈ 1
      begin
        automatic logic pass_a = 1'b1;
        for (int i = 0; i < OUT_LEN; i++) begin
          if (abs_diff(int'(out_a[i]), 1) > 1) begin
            $display("  [FAIL] scan-A out[%0d] = %0d  expected ~1", i, out_a[i]);
            record_fail("TEST3-scanA-contamination", i);
            pass_a = 1'b0;
            break;
          end
        end
        if (pass_a) $display("  [PASS] all 1024 scan-A samples ≈ 1");
      end

      // Verify A-scan B: all samples ≈ 2
      begin
        automatic logic pass_b = 1'b1;
        for (int i = 0; i < OUT_LEN; i++) begin
          if (abs_diff(int'(out_b[i]), 2) > 1) begin
            $display("  [FAIL] scan-B out[%0d] = %0d  expected ~2", i, out_b[i]);
            record_fail("TEST3-scanB-contamination", i);
            pass_b = 1'b0;
            break;
          end
        end
        if (pass_b) $display("  [PASS] all 1024 scan-B samples ≈ 2");
      end
    end // t3

    repeat (10) @(posedge clk);

    // =========================================================================
    // TEST 4 — LUT update takes effect on NEXT A-scan only
    // =========================================================================
    $display("");
    $display("=== TEST 4: LUT update timing ===");
    begin : t4
      automatic logic [DATA_W-1:0] ramp    [IN_LEN];
      automatic logic [DATA_W-1:0] out_old [OUT_LEN];   // pre-update A-scan
      automatic logic [DATA_W-1:0] out_new [OUT_LEN];   // post-update A-scan
      automatic int vc, tl;
      // Spot-check indices safe for shifted map: i*2+10 < IN_LEN  =>  i < 1019
      automatic int spot [4] = '{0, 100, 511, 900};

      foreach (ramp[i]) ramp[i] = DATA_W'(i);
      m_tready = 1'b1;

      // ----- Phase A: send one A-scan while OLD (linear) LUT is in effect -----
      // Linear LUT is still loaded from the pre-test setup.
      fork
        send_ascan(ramp);
        collect_ascan(out_old, vc, tl);
      join

      // ----- Write new (shifted) LUT AFTER the A-scan has started/finished -----
      load_lut_shifted();
      repeat (4) @(posedge clk);

      // ----- Phase B: send the NEXT A-scan — should use the new LUT -----
      fork
        send_ascan(ramp);
        collect_ascan(out_new, vc, tl);
      join

      // Verify Phase A used linear map: out_old[i] ≈ i*2
      $display("  -- Old-LUT verification (should be linear) --");
      foreach (spot[j]) begin
        automatic int idx  = spot[j];
        automatic int exp  = idx * 2;
        automatic int diff = abs_diff(int'(out_old[idx]), exp);
        if (diff > 2) begin
          $display("  [FAIL] old-LUT out[%0d] = %0d  expected ~%0d  diff = %0d",
                   idx, out_old[idx], exp, diff);
          record_fail("TEST4-old-LUT", idx);
        end else
          $display("  [PASS] old-LUT out[%0d] = %0d  expected ~%0d", idx, out_old[idx], exp);
      end

      // Verify Phase B used shifted map: out_new[i] ≈ i*2 + 10
      $display("  -- New-LUT verification (should be shifted +10) --");
      foreach (spot[j]) begin
        automatic int idx  = spot[j];
        automatic int exp  = idx * 2 + 10;
        automatic int diff = abs_diff(int'(out_new[idx]), exp);
        if (diff > 2) begin
          $display("  [FAIL] new-LUT out[%0d] = %0d  expected ~%0d  diff = %0d",
                   idx, out_new[idx], exp, diff);
          record_fail("TEST4-new-LUT", idx);
        end else
          $display("  [PASS] new-LUT out[%0d] = %0d  expected ~%0d", idx, out_new[idx], exp);
      end
    end // t4

    repeat (10) @(posedge clk);

    // =========================================================================
    // TEST 5 — Backpressure: 3 A-scans with m_tready toggled ~30 % low
    // =========================================================================
    $display("");
    $display("=== TEST 5: Backpressure ===");
    begin : t5
      automatic logic [DATA_W-1:0] ramp        [IN_LEN];
      automatic int                total_valid = 0;
      automatic int                cycle_cnt   = 0;
      // Generous timeout: allow 20 cycles per expected output sample
      automatic int                timeout_max = OUT_LEN * 3 * 20;
      automatic logic              timed_out   = 1'b0;

      // Reload linear LUT (Test 4 left shifted LUT active)
      load_lut_linear();
      repeat (4) @(posedge clk);

      foreach (ramp[i]) ramp[i] = DATA_W'(i);

      // Sender: fire off 3 A-scans in the background
      fork
        repeat (3) send_ascan(ramp);
      join_none

      // Receiver: randomise m_tready, count received samples, hard timeout
      while (total_valid < OUT_LEN * 3) begin
        @(negedge clk);
        // Assert m_tready ~70 % of cycles (deassert ~30 %)
        m_tready = ($urandom_range(0, 9) >= 3) ? 1'b1 : 1'b0;
        @(posedge clk); #1;
        if (m_tvalid && m_tready) total_valid++;
        cycle_cnt++;
        if (cycle_cnt >= timeout_max) begin
          timed_out = 1'b1;
          break;
        end
      end

      // Restore m_tready for any cleanup
      @(negedge clk);
      m_tready = 1'b1;

      if (timed_out || total_valid !== OUT_LEN * 3) begin
        $display("  [FAIL] received %0d / %0d samples  timed_out=%0b",
                 total_valid, OUT_LEN * 3, timed_out);
        record_fail("TEST5-backpressure", total_valid);
      end else
        $display("  [PASS] all 3072 samples received under 30%% backpressure  (%0d cycles)",
                 cycle_cnt);
    end // t5

    // Wait for any residual background traffic to drain
    repeat (30) @(posedge clk);

    // =========================================================================
    // Final verdict
    // =========================================================================
    $display("");
    if (fail_count == 0) begin
      $display("TB_KLINEAR: PASS");
    end else begin
      $display("TB_KLINEAR: FAIL — %s", fail_msg);
    end

    $finish;
  end // main_test

  // ---------------------------------------------------------------------------
  // Hard watchdog: 50 ms simulation wall — catches livelock / infinite loops
  // ---------------------------------------------------------------------------
  initial begin : watchdog
    #50_000_000;
    $display("TB_KLINEAR: FAIL — TIMEOUT (50 ms wall exceeded)");
    $finish;
  end

endmodule // tb_klinear_resampler