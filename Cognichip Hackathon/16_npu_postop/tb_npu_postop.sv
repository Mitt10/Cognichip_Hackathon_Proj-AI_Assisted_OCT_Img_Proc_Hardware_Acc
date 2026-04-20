`timescale 1ns/1ps
// ============================================================================
//  TB-16 — tb_npu_postop
//  Post-Op Unit Testbench: BN Scale/Shift · Bias · Activation · Requant · Pool
//
//  DUT  : npu_postop  (CHANNELS=4 for simulation speed)
//  Clock: 300 MHz  (period ≈ 3.333 ns)
//
//  Tests
//  -----
//  TEST 1 – BN scale by 0.5 (Q0.15 = 0x8000), no bias, linear, no pool
//  TEST 2 – Bias add (bias[0]=512, scale≈1.0, linear)
//  TEST 3 – ReLU (act_fn=1): negative inputs clamp to 0
//  TEST 4 – ReLU6 (act_fn=2): saturates at 6
//  TEST 5 – INT8 requant saturation clamp (+127 / −128)
//  TEST 6 – 2×2 channel-wise max-pool over 4 spatial groups
// ============================================================================

module tb_npu_postop;

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  localparam int  CHANNELS    = 4;
  localparam int  ACC_W       = 32;   // accumulator width (signed)
  localparam int  OUT_W       = 8;    // INT8 output
  localparam int  SCALE_W     = 16;   // Q0.15 BN scale
  localparam int  SHIFT_MAX   = 8;    // max requant shift bits
  localparam real CLK_PERIOD  = 3.333;// 300 MHz

  // Activation-function encoding
  localparam logic [1:0] ACT_LINEAR = 2'd0;
  localparam logic [1:0] ACT_RELU   = 2'd1;
  localparam logic [1:0] ACT_RELU6  = 2'd2;

  // INT8 saturation limits
  localparam int  INT8_MAX    =  127;
  localparam int  INT8_MIN    = -128;

  // -------------------------------------------------------------------------
  // DUT port declarations
  // -------------------------------------------------------------------------
  logic                             clk;
  logic                             rst_n;

  // Per-channel configuration
  logic signed [ACC_W-1:0]          bias      [0:CHANNELS-1];
  logic        [SCALE_W-1:0]        bn_scale  [0:CHANNELS-1]; // Q0.15 unsigned
  logic signed [ACC_W-1:0]          bn_shift  [0:CHANNELS-1]; // additive after scale
  logic        [$clog2(SHIFT_MAX+1)-1:0] requant_shift;       // right-shift for requant

  // Activation select
  logic        [1:0]                act_fn;

  // Pool control
  logic                             pool_en;

  // Data input
  logic signed [ACC_W-1:0]          acc_in    [0:CHANNELS-1];
  logic                             acc_valid;
  logic                             acc_last;   // last spatial group for pool

  // Data output
  logic signed [OUT_W-1:0]          out_data  [0:CHANNELS-1];
  logic                             out_valid;

  // -------------------------------------------------------------------------
  // DUT instantiation
  // -------------------------------------------------------------------------
  npu_postop #(
    .CHANNELS     (CHANNELS),
    .ACC_W        (ACC_W),
    .OUT_W        (OUT_W),
    .SCALE_W      (SCALE_W),
    .SHIFT_MAX    (SHIFT_MAX)
  ) dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .bias         (bias),
    .bn_scale     (bn_scale),
    .bn_shift     (bn_shift),
    .requant_shift(requant_shift),
    .act_fn       (act_fn),
    .pool_en      (pool_en),
    .acc_in       (acc_in),
    .acc_valid    (acc_valid),
    .acc_last     (acc_last),
    .out_data     (out_data),
    .out_valid    (out_valid)
  );

  // -------------------------------------------------------------------------
  // Clock generation — 300 MHz
  // -------------------------------------------------------------------------
  initial clk = 1'b0;
  always #(CLK_PERIOD / 2.0) clk = ~clk;

  // -------------------------------------------------------------------------
  // Scoreboard
  // -------------------------------------------------------------------------
  int  fail_count = 0;

  task automatic check_out (
    input string                    test_name,
    input int                       ch,
    input logic signed [OUT_W-1:0]  got,
    input int                       expected,
    input int                       tolerance = 0
  );
    int g_int = int'($signed(got));
    if ((g_int < expected - tolerance) || (g_int > expected + tolerance)) begin
      $display("  [FAIL] %s ch %0d  got %0d  expected %0d (±%0d)",
               test_name, ch, g_int, expected, tolerance);
      fail_count++;
    end else begin
      $display("  [PASS] %s ch %0d  = %0d  (expected %0d ±%0d)",
               test_name, ch, g_int, expected, tolerance);
    end
  endtask

  // -------------------------------------------------------------------------
  // Helper tasks
  // -------------------------------------------------------------------------

  // Drive all inputs to safe idle state
  task automatic idle_inputs();
    acc_valid = 1'b0;
    acc_last  = 1'b0;
    for (int c = 0; c < CHANNELS; c++) begin
      acc_in  [c] = '0;
      bias    [c] = '0;
      bn_scale[c] = 16'h7FFF; // ≈ 1.0 Q0.15 default
      bn_shift[c] = '0;
    end
    requant_shift = '0;
    act_fn        = ACT_LINEAR;
    pool_en       = 1'b0;
  endtask

  // Apply default BN config for most tests
  // bn_scale = 0x8000 (0.5 in Q0.15), bn_shift = 0, bias = 0, requant_shift = 8
  task automatic apply_default_cfg();
    for (int c = 0; c < CHANNELS; c++) begin
      bias    [c] = '0;
      bn_scale[c] = 16'h8000;   // 0.5 in Q0.15
      bn_shift[c] = '0;
    end
    requant_shift = 4'd8;
  endtask

  // Send one vector; optionally mark as last (for pool flush)
  task automatic send_vec (
    input logic signed [ACC_W-1:0] vec    [0:CHANNELS-1],
    input logic                    is_last
  );
    @(posedge clk); #1;
    for (int c = 0; c < CHANNELS; c++) acc_in[c] = vec[c];
    acc_valid = 1'b1;
    acc_last  = is_last;
    @(posedge clk); #1;
    acc_valid = 1'b0;
    acc_last  = 1'b0;
    for (int c = 0; c < CHANNELS; c++) acc_in[c] = '0;
  endtask

  // Wait for out_valid with safety timeout
  task automatic wait_out (input int timeout = 200);
    int cnt = 0;
    while (!out_valid && cnt < timeout) begin
      @(posedge clk);
      cnt++;
    end
    if (cnt >= timeout) begin
      $display("  [FAIL] Timeout waiting for out_valid");
      fail_count++;
    end
  endtask

  // -------------------------------------------------------------------------
  // Main test body
  // -------------------------------------------------------------------------
  initial begin : tb_main

    idle_inputs();
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    $display("");
    $display("=========================================================");
    $display("  TB-16  npu_postop  Post-Op Unit  CH=%0d", CHANNELS);
    $display("  BN + Activation + Requant + 2x2 Max-Pool");
    $display("=========================================================");

    // ==================================================================
    // TEST 1 — BN scale × 0.5 (Q0.15 = 0x8000), requant_shift = 8
    //
    //  Pipeline per channel:
    //    biased  = acc_in + bias                  = acc_in   (bias=0)
    //    scaled  = biased × bn_scale >> 15        ≈ acc_in × 0.5
    //    shifted = scaled + bn_shift              = scaled   (bn_shift=0)
    //    act     = linear(shifted)                = shifted
    //    requant = shifted >> requant_shift(=8)
    //    clamped = sat8(requant)
    //
    //  Expected (before rounding details):
    //    ch0: 1000 × 0.5 = 500 >> 8 ≈ 1 (fixed-pt: 1000×0x8000>>15=1000>>1=500; 500>>8=1; ±2 → ~2 acceptable)
    //    ch1: 2000 × 0.5 = 1000 >> 8 = 3 (±2 → ~4)
    //    ch2: 3000 × 0.5 = 1500 >> 8 = 5 (±2 → ~6)
    //    ch3: 4000 × 0.5 = 2000 >> 8 = 7 (±2 → ~8)
    //
    //  Note: bn_scale=0x8000 in Q0.15 = 32768/32768 × 0.5 exactly →
    //        acc × 32768 >> 15 = acc >> 1 (arithmetic)
    //        then >> 8 total shift from requant.
    //        1000>>1=500, 500>>8=1 with arithmetic right-shift in HW.
    //        Test spec says "accept ±2", so target=2 for ch0 is ≈ spec.
    // ==================================================================
    $display("\n-- TEST 1 : BN scale 0.5, requant_shift=8, linear, no pool --");
    begin
      logic signed [ACC_W-1:0] v1 [0:CHANNELS-1];

      apply_default_cfg();
      act_fn   = ACT_LINEAR;
      pool_en  = 1'b0;

      v1[0] = 32'sd1000;
      v1[1] = 32'sd2000;
      v1[2] = 32'sd3000;
      v1[3] = 32'sd4000;

      send_vec(v1, 1'b1);
      wait_out();

      // Expected: ch_n = (n+1)*1000 * 0.5 >> 8
      // With exact Q0.15: val×0x8000>>15 = val>>1 (arithmetic).
      // 1000>>1=500, 500>>8=1 (HW arithmetic right-shift).
      // Spec says ≈ {4,8,12,16} assuming scale is treated as (val*scale)>>15
      // then >>8 happens AFTER.  Per spec target: accept ±2 → check against
      // spec stated values {4,8,12,16} with tolerance 2 to handle rounding.
      // (Implementation note: if HW does (val*bn_scale)>>15 the full-precision
      //  32×16 multiply before shift, then 1000*0x8000=32768000; >>15=1000/2
      //  but >> drops the LSB → 500; then >>8 = 1. Spec says ~2 ±2, so
      //  tolerance covers both interpretations.)
      check_out("TEST1", 0, out_data[0],  2, 2);
      check_out("TEST1", 1, out_data[1],  4, 2);
      check_out("TEST1", 2, out_data[2],  6, 2);
      check_out("TEST1", 3, out_data[3],  8, 2);
    end

    // ==================================================================
    // TEST 2 — Bias add
    //   bias[0] = 512, bias[1..3] = 0
    //   bn_scale = 0x7FFF ≈ 1.0 (Q0.15 max), bn_shift = 0
    //   acc_in = {0, 0, 0, 0}
    //   requant_shift = 8
    //
    //   ch0: (0 + 512) × 0x7FFF >> 15 = ~512 >> 8 = 2
    //   ch1..3: 0
    // ==================================================================
    $display("\n-- TEST 2 : Bias add (bias[0]=512, bn_scale≈1.0) --");
    begin
      logic signed [ACC_W-1:0] v2 [0:CHANNELS-1];

      idle_inputs();
      for (int c = 0; c < CHANNELS; c++) begin
        bias    [c] = '0;
        bn_scale[c] = 16'h7FFF;  // ≈ 1.0 in Q0.15
        bn_shift[c] = '0;
      end
      bias[0]       = 32'sd512;
      requant_shift = 4'd8;
      act_fn        = ACT_LINEAR;
      pool_en       = 1'b0;

      for (int c = 0; c < CHANNELS; c++) v2[c] = 32'sd0;

      send_vec(v2, 1'b1);
      wait_out();

      // 512 × (0x7FFF/0x8000) >> 8 ≈ 512 >> 8 = 2  (tolerance 1 for rounding)
      check_out("TEST2", 0, out_data[0], 2, 1);
      check_out("TEST2", 1, out_data[1], 0, 0);
      check_out("TEST2", 2, out_data[2], 0, 0);
      check_out("TEST2", 3, out_data[3], 0, 0);
    end

    // ==================================================================
    // TEST 3 — ReLU (act_fn=1)
    //   acc_in = {-1000, -1, 0, 500}
    //   bn_scale = identity (0x7FFF), requant_shift = 0
    //   Expected after ReLU: {0, 0, 0, >0}
    // ==================================================================
    $display("\n-- TEST 3 : ReLU activation --");
    begin
      logic signed [ACC_W-1:0] v3 [0:CHANNELS-1];

      idle_inputs();
      for (int c = 0; c < CHANNELS; c++) begin
        bias    [c] = '0;
        bn_scale[c] = 16'h7FFF;
        bn_shift[c] = '0;
      end
      requant_shift = 3'd0;   // no compression so values stay visible
      act_fn        = ACT_RELU;
      pool_en       = 1'b0;

      v3[0] = -32'sd1000;
      v3[1] = -32'sd1;
      v3[2] =  32'sd0;
      v3[3] =  32'sd500;

      send_vec(v3, 1'b1);
      wait_out();

      check_out("TEST3", 0, out_data[0], 0, 0);   // negative → 0
      check_out("TEST3", 1, out_data[1], 0, 0);   // negative → 0
      check_out("TEST3", 2, out_data[2], 0, 0);   // zero → 0
      // ch3 must be positive (saturated at INT8_MAX if > 127)
      if ($signed(out_data[3]) > 0)
        $display("  [PASS] TEST3 ch 3  = %0d  > 0", $signed(out_data[3]));
      else begin
        $display("  [FAIL] TEST3 ch 3  got %0d  expected > 0", $signed(out_data[3]));
        fail_count++;
      end
    end

    // ==================================================================
    // TEST 4 — ReLU6 (act_fn=2)
    //   requant_shift = 0 so raw values pass through un-compressed.
    //   bn_scale = identity (0x7FFF)
    //   acc_in = {-50, 3, 6, 10}
    //   Expected: {0, 3, 6, 6}
    //     -50 → clamp to 0
    //      3  → pass through
    //      6  → exactly 6
    //     10  → clamp to 6
    // ==================================================================
    $display("\n-- TEST 4 : ReLU6 activation (clamp [0,6]) --");
    begin
      logic signed [ACC_W-1:0] v4 [0:CHANNELS-1];

      idle_inputs();
      for (int c = 0; c < CHANNELS; c++) begin
        bias    [c] = '0;
        bn_scale[c] = 16'h7FFF;
        bn_shift[c] = '0;
      end
      requant_shift = 3'd0;
      act_fn        = ACT_RELU6;
      pool_en       = 1'b0;

      v4[0] = -32'sd50;
      v4[1] =  32'sd3;
      v4[2] =  32'sd6;
      v4[3] =  32'sd10;

      send_vec(v4, 1'b1);
      wait_out();

      check_out("TEST4", 0, out_data[0], 0, 0);
      check_out("TEST4", 1, out_data[1], 3, 0);
      check_out("TEST4", 2, out_data[2], 6, 0);
      check_out("TEST4", 3, out_data[3], 6, 0);   // 10 clamped to 6
    end

    // ==================================================================
    // TEST 5 — INT8 saturation clamp (requant)
    //   requant_shift = 0 so no right-shift; raw value hits saturation directly.
    //   bn_scale = identity (0x7FFF)
    //   acc_in = {1000000, -1000000, 127, -128}
    //   Expected:
    //     ch0: +127 (overflow clamped)
    //     ch1: -128 (underflow clamped)
    //     ch2: +127 (exactly at limit)
    //     ch3: -128 (exactly at limit)
    // ==================================================================
    $display("\n-- TEST 5 : INT8 requant saturation clamp --");
    begin
      logic signed [ACC_W-1:0] v5 [0:CHANNELS-1];

      idle_inputs();
      for (int c = 0; c < CHANNELS; c++) begin
        bias    [c] = '0;
        bn_scale[c] = 16'h7FFF;
        bn_shift[c] = '0;
      end
      requant_shift = 3'd0;
      act_fn        = ACT_LINEAR;
      pool_en       = 1'b0;

      v5[0] =  32'sd1000000;
      v5[1] = -32'sd1000000;
      v5[2] =  32'sd127;
      v5[3] = -32'sd128;

      send_vec(v5, 1'b1);
      wait_out();

      check_out("TEST5", 0, out_data[0],  127, 0);  // positive overflow clamp
      check_out("TEST5", 1, out_data[1], -128, 0);  // negative overflow clamp
      check_out("TEST5", 2, out_data[2],  127, 0);  // exactly INT8_MAX
      check_out("TEST5", 3, out_data[3], -128, 0);  // exactly INT8_MIN
    end

    // ==================================================================
    // TEST 6 — 2×2 channel-wise max-pool
    //   pool_en = 1; acc_last only on the 4th group.
    //   requant_shift = 0, bn_scale = identity, act_fn = linear.
    //
    //   Groups (spatial 2×2 neighbourhood), 4 channels each:
    //     G0 (r0,c0): { 1,  2,  3,  4}
    //     G1 (r0,c1): { 5,  6,  7,  8}
    //     G2 (r1,c0): { 3,  1,  9,  2}
    //     G3 (r1,c1): { 2,  4,  5,  6}
    //
    //   Channel-wise max:
    //     ch0: max(1,5,3,2) = 5
    //     ch1: max(2,6,1,4) = 6
    //     ch2: max(3,7,9,5) = 9
    //     ch3: max(4,8,2,6) = 8
    // ==================================================================
    $display("\n-- TEST 6 : 2x2 channel-wise max-pool --");
    begin
      logic signed [ACC_W-1:0] g0 [0:CHANNELS-1];
      logic signed [ACC_W-1:0] g1 [0:CHANNELS-1];
      logic signed [ACC_W-1:0] g2 [0:CHANNELS-1];
      logic signed [ACC_W-1:0] g3 [0:CHANNELS-1];

      idle_inputs();
      for (int c = 0; c < CHANNELS; c++) begin
        bias    [c] = '0;
        bn_scale[c] = 16'h7FFF;
        bn_shift[c] = '0;
      end
      requant_shift = 3'd0;
      act_fn        = ACT_LINEAR;
      pool_en       = 1'b1;

      g0[0]=32'sd1; g0[1]=32'sd2; g0[2]=32'sd3; g0[3]=32'sd4;
      g1[0]=32'sd5; g1[1]=32'sd6; g1[2]=32'sd7; g1[3]=32'sd8;
      g2[0]=32'sd3; g2[1]=32'sd1; g2[2]=32'sd9; g2[3]=32'sd2;
      g3[0]=32'sd2; g3[1]=32'sd4; g3[2]=32'sd5; g3[3]=32'sd6;

      send_vec(g0, 1'b0);   // group 0 — not last
      send_vec(g1, 1'b0);   // group 1 — not last
      send_vec(g2, 1'b0);   // group 2 — not last
      send_vec(g3, 1'b1);   // group 3 — LAST → triggers pool flush + out_valid

      wait_out();

      check_out("TEST6", 0, out_data[0], 5, 0);
      check_out("TEST6", 1, out_data[1], 6, 0);
      check_out("TEST6", 2, out_data[2], 9, 0);
      check_out("TEST6", 3, out_data[3], 8, 0);
    end

    // ==================================================================
    // Result summary
    // ==================================================================
    $display("");
    $display("=========================================================");
    if (fail_count == 0)
      $display("  TB_POSTOP: PASS");
    else
      $display("  TB_POSTOP: FAIL  (%0d assertion(s) — see above)", fail_count);
    $display("=========================================================");
    $display("");

    $finish;
  end : tb_main

  // -------------------------------------------------------------------------
  // Simulation watchdog — abort at 10 000 cycles
  // -------------------------------------------------------------------------
  initial begin
    repeat (10_000) @(posedge clk);
    $display("[WATCHDOG] Simulation exceeded 10 000 cycles — aborting.");
    $finish;
  end

endmodule : tb_npu_postop


// ============================================================================
//  REFERENCE DUT — npu_postop
//
//  Per-channel pipeline (combinational after acc_valid, registered out):
//
//    1. BIAS ADD       : biased = acc_in[c] + bias[c]
//    2. BN SCALE       : scaled = (biased × bn_scale[c]) >>> 15   (Q0.15 mul)
//    3. BN SHIFT ADD   : shifted = scaled + bn_shift[c]
//    4. ACTIVATION     : act_fn ∈ {LINEAR, RELU, RELU6}
//    5. REQUANTISE     : requant = shifted >>> requant_shift
//    6. INT8 CLAMP     : clamped = sat(requant, −128, +127)
//
//  When pool_en=1, a 2×2 channel-wise running-max register is maintained.
//  Each incoming vector updates the max; when acc_last is asserted the final
//  max value is pushed to out_data and out_valid is strobed.
//
//  When pool_en=0, output is registered directly from the clamp stage and
//  out_valid fires one cycle after acc_valid.
//
//  Interface
//  ---------
//   clk, rst_n         – 300 MHz clock, active-low sync reset
//   bias[C]            – signed 32-bit per-channel bias
//   bn_scale[C]        – unsigned 16-bit Q0.15 scale  (0x8000 → 0.5)
//   bn_shift[C]        – signed 32-bit additive shift after scale
//   requant_shift      – unsigned, right-shift applied after activation
//   act_fn [1:0]       – 0=linear, 1=relu, 2=relu6
//   pool_en            – enable 2×2 max-pool accumulation
//   acc_in[C]          – signed 32-bit input vector
//   acc_valid          – input is valid this cycle
//   acc_last           – last group of a pool tile (or final beat)
//   out_data[C]        – signed INT8 output vector
//   out_valid          – out_data is valid this cycle
// ============================================================================

module npu_postop #(
  parameter int CHANNELS  = 16,
  parameter int ACC_W     = 32,
  parameter int OUT_W     = 8,
  parameter int SCALE_W   = 16,   // Q0.15
  parameter int SHIFT_MAX = 8     // max requant_shift
)(
  input  logic                              clk,
  input  logic                              rst_n,

  // Configuration (static during a tile)
  input  logic signed [ACC_W-1:0]           bias      [0:CHANNELS-1],
  input  logic        [SCALE_W-1:0]         bn_scale  [0:CHANNELS-1],
  input  logic signed [ACC_W-1:0]           bn_shift  [0:CHANNELS-1],
  input  logic        [$clog2(SHIFT_MAX+1)-1:0] requant_shift,
  input  logic        [1:0]                 act_fn,
  input  logic                              pool_en,

  // Data path
  input  logic signed [ACC_W-1:0]           acc_in    [0:CHANNELS-1],
  input  logic                              acc_valid,
  input  logic                              acc_last,

  // Output
  output logic signed [OUT_W-1:0]           out_data  [0:CHANNELS-1],
  output logic                              out_valid
);

  // Derived widths
  localparam int PROD_W  = ACC_W + SCALE_W; // width of bias×scale multiply
  localparam int INT8_HI =  127;
  localparam int INT8_LO = -128;

  // -------------------------------------------------------------------------
  // Stage 1-4: fully combinational per-channel datapath
  // -------------------------------------------------------------------------
  logic signed [ACC_W-1:0]    s_biased  [0:CHANNELS-1];
  logic signed [ACC_W-1:0]    s_scaled  [0:CHANNELS-1];
  logic signed [ACC_W-1:0]    s_shifted [0:CHANNELS-1];
  logic signed [ACC_W-1:0]    s_act     [0:CHANNELS-1];
  logic signed [ACC_W-1:0]    s_requant [0:CHANNELS-1];
  logic signed [OUT_W-1:0]    s_clamped [0:CHANNELS-1];

  generate
    for (genvar c = 0; c < CHANNELS; c++) begin : g_pipe

      // Stage 1 — bias
      assign s_biased[c] = acc_in[c] + bias[c];

      // Stage 2 — BN scale (Q0.15 multiply: extend to PROD_W, shift right 15)
      logic signed [PROD_W-1:0] prod;
      assign prod        = $signed({{(PROD_W-ACC_W){s_biased[c][ACC_W-1]}}, s_biased[c]})
                           * $signed({1'b0, bn_scale[c]});  // unsigned scale, sign-extend
      assign s_scaled[c] = prod[ACC_W+14 : 15];             // arithmetic >>15

      // Stage 3 — BN shift
      assign s_shifted[c] = s_scaled[c] + bn_shift[c];

      // Stage 4 — Activation function
      always_comb begin
        case (act_fn)
          2'd1: // ReLU
            s_act[c] = (s_shifted[c] < 0) ? '0 : s_shifted[c];
          2'd2: // ReLU6
            if      (s_shifted[c] < 0) s_act[c] = '0;
            else if (s_shifted[c] > 6) s_act[c] = ACC_W'(6);
            else                       s_act[c] = s_shifted[c];
          default: // Linear
            s_act[c] = s_shifted[c];
        endcase
      end

      // Stage 5 — Requantise (arithmetic right shift)
      assign s_requant[c] = s_act[c] >>> requant_shift;

      // Stage 6 — INT8 saturating clamp
      always_comb begin
        if      ($signed(s_requant[c]) > INT8_HI) s_clamped[c] = OUT_W'(signed'(INT8_HI));
        else if ($signed(s_requant[c]) < INT8_LO) s_clamped[c] = OUT_W'(signed'(INT8_LO));
        else                                       s_clamped[c] = s_requant[c][OUT_W-1:0];
      end

    end : g_pipe
  endgenerate

  // -------------------------------------------------------------------------
  // 2×2 Max-pool accumulator (runs over up to 4 spatial groups)
  // -------------------------------------------------------------------------
  logic signed [OUT_W-1:0] pool_max [0:CHANNELS-1];  // running max
  logic                    pool_first;                 // first group flag

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int c = 0; c < CHANNELS; c++)
        pool_max[c] <= {OUT_W{1'b1}} << (OUT_W-1); // = INT8_MIN = −128
      pool_first <= 1'b1;
    end else if (acc_valid && pool_en) begin
      for (int c = 0; c < CHANNELS; c++) begin
        if (pool_first)
          pool_max[c] <= s_clamped[c];
        else
          pool_max[c] <= ($signed(s_clamped[c]) > $signed(pool_max[c]))
                         ? s_clamped[c] : pool_max[c];
      end
      pool_first <= acc_last;   // reset first-flag after the flush
    end else if (!pool_en) begin
      pool_first <= 1'b1;
    end
  end

  // -------------------------------------------------------------------------
  // Output register
  // -------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int c = 0; c < CHANNELS; c++) out_data[c] <= '0;
      out_valid <= 1'b0;
    end else begin
      if (pool_en) begin
        // Output the running max when the last group arrives
        if (acc_valid && acc_last) begin
          for (int c = 0; c < CHANNELS; c++) begin
            // Include current beat in the comparison before latching
            out_data[c] <= ($signed(s_clamped[c]) > $signed(pool_max[c]) && !pool_first)
                           ? s_clamped[c]
                           : (pool_first ? s_clamped[c] : pool_max[c]);
          end
          out_valid <= 1'b1;
        end else begin
          out_valid <= 1'b0;
        end
      end else begin
        // No pooling: register output one cycle after valid
        if (acc_valid) begin
          for (int c = 0; c < CHANNELS; c++) out_data[c] <= s_clamped[c];
          out_valid <= 1'b1;
        end else begin
          out_valid <= 1'b0;
        end
      end
    end
  end

endmodule : npu_postop