`timescale 1ns/1ps
// ============================================================================
//  TB-15 — tb_npu_pe_array
//  INT8 Systolic MAC Array Testbench
//  DUT  : npu_pe_array  (parameterized ROWS=4, COLS=4 for simulation speed)
//  Clock: 300 MHz  (period = 3.333 ns → use 10/3 → model as 3.333 ns)
//
//  Tests
//  -----
//  TEST 1 – Single activation vector, known weights
//  TEST 2 – Accumulation across multiple activation vectors
//  TEST 3 – acc_clear between layers
//  TEST 4 – Signed arithmetic (negative weights)
//  TEST 5 – Systolic skew verification
// ============================================================================

module tb_npu_pe_array;

  // -------------------------------------------------------------------------
  // Parameters (scaled-down from 16×16 for sim speed)
  // -------------------------------------------------------------------------
  localparam int ROWS        = 4;
  localparam int COLS        = 4;
  localparam int DATA_W      = 8;   // INT8 activations & weights
  localparam int ACC_W       = 32;  // accumulator width
  localparam real CLK_PERIOD = 3.333; // 300 MHz → ~3.333 ns

  // -------------------------------------------------------------------------
  // DUT port declarations
  // -------------------------------------------------------------------------
  logic                          clk;
  logic                          rst_n;

  // Weight load interface
  logic                          weight_load_en;
  logic [$clog2(ROWS)-1:0]       weight_row_sel;
  logic signed [DATA_W-1:0]      weight_data [0:COLS-1];

  // Activation input interface
  logic signed [DATA_W-1:0]      act_in      [0:COLS-1];
  logic                          act_valid;
  logic                          act_last;

  // Accumulator control
  logic                          acc_clear;

  // Output interface
  logic signed [ACC_W-1:0]       acc_out     [0:ROWS-1];
  logic                          out_valid;

  // -------------------------------------------------------------------------
  // DUT instantiation
  // -------------------------------------------------------------------------
  npu_pe_array #(
    .ROWS    (ROWS),
    .COLS    (COLS),
    .DATA_W  (DATA_W),
    .ACC_W   (ACC_W)
  ) dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .weight_load_en (weight_load_en),
    .weight_row_sel (weight_row_sel),
    .weight_data    (weight_data),
    .act_in         (act_in),
    .act_valid      (act_valid),
    .act_last       (act_last),
    .acc_clear      (acc_clear),
    .acc_out        (acc_out),
    .out_valid      (out_valid)
  );

  // -------------------------------------------------------------------------
  // Clock generation  – 300 MHz
  // -------------------------------------------------------------------------
  initial clk = 1'b0;
  always #(CLK_PERIOD / 2.0) clk = ~clk;

  // -------------------------------------------------------------------------
  // Scoreboard / pass-fail tracking
  // -------------------------------------------------------------------------
  int  fail_count = 0;
  string fail_log  = "";

  // Helper: check a single accumulator output
  task automatic check_acc (
    input string         test_name,
    input int            row,
    input logic signed [ACC_W-1:0] got,
    input int            expected
  );
    if (signed'(got) !== expected) begin
      $display("  [FAIL] %s row %0d  got %0d  expected %0d",
               test_name, row, $signed(got), expected);
      fail_count++;
    end else begin
      $display("  [PASS] %s row %0d  = %0d", test_name, row, $signed(got));
    end
  endtask

  // -------------------------------------------------------------------------
  // Helper tasks
  // -------------------------------------------------------------------------

  // Drive all idle values
  task automatic idle_inputs();
    weight_load_en = 1'b0;
    weight_row_sel = '0;
    for (int c = 0; c < COLS; c++) weight_data[c] = '0;
    act_valid      = 1'b0;
    act_last       = 1'b0;
    acc_clear      = 1'b0;
    for (int c = 0; c < COLS; c++) act_in[c] = '0;
  endtask

  // Load entire weight matrix in ROWS consecutive cycles
  // weights_2d[r][c] is the weight for row r, column c
  task automatic load_weights (
    input logic signed [DATA_W-1:0] weights_2d [0:ROWS-1][0:COLS-1]
  );
    @(posedge clk); #1;
    for (int r = 0; r < ROWS; r++) begin
      weight_load_en = 1'b1;
      weight_row_sel = r[$clog2(ROWS)-1:0];
      for (int c = 0; c < COLS; c++)
        weight_data[c] = weights_2d[r][c];
      @(posedge clk); #1;
    end
    weight_load_en = 1'b0;
    weight_row_sel = '0;
    for (int c = 0; c < COLS; c++) weight_data[c] = '0;
  endtask

  // Send a single activation vector (act_last = 1 on last beat)
  // For multi-vector bursts call this with is_last=0 for intermediate beats
  task automatic send_act_vec (
    input logic signed [DATA_W-1:0] vec [0:COLS-1],
    input logic                     is_last
  );
    @(posedge clk); #1;
    act_valid = 1'b1;
    act_last  = is_last;
    for (int c = 0; c < COLS; c++) act_in[c] = vec[c];
    @(posedge clk); #1;
    act_valid = 1'b0;
    act_last  = 1'b0;
    for (int c = 0; c < COLS; c++) act_in[c] = '0;
  endtask

  // Wait for out_valid with a cycle-count safety timeout
  task automatic wait_out_valid (input int timeout_cycles = 200);
    int cnt = 0;
    while (!out_valid && cnt < timeout_cycles) begin
      @(posedge clk);
      cnt++;
    end
    if (cnt >= timeout_cycles) begin
      $display("  [FAIL] Timeout waiting for out_valid after %0d cycles", timeout_cycles);
      fail_count++;
    end
  endtask

  // Assert acc_clear for exactly one clock cycle
  task automatic do_acc_clear();
    @(posedge clk); #1;
    acc_clear = 1'b1;
    @(posedge clk); #1;
    acc_clear = 1'b0;
  endtask

  // -------------------------------------------------------------------------
  // Main test sequence
  // -------------------------------------------------------------------------
  initial begin : tb_main

    // ------------------------------------------------------------------
    // Initialise
    // ------------------------------------------------------------------
    idle_inputs();
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    $display("");
    $display("========================================================");
    $display("  TB-15  npu_pe_array  %0d×%0d  INT8 Systolic MAC Array", ROWS, COLS);
    $display("========================================================");

    // ==================================================================
    // TEST 1 — Single activation vector, known weights
    //   weight[r][c] = r+1   →  row0=1, row1=2, row2=3, row3=4
    //   act_in = {1,1,1,1}
    //   Expected: acc_out[r] = (r+1) × COLS
    //             acc_out[0]=4, [1]=8, [2]=12, [3]=16
    // ==================================================================
    $display("\n-- TEST 1 : Single activation vector, known weights --");
    begin
      logic signed [DATA_W-1:0] W1 [0:ROWS-1][0:COLS-1];
      logic signed [DATA_W-1:0] A1 [0:COLS-1];

      for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
          W1[r][c] = DATA_W'(signed'(r + 1));

      for (int c = 0; c < COLS; c++) A1[c] = 8'sd1;

      load_weights(W1);
      send_act_vec(A1, 1'b1);   // act_last = 1
      wait_out_valid();

      for (int r = 0; r < ROWS; r++)
        check_acc("TEST1", r, acc_out[r], (r + 1) * COLS);
    end

    // ==================================================================
    // TEST 2 — Accumulation across multiple activation vectors
    //   weight[r][c] = 1  (all ones)
    //   Vectors: identity columns of 4×4  →  each row accumulates 4 ones
    //   Expected: acc_out[r] == 4 for all r
    // ==================================================================
    $display("\n-- TEST 2 : Accumulation across multiple activation vectors --");
    begin
      logic signed [DATA_W-1:0] W2 [0:ROWS-1][0:COLS-1];
      logic signed [DATA_W-1:0] A2 [4][0:COLS-1];

      for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
          W2[r][c] = 8'sd1;

      // Identity columns: {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}
      for (int v = 0; v < 4; v++)
        for (int c = 0; c < COLS; c++)
          A2[v][c] = (c == v) ? 8'sd1 : 8'sd0;

      do_acc_clear();            // start clean
      load_weights(W2);

      for (int v = 0; v < 4; v++)
        send_act_vec(A2[v], (v == 3) ? 1'b1 : 1'b0);

      wait_out_valid();

      for (int r = 0; r < ROWS; r++)
        check_acc("TEST2", r, acc_out[r], 4);
    end

    // ==================================================================
    // TEST 3 — acc_clear between layers
    //   Run TEST 2 accumulation (leaves acc == 4), then clear.
    //   Send one vector {2,2,2,2}.
    //   Expected: acc_out[r] == 8  (clear removed old 4; 1×2×4 = 8)
    // ==================================================================
    $display("\n-- TEST 3 : acc_clear between layers --");
    begin
      logic signed [DATA_W-1:0] W3 [0:ROWS-1][0:COLS-1];
      logic signed [DATA_W-1:0] A3 [0:COLS-1];

      // Reuse all-ones weight matrix from TEST 2
      for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
          W3[r][c] = 8'sd1;

      // TEST 2 state is still live — acc_out[r] should be 4 here.
      // Issue acc_clear
      do_acc_clear();
      repeat (4) @(posedge clk);  // let pipeline drain

      // New activation after clear
      for (int c = 0; c < COLS; c++) A3[c] = 8'sd2;

      load_weights(W3);
      send_act_vec(A3, 1'b1);
      wait_out_valid();

      // 1×2×COLS = 8 per row
      for (int r = 0; r < ROWS; r++)
        check_acc("TEST3", r, acc_out[r], 2 * COLS);
    end

    // ==================================================================
    // TEST 4 — Signed arithmetic (negative weights)
    //   weight[0][c] = -1 (8'hFF in INT8 two's complement)
    //   act_in = {10,10,10,10}
    //   Expected: acc_out[0] == -40  (signed check)
    //             acc_out[1..3] unchanged / don't-care, but check row 0
    // ==================================================================
    $display("\n-- TEST 4 : Signed arithmetic (negative weights) --");
    begin
      logic signed [DATA_W-1:0] W4 [0:ROWS-1][0:COLS-1];
      logic signed [DATA_W-1:0] A4 [0:COLS-1];

      for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
          W4[r][c] = (r == 0) ? -8'sd1 : 8'sd0;  // row 0 = -1, others = 0

      for (int c = 0; c < COLS; c++) A4[c] = 8'sd10;

      do_acc_clear();
      load_weights(W4);
      send_act_vec(A4, 1'b1);
      wait_out_valid();

      // row 0: (-1)*10 * COLS = -40
      check_acc("TEST4", 0, acc_out[0], -10 * COLS);
    end

    // ==================================================================
    // TEST 5 — Systolic skew verification
    //   weight[r][c] = c+1  (column-dependent, same for every row)
    //   act_in = {4,3,2,1}  (decreasing)
    //   Expected: acc_out[r] = Σ(c+1)*(4-c) for c=0..3
    //                        = 1×4 + 2×3 + 3×2 + 4×1 = 4+6+6+4 = 20
    //   Validates that the pipeline skew routes each activation value
    //   to the correct column PE.
    // ==================================================================
    $display("\n-- TEST 5 : Systolic skew verification --");
    begin
      logic signed [DATA_W-1:0] W5 [0:ROWS-1][0:COLS-1];
      logic signed [DATA_W-1:0] A5 [0:COLS-1];
      int expected5;

      for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
          W5[r][c] = DATA_W'(signed'(c + 1));

      // act = {4,3,2,1}
      for (int c = 0; c < COLS; c++)
        A5[c] = DATA_W'(signed'(COLS - c));

      // Compute expected sum symbolically
      expected5 = 0;
      for (int c = 0; c < COLS; c++)
        expected5 += (c + 1) * (COLS - c);

      do_acc_clear();
      load_weights(W5);
      send_act_vec(A5, 1'b1);
      wait_out_valid();

      for (int r = 0; r < ROWS; r++)
        check_acc("TEST5", r, acc_out[r], expected5);
    end

    // ==================================================================
    // Result summary
    // ==================================================================
    $display("");
    $display("========================================================");
    if (fail_count == 0)
      $display("  TB_PE_ARRAY: PASS");
    else
      $display("  TB_PE_ARRAY: FAIL  (%0d assertion(s) failed — see above)", fail_count);
    $display("========================================================");
    $display("");

    $finish;
  end : tb_main

  // -------------------------------------------------------------------------
  // Simulation watchdog — abort after 10 000 cycles
  // -------------------------------------------------------------------------
  initial begin
    repeat (10_000) @(posedge clk);
    $display("[WATCHDOG] Simulation exceeded 10 000 cycles — aborting.");
    $finish;
  end

endmodule : tb_npu_pe_array


// ============================================================================
//  REFERENCE DUT — npu_pe_array
//  Parameterised INT8 systolic-array MAC with row-wise weight registers,
//  column-skewed activation pipeline, and per-row accumulators.
//
//  Interface summary
//  -----------------
//   weight_load_en  : pulse high to write one row of weights
//   weight_row_sel  : selects which row's weight register to write
//   weight_data[]   : COLS-wide INT8 weight values
//   act_in[]        : COLS-wide INT8 activation inputs (column 0 first)
//   act_valid       : activation data is valid this cycle
//   act_last        : last activation beat of this tile; triggers out_valid
//   acc_clear       : synchronous clear of all ROWS accumulators
//   acc_out[]       : ROWS-wide 32-bit signed accumulator outputs
//   out_valid       : asserted one cycle after act_last is processed
//
//  Systolic skew
//  -------------
//  Column c receives its activation one cycle later than column c-1,
//  modelling the horizontal wavefront propagation of a systolic array.
//  A COLS-deep shift register bank on the activation bus implements this.
// ============================================================================

module npu_pe_array #(
  parameter int ROWS   = 16,
  parameter int COLS   = 16,
  parameter int DATA_W = 8,
  parameter int ACC_W  = 32
)(
  input  logic                         clk,
  input  logic                         rst_n,

  // Weight load
  input  logic                         weight_load_en,
  input  logic [$clog2(ROWS)-1:0]      weight_row_sel,
  input  logic signed [DATA_W-1:0]     weight_data [0:COLS-1],

  // Activation input
  input  logic signed [DATA_W-1:0]     act_in    [0:COLS-1],
  input  logic                         act_valid,
  input  logic                         act_last,

  // Control
  input  logic                         acc_clear,

  // Outputs
  output logic signed [ACC_W-1:0]      acc_out   [0:ROWS-1],
  output logic                         out_valid
);

  // -------------------------------------------------------------------------
  // Weight register file  [ROWS][COLS]
  // -------------------------------------------------------------------------
  logic signed [DATA_W-1:0] w_reg [0:ROWS-1][0:COLS-1];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
          w_reg[r][c] <= '0;
    end else if (weight_load_en) begin
      for (int c = 0; c < COLS; c++)
        w_reg[weight_row_sel][c] <= weight_data[c];
    end
  end

  // -------------------------------------------------------------------------
  // Systolic skew — shift register bank
  //   skew_pipe[c][d] : column c, delay-stage d
  //   column c sees the activation delayed by c cycles.
  // -------------------------------------------------------------------------
  // Maximum skew depth = COLS-1 stages for the last column.
  logic signed [DATA_W-1:0] skew_pipe [0:COLS-1][0:COLS-1]; // [col][delay]
  logic                     skew_valid [0:COLS-1][0:COLS-1];
  logic                     skew_last  [0:COLS-1][0:COLS-1];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int c = 0; c < COLS; c++)
        for (int d = 0; d < COLS; d++) begin
          skew_pipe [c][d] <= '0;
          skew_valid[c][d] <= 1'b0;
          skew_last [c][d] <= 1'b0;
        end
    end else begin
      for (int c = 0; c < COLS; c++) begin
        if (c == 0) begin
          // Column 0: zero delay — take directly from act_in
          skew_pipe [0][0] <= act_in[0];
          skew_valid[0][0] <= act_valid;
          skew_last [0][0] <= act_last;
        end else begin
          // Column c: shift through c pipeline stages
          skew_pipe [c][0] <= act_in[c];
          skew_valid[c][0] <= act_valid;
          skew_last [c][0] <= act_last;
          for (int d = 1; d <= c; d++) begin
            skew_pipe [c][d] <= skew_pipe [c][d-1];
            skew_valid[c][d] <= skew_valid[c][d-1];
            skew_last [c][d] <= skew_last [c][d-1];
          end
        end
      end
    end
  end

  // Convenience: skewed value at the PE input for column c
  // Column c reads from skew_pipe[c][c].
  // Column 0 is undelayed, so we read skew_pipe[0][0].
  function automatic logic signed [DATA_W-1:0] skewed_act (input int c);
    return skew_pipe[c][c];
  endfunction
  function automatic logic skewed_valid (input int c);
    return skew_valid[c][c];
  endfunction
  function automatic logic skewed_last (input int c);
    return skew_last[c][c];
  endfunction

  // -------------------------------------------------------------------------
  // Accumulators  [ROWS]
  //   Each accumulator sums across all COLS PEs in its row.
  //   out_valid is registered one cycle after the last-column last-beat lands.
  // -------------------------------------------------------------------------
  // The last-valid indicator for a row is driven by the last column's skewed
  // last/valid signal (it arrives COLS-1 cycles after column 0).
  logic signed [ACC_W-1:0] acc_reg [0:ROWS-1];
  logic last_col_last_valid;

  assign last_col_last_valid = skewed_valid(COLS-1) & skewed_last(COLS-1);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int r = 0; r < ROWS; r++)
        acc_reg[r] <= '0;
      out_valid <= 1'b0;
    end else begin

      // Clear takes priority over accumulation
      if (acc_clear) begin
        for (int r = 0; r < ROWS; r++)
          acc_reg[r] <= '0;
        out_valid <= 1'b0;

      end else begin
        // MAC across all columns for each row
        for (int r = 0; r < ROWS; r++) begin
          logic signed [ACC_W-1:0] mac_sum;
          mac_sum = acc_reg[r];
          for (int c = 0; c < COLS; c++) begin
            if (skewed_valid(c))
              mac_sum = mac_sum +
                        (ACC_W)'(w_reg[r][c]) * (ACC_W)'(skewed_act(c));
          end
          acc_reg[r] <= mac_sum;
        end

        // out_valid: strobe one cycle after last-column last beat
        out_valid <= last_col_last_valid;
      end
    end
  end

  // Drive outputs
  always_comb begin
    for (int r = 0; r < ROWS; r++)
      acc_out[r] = acc_reg[r];
  end

endmodule : npu_pe_array