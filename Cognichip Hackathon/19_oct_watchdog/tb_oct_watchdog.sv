`timescale 1ns/1ps
// ============================================================
//  TB-16 — tb_oct_watchdog
//  OCT A-Scan Rate Watchdog Testbench
//  Clock  : 250 MHz (4 ns period)
//  DUT    : oct_watchdog
//  Config : expected_ascan_rate = 100 kHz → period = 2500 cycles
//           lock window ±10 % → [2250, 2750] cycles
// ============================================================

// ============================================================
//  Behavioral DUT : oct_watchdog
//  Replace with your synthesisable RTL when available —
//  the testbench binds only to the port list below.
// ============================================================
module oct_watchdog (
  input  logic        clk,
  input  logic        rst_n,
  input  logic [31:0] expected_ascan_rate,  // A-scans per second
  input  logic        ascan_tlast,          // one pulse per A-scan frame
  input  logic [1:0]  tuser_errors,         // tuser[0] = overflow
  // STATUS register write — pulse status_wr for 1 cycle;
  // status_wdata[0] = 1 clears overflow_flag
  input  logic        status_wr,
  input  logic [31:0] status_wdata,
  // Outputs
  output logic        locked,
  output logic        galvo_drift,
  output logic        overflow_flag,
  output logic        watchdog_irq         // 1-cycle active-high pulse
);

  // --------------------------------------------------------
  //  Compile-time constants
  // --------------------------------------------------------
  localparam int CLK_FREQ  = 250_000_000;
  localparam int LOCK_CNT  = 5;   // consecutive in-window periods → locked
  localparam int DRIFT_CNT = 3;   // consecutive out-of-window periods → galvo_drift

  // --------------------------------------------------------
  //  Expected period + ±10 % window  (behavioural division)
  // --------------------------------------------------------
  logic [31:0] exp_period;
  logic [31:0] win_lo, win_hi;

  always_comb begin
    exp_period = (expected_ascan_rate != 32'd0)
                 ? (32'(CLK_FREQ) / expected_ascan_rate)
                 : 32'd1;
    win_lo = (exp_period * 9)  / 10;
    win_hi = (exp_period * 11) / 10;
  end

  // --------------------------------------------------------
  //  Inter-tlast counter
  //  On every ascan_tlast pulse: latch period_ctr → meas_period,
  //  then reset period_ctr to 0.
  // --------------------------------------------------------
  logic [31:0] period_ctr;
  logic [31:0] meas_period;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      period_ctr  <= '0;
      meas_period <= '0;
    end else if (ascan_tlast) begin
      meas_period <= period_ctr;  // latch before clearing
      period_ctr  <= '0;
    end else begin
      period_ctr  <= period_ctr + 1;
    end
  end

  // --------------------------------------------------------
  //  1-cycle tlast pipeline
  //  Gives meas_period exactly one cycle to settle after tlast
  //  before the in_window comparison drives the counter update.
  // --------------------------------------------------------
  logic tlast_d1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) tlast_d1 <= 1'b0;
    else        tlast_d1 <= ascan_tlast;
  end

  // in_window: combinational — valid when tlast_d1 is high
  logic in_window;
  assign in_window = (meas_period >= win_lo) && (meas_period <= win_hi);

  // --------------------------------------------------------
  //  Consecutive good / bad counters + locked / galvo_drift
  //
  //  Both counters evaluate good_cnt / bad_cnt BEFORE the
  //  increment (NBA semantics) so the threshold check
  //  `>= LOCK_CNT - 1` means "this is the Nth good edge."
  // --------------------------------------------------------
  logic [3:0] good_cnt;
  logic [3:0] bad_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      good_cnt    <= '0;
      bad_cnt     <= '0;
      locked      <= 1'b0;
      galvo_drift <= 1'b0;
    end else if (tlast_d1) begin
      if (in_window) begin
        // ---- Good period ----
        good_cnt    <= (good_cnt < 4'(LOCK_CNT)) ? good_cnt + 1 : 4'(LOCK_CNT);
        bad_cnt     <= '0;
        galvo_drift <= 1'b0;
        // Lock once good_cnt reaches LOCK_CNT-1 (pre-increment check)
        if (good_cnt >= 4'(LOCK_CNT - 1))
          locked <= 1'b1;
      end else begin
        // ---- Bad period ----
        bad_cnt  <= (bad_cnt < 4'(DRIFT_CNT)) ? bad_cnt + 1 : 4'(DRIFT_CNT);
        good_cnt <= '0;
        locked   <= 1'b0;
        // Drift once bad_cnt reaches DRIFT_CNT-1 (pre-increment check)
        if (bad_cnt >= 4'(DRIFT_CNT - 1))
          galvo_drift <= 1'b1;
      end
    end
  end

  // --------------------------------------------------------
  //  watchdog_irq : single-cycle rising-edge pulse on galvo_drift
  // --------------------------------------------------------
  logic galvo_drift_d1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) galvo_drift_d1 <= 1'b0;
    else        galvo_drift_d1 <= galvo_drift;
  end

  assign watchdog_irq = galvo_drift & ~galvo_drift_d1;

  // --------------------------------------------------------
  //  overflow_flag : SR latch with synchronous clear
  //    Set  : tuser_errors[0] = 1
  //    Clear: status_wr pulse with status_wdata[0] = 1
  //    Clear wins over set when both arrive simultaneously.
  // --------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      overflow_flag <= 1'b0;
    else if (status_wr && status_wdata[0])
      overflow_flag <= 1'b0;        // clear wins
    else if (tuser_errors[0])
      overflow_flag <= 1'b1;
  end

endmodule : oct_watchdog


// ============================================================
//  Testbench
// ============================================================
module tb_oct_watchdog;

  // --------------------------------------------------------
  //  Parameters
  // --------------------------------------------------------
  localparam int    ASCAN_RATE_HZ  = 100_000;
  localparam int    CLK_FREQ_HZ    = 250_000_000;
  localparam real   CLK_PERIOD_NS  = 4.0;           // 250 MHz
  localparam int    ASCAN_PERIOD   = CLK_FREQ_HZ / ASCAN_RATE_HZ;  // 2500
  localparam int    BAD_PERIOD     = 3000;           // 20 % slower — outside ±10 % window
  localparam real   TIMEOUT_US     = 300.0;
  localparam int    TIMEOUT_CYCLES = int'(TIMEOUT_US * 1000.0 / CLK_PERIOD_NS);

  // --------------------------------------------------------
  //  DUT Port Connections
  // --------------------------------------------------------
  logic        clk;
  logic        rst_n;
  logic [31:0] expected_ascan_rate;
  logic        ascan_tlast;
  logic  [1:0] tuser_errors;
  logic        status_wr;
  logic [31:0] status_wdata;
  logic        locked;
  logic        galvo_drift;
  logic        overflow_flag;
  logic        watchdog_irq;

  // --------------------------------------------------------
  //  DUT Instantiation
  // --------------------------------------------------------
  oct_watchdog dut (
    .clk                (clk),
    .rst_n              (rst_n),
    .expected_ascan_rate(expected_ascan_rate),
    .ascan_tlast        (ascan_tlast),
    .tuser_errors       (tuser_errors),
    .status_wr          (status_wr),
    .status_wdata       (status_wdata),
    .locked             (locked),
    .galvo_drift        (galvo_drift),
    .overflow_flag      (overflow_flag),
    .watchdog_irq       (watchdog_irq)
  );

  // --------------------------------------------------------
  //  Clock Generation  (250 MHz)
  // --------------------------------------------------------
  initial clk = 1'b0;
  always  #(CLK_PERIOD_NS / 2.0) clk = ~clk;

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
        $display("TB_WATCHDOG: FAIL — TIMEOUT after %.0f µs", TIMEOUT_US);
        $finish;
      end
    end
  end

  // --------------------------------------------------------
  //  Clock helpers
  // --------------------------------------------------------
  task automatic tick(input int n = 1);
    repeat (n) @(posedge clk);
  endtask

  // Sample registered DUT outputs 1 ns after the rising edge so all
  // NBA assignments have resolved before we read any signal value.
  task automatic stable_tick();
    @(posedge clk); #1;
  endtask

  // --------------------------------------------------------
  //  Drive one A-scan tlast pulse.
  //  The pulse-to-pulse interval (including the 1-cycle tlast
  //  assertion) equals period_cyc clock cycles.
  //  Sequence: wait (period_cyc-1) edges → assert tlast → 1 edge → deassert.
  // --------------------------------------------------------
  task automatic drive_one_frame(input int period_cyc);
    repeat (period_cyc - 1) @(posedge clk);
    ascan_tlast <= 1'b1;
    @(posedge clk);
    ascan_tlast <= 1'b0;
  endtask

  // Drive n consecutive frames at a fixed period
  task automatic drive_frames(input int n, input int period_cyc);
    for (int i = 0; i < n; i++)
      drive_one_frame(period_cyc);
  endtask

  // --------------------------------------------------------
  //  Module-scope scratch variables
  //  (declared here to avoid static-initialiser errors in
  //   named begin blocks)
  // --------------------------------------------------------
  logic       drift_seen;
  logic       irq_seen;
  int         irq_cycle_offset;   // cycle offset from galvo_drift assertion

  // --------------------------------------------------------
  //  Main Test Sequence
  // --------------------------------------------------------
  initial begin : tb_main

    // ---- Safe defaults ----
    rst_n                <= 1'b0;
    expected_ascan_rate  <= 32'd100_000;
    ascan_tlast          <= 1'b0;
    tuser_errors         <= 2'b00;
    status_wr            <= 1'b0;
    status_wdata         <= 32'h0;

    tick(8);
    rst_n <= 1'b1;
    tick(4);   // allow reset to propagate; next tlast ≈ 2504 cycles after rst_n (in-window)

    // ============================================================
    //  TEST 1 — locked asserts at correct rate (5 good frames)
    // ============================================================
    $display("TB_WATCHDOG: Starting TEST 1 — locked at 2500-cycle rate");

    // Drive 5 consecutive in-window frames (2500 cycles each).
    // The very first frame starts ≈4 cycles into the first period (post-reset
    // settle), giving meas_period ≈ 2503 — still well inside [2250, 2750].
    drive_frames(5, ASCAN_PERIOD);

    // Wait 3 cycles for tlast_d1 pipeline + locked register to settle.
    tick(3); #1;

    if (!locked) begin
      $display("TB_WATCHDOG: FAIL — TEST1 locked did not assert after 5 good frames");
      $finish;
    end
    if (galvo_drift) begin
      $display("TB_WATCHDOG: FAIL — TEST1 galvo_drift spuriously high");
      $finish;
    end

    $display("TB_WATCHDOG: TEST 1 PASS — locked=1, galvo_drift=0 after 5 good frames");

    // ============================================================
    //  TEST 2 — locked deasserts on drift (rate changes to 3000)
    // ============================================================
    $display("TB_WATCHDOG: Starting TEST 2 — locked deasserts at 3000-cycle rate");

    // First bad frame — meas_period ≈ 2999 > win_hi (2750) → out-of-window.
    drive_one_frame(BAD_PERIOD);
    tick(3); #1;

    if (locked) begin
      $display("TB_WATCHDOG: FAIL — TEST2 locked still high after 1 out-of-window frame");
      $finish;
    end

    $display("TB_WATCHDOG: TEST 2 PASS — locked=0 after first out-of-window frame");

    // ============================================================
    //  TEST 3 — galvo_drift after exactly 3 bad periods
    //  TEST 5 — watchdog_irq pulses within 5 cycles of galvo_drift
    //  (Both tests share the same driving sequence.)
    // ============================================================
    $display("TB_WATCHDOG: Starting TEST 3 — galvo_drift after 3 bad periods");
    $display("TB_WATCHDOG: Starting TEST 5 — watchdog_irq within 5 cycles of galvo_drift");

    // Verify galvo_drift is still clear after only 1 bad frame.
    tick(1); #1;
    if (galvo_drift) begin
      $display("TB_WATCHDOG: FAIL — TEST3 galvo_drift premature after 1 bad frame");
      $finish;
    end

    // Second bad frame  (bad_cnt → 2, drift still clear)
    drive_one_frame(BAD_PERIOD);
    tick(3); #1;

    if (galvo_drift) begin
      $display("TB_WATCHDOG: FAIL — TEST3 galvo_drift premature after 2 bad frames");
      $finish;
    end

    // Third bad frame — galvo_drift must assert immediately after
    drive_one_frame(BAD_PERIOD);

    // Scan the next 6 cycles for galvo_drift and the 1-cycle watchdog_irq pulse.
    // galvo_drift  asserts at T+1 (tlast_d1 pipeline)
    // watchdog_irq fires  at T+1 (combinatorial from galvo_drift rising edge on same cycle)
    drift_seen       = 1'b0;
    irq_seen         = 1'b0;
    irq_cycle_offset = -1;

    for (int i = 0; i < 6; i++) begin
      @(posedge clk); #1;
      if (galvo_drift  && !drift_seen) drift_seen = 1'b1;
      if (watchdog_irq && !irq_seen) begin
        irq_seen         = 1'b1;
        irq_cycle_offset = i + 1;
      end
    end

    if (!drift_seen) begin
      $display("TB_WATCHDOG: FAIL — TEST3 galvo_drift never asserted after 3 bad frames");
      $finish;
    end
    $display("TB_WATCHDOG: TEST 3 PASS — galvo_drift asserted after exactly 3 consecutive bad periods");

    if (!irq_seen) begin
      $display("TB_WATCHDOG: FAIL — TEST5 watchdog_irq never pulsed within 5 cycles of galvo_drift");
      $finish;
    end
    if (irq_cycle_offset > 5) begin
      $display("TB_WATCHDOG: FAIL — TEST5 watchdog_irq too late: %0d cycles after galvo_drift",
               irq_cycle_offset);
      $finish;
    end
    $display("TB_WATCHDOG: TEST 5 PASS — watchdog_irq pulsed %0d cycle(s) after galvo_drift",
             irq_cycle_offset);

    // ============================================================
    //  TEST 4 — overflow_flag latching and register-clear
    // ============================================================
    $display("TB_WATCHDOG: Starting TEST 4 — overflow_flag latch and clear");

    // Drive tuser_errors[0] for exactly 1 cycle.
    @(posedge clk);
    tuser_errors <= 2'b01;
    @(posedge clk);
    tuser_errors <= 2'b00;

    // overflow_flag must now be latched high.
    tick(1); #1;
    if (!overflow_flag) begin
      $display("TB_WATCHDOG: FAIL — TEST4 overflow_flag did not latch on tuser_errors[0]");
      $finish;
    end

    // Flag must stay high even with tuser_errors back to zero.
    tick(4); #1;
    if (!overflow_flag) begin
      $display("TB_WATCHDOG: FAIL — TEST4 overflow_flag cleared without STATUS write");
      $finish;
    end

    // Simulate AXI STATUS register write: set bit 0 to clear overflow_flag.
    @(posedge clk);
    status_wr    <= 1'b1;
    status_wdata <= 32'h0000_0001;
    @(posedge clk);
    status_wr    <= 1'b0;
    status_wdata <= 32'h0;

    // overflow_flag must now be clear.
    tick(1); #1;
    if (overflow_flag) begin
      $display("TB_WATCHDOG: FAIL — TEST4 overflow_flag did not clear after STATUS write");
      $finish;
    end

    $display("TB_WATCHDOG: TEST 4 PASS — overflow_flag latched and cleared correctly");

    // ============================================================
    //  TEST 6 — Recovery: restore 2500-cycle rate → locked again
    // ============================================================
    $display("TB_WATCHDOG: Starting TEST 6 — Recovery to 2500-cycle rate");

    // System is currently in galvo_drift state with bad_cnt=3.
    // Drive 5 consecutive in-window frames to re-establish lock.
    drive_frames(5, ASCAN_PERIOD);
    tick(3); #1;

    if (!locked) begin
      $display("TB_WATCHDOG: FAIL — TEST6 locked did not reassert after 5 good frames");
      $finish;
    end
    if (galvo_drift) begin
      $display("TB_WATCHDOG: FAIL — TEST6 galvo_drift still high after recovery");
      $finish;
    end

    $display("TB_WATCHDOG: TEST 6 PASS — locked=1, galvo_drift=0 after recovery");

    // ============================================================
    //  All Tests Passed
    // ============================================================
    $display("TB_WATCHDOG: PASS");
    $finish;

  end : tb_main

endmodule : tb_oct_watchdog