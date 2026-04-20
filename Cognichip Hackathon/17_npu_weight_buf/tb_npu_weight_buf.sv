`timescale 1ns/1ps
// ============================================================
//  TB-15 — tb_npu_weight_buf
//  NPU Weight Buffer (N_URAM=1, 256K × 64-bit)
//  Clock  : 300 MHz (≈3.333 ns period)
//  Note   : Behavioral URAM288 model replaces the vendor
//           primitive so the testbench runs on any simulator.
// ============================================================

// ============================================================
//  Behavioral URAM288 model
//  — Synchronous write  (1 cycle)
//  — Synchronous read   (2-cycle latency: addr→pipeline→data)
//  — (* ram_style = "ultra" *) attribute kept for synthesis
// ============================================================
module uram288_behavioral #(
  parameter int DEPTH = 256 * 1024,   // 256K rows
  parameter int WIDTH = 64
)(
  input  logic                      clk,
  // Write port
  input  logic                      we,
  input  logic [$clog2(DEPTH)-1:0]  waddr,
  input  logic [WIDTH-1:0]          wdata,
  // Read port
  input  logic                      re,
  input  logic [$clog2(DEPTH)-1:0]  raddr,
  output logic [WIDTH-1:0]          rdata
);
  (* ram_style = "ultra" *)
  logic [WIDTH-1:0] mem [0:DEPTH-1];

  // 2-stage output pipeline matches URAM288 read latency
  logic [WIDTH-1:0] pipe0, pipe1;

  always_ff @(posedge clk) begin
    if (we)
      mem[waddr] <= wdata;

    // Stage 1 — array access
    if (re)
      pipe0 <= mem[raddr];

    // Stage 2 — output register
    pipe1 <= pipe0;
  end

  assign rdata = pipe1;

endmodule : uram288_behavioral


// ============================================================
//  npu_weight_buf  — behavioral DUT wrapper
//
//  If you already have a synthesisable npu_weight_buf that
//  instantiates the vendor URAM288 by name, replace this
//  module with your real one.  The testbench below only
//  depends on the port list shown here.
// ============================================================
module npu_weight_buf #(
  parameter int N_URAM   = 1,
  parameter int DEPTH    = 256 * 1024,
  parameter int AW       = $clog2(DEPTH),   // 18
  parameter int DW       = 64
)(
  input  logic          clk,
  input  logic          rst_n,

  // DMA write port
  input  logic          dma_we,
  input  logic [AW-1:0] dma_waddr,
  input  logic [DW-1:0] dma_wdata,

  // Read port
  input  logic          rd_en,
  input  logic [AW-1:0] rd_addr,
  output logic [DW-1:0] rd_data,
  output logic          rd_valid,

  // Status
  output logic          load_done
);
  // ----------------------------------------------------------
  //  Behavioral URAM bank (one instance for N_URAM=1)
  // ----------------------------------------------------------
  logic [DW-1:0] uram_rdata;

  uram288_behavioral #(
    .DEPTH (DEPTH),
    .WIDTH (DW)
  ) u_uram (
    .clk   (clk),
    .we    (dma_we),
    .waddr (dma_waddr),
    .wdata (dma_wdata),
    .re    (rd_en),
    .raddr (rd_addr),
    .rdata (uram_rdata)
  );

  // ----------------------------------------------------------
  //  rd_valid pipeline: track rd_en through the 2-cycle
  //  read latency so rd_valid aligns with rd_data.
  // ----------------------------------------------------------
  logic rd_en_d1, rd_en_d2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_en_d1 <= 1'b0;
      rd_en_d2 <= 1'b0;
    end else begin
      rd_en_d1 <= rd_en;
      rd_en_d2 <= rd_en_d1;
    end
  end

  assign rd_data  = uram_rdata;
  assign rd_valid = rd_en_d2;

  // ----------------------------------------------------------
  //  load_done: pulses for one cycle when dma_we falls
  //  (trailing edge of the DMA burst).
  // ----------------------------------------------------------
  logic dma_we_d1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      dma_we_d1 <= 1'b0;
    else
      dma_we_d1 <= dma_we;
  end

  assign load_done = dma_we_d1 & ~dma_we;

endmodule : npu_weight_buf


// ============================================================
//  Testbench
// ============================================================
module tb_npu_weight_buf;

  // --------------------------------------------------------
  //  Parameters
  // --------------------------------------------------------
  localparam int    N_URAM       = 1;
  localparam int    DEPTH        = 256 * 1024;
  localparam int    AW           = $clog2(DEPTH);   // 18
  localparam int    DW           = 64;
  localparam int    MAX_ADDR     = DEPTH - 1;       // 18'h3FFFF
  localparam real   CLK_PERIOD   = 10.0 / 3.0;      // 300 MHz ≈ 3.333 ns
  localparam real   TIMEOUT_US   = 50.0;
  localparam int    TIMEOUT_CYC  = int'(TIMEOUT_US * 1000.0 / CLK_PERIOD);

  // Read latency of the URAM pipeline (cycles)
  localparam int    RD_LAT       = 2;

  // --------------------------------------------------------
  //  DUT Ports
  // --------------------------------------------------------
  logic          clk;
  logic          rst_n;

  logic          dma_we;
  logic [AW-1:0] dma_waddr;
  logic [DW-1:0] dma_wdata;

  logic          rd_en;
  logic [AW-1:0] rd_addr;
  logic [DW-1:0] rd_data;
  logic          rd_valid;

  logic          load_done;

  // --------------------------------------------------------
  //  DUT Instantiation
  // --------------------------------------------------------
  npu_weight_buf #(
    .N_URAM (N_URAM),
    .DEPTH  (DEPTH),
    .AW     (AW),
    .DW     (DW)
  ) dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .dma_we    (dma_we),
    .dma_waddr (dma_waddr),
    .dma_wdata (dma_wdata),
    .rd_en     (rd_en),
    .rd_addr   (rd_addr),
    .rd_data   (rd_data),
    .rd_valid  (rd_valid),
    .load_done (load_done)
  );

  // --------------------------------------------------------
  //  Clock Generation  (300 MHz)
  // --------------------------------------------------------
  initial clk = 1'b0;
  always  #(CLK_PERIOD / 2.0) clk = ~clk;

  // --------------------------------------------------------
  //  Timeout Watchdog
  // --------------------------------------------------------
  int timeout_ctr;
  always @(posedge clk) begin
    if (!rst_n)
      timeout_ctr <= 0;
    else begin
      timeout_ctr <= timeout_ctr + 1;
      if (timeout_ctr >= TIMEOUT_CYC) begin
        $display("TB_WEIGHT_BUF: FAIL — TIMEOUT after %.0f µs", TIMEOUT_US);
        $finish;
      end
    end
  end

  // --------------------------------------------------------
  //  Helpers
  // --------------------------------------------------------
  task automatic tick(input int n = 1);
    repeat (n) @(posedge clk);
  endtask

  // Single DMA write (combinational for one cycle)
  task automatic dma_write(
    input logic [AW-1:0] addr,
    input logic [DW-1:0] data
  );
    @(posedge clk);
    dma_waddr <= addr;
    dma_wdata <= data;
    dma_we    <= 1'b1;
    @(posedge clk);
    dma_we    <= 1'b0;
  endtask

  // Issue a read and return the data after RD_LAT cycles.
  // Captures the value of rd_data when rd_valid asserts.
  task automatic mem_read(
    input  logic [AW-1:0] addr,
    output logic [DW-1:0] data
  );
    @(posedge clk);
    rd_addr <= addr;
    rd_en   <= 1'b1;
    @(posedge clk);
    rd_en   <= 1'b0;
    // Wait for rd_valid (max RD_LAT + 1 extra guard cycles)
    repeat (RD_LAT + 1) begin
      @(posedge clk);
      if (rd_valid) begin
        data = rd_data;
        return;
      end
    end
    $display("TB_WEIGHT_BUF: FAIL — rd_valid never asserted for addr 18'h%05h", addr);
    $finish;
  endtask

  // --------------------------------------------------------
  //  Scratch variables (module scope — avoids static-init errors)
  // --------------------------------------------------------
  logic [DW-1:0] rd_word;
  logic [DW-1:0] burst_data [0:7];
  logic [DW-1:0] rd_burst   [0:7];
  logic          done_during;
  logic          done_after;
  int            valid_cycle;
  int            en_cycle;

  // --------------------------------------------------------
  //  Main Test Sequence
  // --------------------------------------------------------
  initial begin : tb_main

    // --- Safe defaults ---
    rst_n     <= 1'b0;
    dma_we    <= 1'b0;
    dma_waddr <= '0;
    dma_wdata <= '0;
    rd_en     <= 1'b0;
    rd_addr   <= '0;

    tick(8);
    rst_n <= 1'b1;
    tick(4);

    // ========================================================
    //  TEST 1 — Write then read single word
    // ========================================================
    $display("TB_WEIGHT_BUF: Starting TEST 1 — Single word write/read");

    dma_write(18'h00000, 64'hDEAD_BEEF_1234_5678);
    tick(2);

    mem_read(18'h00000, rd_word);

    if (rd_word !== 64'hDEAD_BEEF_1234_5678) begin
      $display("TB_WEIGHT_BUF: FAIL — TEST1 addr 0 got 64'h%016h expected 64'hDEAD_BEEF_1234_5678",
               rd_word);
      $finish;
    end

    $display("TB_WEIGHT_BUF: TEST 1 PASS — read 64'h%016h", rd_word);
    tick(2);

    // ========================================================
    //  TEST 2 — Burst write 8 words, burst read back
    //           data pattern: {addr[31:0], addr[31:0]}
    // ========================================================
    $display("TB_WEIGHT_BUF: Starting TEST 2 — Burst write/read 8 words");

    // Build expected data; write in a continuous burst (dma_we held high)
    @(posedge clk);
    for (int i = 0; i < 8; i++) begin
      burst_data[i] = {32'(i), 32'(i)};
      dma_waddr     <= AW'(i);
      dma_wdata     <= burst_data[i];
      dma_we        <= 1'b1;
      @(posedge clk);
    end
    dma_we <= 1'b0;
    tick(2);

    // Read back all 8 in sequence
    for (int i = 0; i < 8; i++) begin
      mem_read(AW'(i), rd_burst[i]);
      if (rd_burst[i] !== burst_data[i]) begin
        $display("TB_WEIGHT_BUF: FAIL — TEST2 addr %0d got 64'h%016h expected 64'h%016h",
                 i, rd_burst[i], burst_data[i]);
        $finish;
      end
    end

    $display("TB_WEIGHT_BUF: TEST 2 PASS — 8-word burst verified");
    tick(2);

    // ========================================================
    //  TEST 3 — load_done pulse
    //
    //  Write 16 words in a burst.
    //  load_done must NOT assert during dma_we=1.
    //  load_done must pulse within 2 cycles of dma_we falling.
    // ========================================================
    $display("TB_WEIGHT_BUF: Starting TEST 3 — load_done pulse");

    done_during = 1'b0;
    done_after  = 1'b0;

    @(posedge clk);
    for (int i = 0; i < 16; i++) begin
      dma_waddr <= AW'(i);
      dma_wdata <= {32'(i + 16'h100), 32'(i + 16'h100)};
      dma_we    <= 1'b1;
      @(posedge clk);
      if (load_done) done_during = 1'b1;
    end
    // Drop dma_we — this is the burst-end trigger
    dma_we <= 1'b0;

    // Monitor load_done for up to 2 cycles after dma_we fell
    repeat (3) begin
      @(posedge clk);
      if (load_done) done_after = 1'b1;
    end

    if (done_during) begin
      $display("TB_WEIGHT_BUF: FAIL — TEST3 load_done asserted during active DMA burst");
      $finish;
    end

    if (!done_after) begin
      $display("TB_WEIGHT_BUF: FAIL — TEST3 load_done never pulsed within 2 cycles of last dma_we");
      $finish;
    end

    $display("TB_WEIGHT_BUF: TEST 3 PASS — load_done pulsed cleanly after burst end");
    tick(2);

    // ========================================================
    //  TEST 4 — Read latency is exactly 2 cycles
    //
    //  Write one word to address 5, then assert rd_en at
    //  a known cycle T and verify rd_valid arrives at T+2.
    // ========================================================
    $display("TB_WEIGHT_BUF: Starting TEST 4 — Read latency exactly %0d cycles", RD_LAT);

    dma_write(AW'(5), 64'hC0DE_AABB_CCDD_EF01);
    tick(2);

    // Capture the cycle index when rd_en is asserted
    en_cycle    = timeout_ctr;
    valid_cycle = -1;

    @(posedge clk);
    rd_addr <= AW'(5);
    rd_en   <= 1'b1;
    @(posedge clk);
    rd_en   <= 1'b0;

    // Sample for up to RD_LAT+2 cycles
    for (int i = 1; i <= RD_LAT + 2; i++) begin
      @(posedge clk);
      if (rd_valid && valid_cycle < 0)
        valid_cycle = i;
    end

    if (valid_cycle < 0) begin
      $display("TB_WEIGHT_BUF: FAIL — TEST4 rd_valid never asserted");
      $finish;
    end

    if (valid_cycle !== RD_LAT) begin
      $display("TB_WEIGHT_BUF: FAIL — TEST4 latency mismatch: rd_valid at T+%0d, expected T+%0d",
               valid_cycle, RD_LAT);
      $finish;
    end

    $display("TB_WEIGHT_BUF: TEST 4 PASS — rd_valid arrived exactly %0d cycles after rd_en",
             valid_cycle);
    tick(2);

    // ========================================================
    //  TEST 5 — Address range: highest address (18'h3FFFF)
    // ========================================================
    $display("TB_WEIGHT_BUF: Starting TEST 5 — Highest address 18'h%05h", MAX_ADDR);

    dma_write(AW'(MAX_ADDR), 64'hFFFF_FFFF_FFFF_FFFF);
    tick(2);

    mem_read(AW'(MAX_ADDR), rd_word);

    if (rd_word !== 64'hFFFF_FFFF_FFFF_FFFF) begin
      $display("TB_WEIGHT_BUF: FAIL — TEST5 addr 18'h%05h got 64'h%016h expected 64'hFFFF_FFFF_FFFF_FFFF",
               MAX_ADDR, rd_word);
      $finish;
    end

    // Also verify no address aliasing — address 0 should be unchanged
    // from TEST 2 which wrote {32'h0, 32'h0} there
    mem_read(18'h00000, rd_word);

    if (rd_word !== {32'h0, 32'h0}) begin
      $display("TB_WEIGHT_BUF: FAIL — TEST5 aliasing detected: addr 0 corrupted to 64'h%016h",
               rd_word);
      $finish;
    end

    $display("TB_WEIGHT_BUF: TEST 5 PASS — max address verified, no aliasing");
    tick(2);

    // ========================================================
    //  All Tests Passed
    // ========================================================
    $display("TB_WEIGHT_BUF: PASS");
    $finish;

  end : tb_main

endmodule : tb_npu_weight_buf