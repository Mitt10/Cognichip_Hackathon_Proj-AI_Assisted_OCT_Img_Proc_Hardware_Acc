`timescale 1ns/1ps
// ============================================================
//  TB-14 — tb_bscan_buffer
//  Ping-Pong B-Scan Frame Buffer Testbench
//  DSP clock : 250 MHz (4 ns)
//  NPU clock : 300 MHz (≈3.333 ns)
//  Buffer    : BSCAN_W=16, BSCAN_H=16 (256 pixels, 8-bit each)
// ============================================================

module tb_bscan_buffer;

  // --------------------------------------------------------
  //  Parameters
  // --------------------------------------------------------
  localparam int BSCAN_W       = 16;
  localparam int BSCAN_H       = 16;
  localparam int FRAME_PIXELS  = BSCAN_W * BSCAN_H;   // 256
  localparam int ADDR_W        = $clog2(FRAME_PIXELS); // 8
  localparam int DATA_W        = 8;

  localparam real DSP_CLK_NS   = 4.0;                  // 250 MHz
  localparam real NPU_CLK_NS   = 10.0/3.0;             // 300 MHz

  localparam real TIMEOUT_US   = 200.0;
  localparam int  DSP_TIMEOUT  = int'(TIMEOUT_US * 1000.0 / DSP_CLK_NS);
  localparam int  NPU_TIMEOUT  = int'(TIMEOUT_US * 1000.0 / NPU_CLK_NS);

  // CDC synchroniser settling — wait this many NPU cycles after
  // a DSP-domain event before sampling NPU-domain signals.
  localparam int  CDC_SETTLE   = 6;

  // --------------------------------------------------------
  //  DUT Port Declarations
  // --------------------------------------------------------

  // Clocks / resets
  logic        dsp_clk;
  logic        npu_clk;
  logic        dsp_rst_n;
  logic        npu_rst_n;

  // DSP AXI4-Stream write port  (dsp_clk domain)
  logic        dsp_tvalid;
  logic        dsp_tready;
  logic  [7:0] dsp_tdata;
  logic        dsp_tlast;

  // DSP status (dsp_clk domain)
  logic        dsp_frame_done;   // pulses one cycle after tlast accepted

  // Swap handshake
  // dsp_clk domain : dsp_frame_done triggers swap request inside DUT
  // npu_clk domain : npu_frame_ready pulses when new frame is readable
  logic        npu_frame_ready;

  // NPU read/write port  (npu_clk domain)
  logic [ADDR_W-1:0] npu_raddr;
  logic              npu_re;
  logic  [DATA_W-1:0] npu_rdata;
  logic [ADDR_W-1:0] npu_waddr;
  logic  [DATA_W-1:0] npu_wdata;
  logic              npu_we;

  // Host read port  (npu_clk domain, read-only)
  logic [ADDR_W-1:0] host_raddr;
  logic              host_re;
  logic  [DATA_W-1:0] host_rdata;

  // --------------------------------------------------------
  //  DUT Instantiation
  // --------------------------------------------------------
  bscan_buffer #(
    .BSCAN_W (BSCAN_W),
    .BSCAN_H (BSCAN_H)
  ) dut (
    .dsp_clk        (dsp_clk),
    .npu_clk        (npu_clk),
    .dsp_rst_n      (dsp_rst_n),
    .npu_rst_n      (npu_rst_n),
    // DSP AXI4-Stream
    .dsp_tvalid     (dsp_tvalid),
    .dsp_tready     (dsp_tready),
    .dsp_tdata      (dsp_tdata),
    .dsp_tlast      (dsp_tlast),
    // Status
    .dsp_frame_done (dsp_frame_done),
    .npu_frame_ready(npu_frame_ready),
    // NPU port
    .npu_raddr      (npu_raddr),
    .npu_re         (npu_re),
    .npu_rdata      (npu_rdata),
    .npu_waddr      (npu_waddr),
    .npu_wdata      (npu_wdata),
    .npu_we         (npu_we),
    // Host read port
    .host_raddr     (host_raddr),
    .host_re        (host_re),
    .host_rdata     (host_rdata)
  );

  // --------------------------------------------------------
  //  Clock Generation
  // --------------------------------------------------------
  initial dsp_clk = 1'b0;
  always  #(DSP_CLK_NS / 2.0) dsp_clk = ~dsp_clk;

  initial npu_clk = 1'b0;
  always  #(NPU_CLK_NS / 2.0) npu_clk = ~npu_clk;

  // --------------------------------------------------------
  //  Timeout Watchdogs (independent per domain)
  // --------------------------------------------------------
  int dsp_ctr, npu_ctr;

  always @(posedge dsp_clk) begin
    if (!dsp_rst_n) dsp_ctr <= 0;
    else begin
      dsp_ctr <= dsp_ctr + 1;
      if (dsp_ctr >= DSP_TIMEOUT) begin
        $display("TB_BSCAN_BUF: FAIL — DSP-domain TIMEOUT after %.0f µs", TIMEOUT_US);
        $finish;
      end
    end
  end

  always @(posedge npu_clk) begin
    if (!npu_rst_n) npu_ctr <= 0;
    else begin
      npu_ctr <= npu_ctr + 1;
      if (npu_ctr >= NPU_TIMEOUT) begin
        $display("TB_BSCAN_BUF: FAIL — NPU-domain TIMEOUT after %.0f µs", TIMEOUT_US);
        $finish;
      end
    end
  end

  // --------------------------------------------------------
  //  Clock-domain helpers
  // --------------------------------------------------------
  task automatic dsp_tick(input int n = 1);
    repeat (n) @(posedge dsp_clk);
  endtask

  task automatic npu_tick(input int n = 1);
    repeat (n) @(posedge npu_clk);
  endtask

  // --------------------------------------------------------
  //  DSP AXI4-Stream write task
  //  Streams 'pixels[0..FRAME_PIXELS-1]' into the DUT.
  //  tlast is asserted on the final sample.
  //  done_seen is set 1 if dsp_frame_done pulsed during the stream.
  // --------------------------------------------------------
  task automatic dsp_write_frame(
    input  logic [DATA_W-1:0] pixels [0:FRAME_PIXELS-1],
    output logic              done_seen
  );
    done_seen = 1'b0;
    for (int i = 0; i < FRAME_PIXELS; i++) begin
      @(posedge dsp_clk);
      dsp_tvalid <= 1'b1;
      dsp_tdata  <= pixels[i];
      dsp_tlast  <= (i % BSCAN_W == BSCAN_W-1) ? 1'b1 : 1'b0;

      // Honour back-pressure
      do @(posedge dsp_clk);
      while (!dsp_tready);

      if (dsp_frame_done) done_seen = 1'b1;
    end
    dsp_tvalid <= 1'b0;
    dsp_tlast  <= 1'b0;

    // Watch for done one extra cycle (pipelined DUTs fire it a cycle late)
    @(posedge dsp_clk);
    if (dsp_frame_done) done_seen = 1'b1;
  endtask

  // --------------------------------------------------------
  //  Wait for npu_frame_ready to pulse (NPU clock domain)
  // --------------------------------------------------------
  task automatic wait_npu_frame_ready();
    do npu_tick(); while (!npu_frame_ready);
    npu_tick(CDC_SETTLE);   // allow any sync chain to settle
  endtask

  // --------------------------------------------------------
  //  NPU read one pixel (registered read: addr presented on
  //  cycle N, data valid on cycle N+1)
  // --------------------------------------------------------
  task automatic npu_read_pixel(
    input  int              addr,
    output logic [DATA_W-1:0] data
  );
    @(posedge npu_clk);
    npu_raddr <= ADDR_W'(addr);
    npu_re    <= 1'b1;
    @(posedge npu_clk);
    npu_re    <= 1'b0;
    @(posedge npu_clk);
    data = npu_rdata;
  endtask

  // --------------------------------------------------------
  //  NPU write one pixel
  // --------------------------------------------------------
  task automatic npu_write_pixel(
    input int              addr,
    input logic [DATA_W-1:0] data
  );
    @(posedge npu_clk);
    npu_waddr <= ADDR_W'(addr);
    npu_wdata <= data;
    npu_we    <= 1'b1;
    @(posedge npu_clk);
    npu_we    <= 1'b0;
  endtask

  // --------------------------------------------------------
  //  Host read one pixel
  // --------------------------------------------------------
  task automatic host_read_pixel(
    input  int              addr,
    output logic [DATA_W-1:0] data
  );
    @(posedge npu_clk);
    host_raddr <= ADDR_W'(addr);
    host_re    <= 1'b1;
    @(posedge npu_clk);
    host_re    <= 1'b0;
    @(posedge npu_clk);
    data = host_rdata;
  endtask

  // --------------------------------------------------------
  //  NPU read full frame into array
  // --------------------------------------------------------
  task automatic npu_read_frame(output logic [DATA_W-1:0] buf_out [0:FRAME_PIXELS-1]);
    for (int i = 0; i < FRAME_PIXELS; i++)
      npu_read_pixel(i, buf_out[i]);
  endtask

  // --------------------------------------------------------
  //  Declare pixel arrays and scratch variables at module scope
  //  (avoids 'static variable' initializer errors)
  // --------------------------------------------------------
  logic [DATA_W-1:0] pix_ramp   [0:FRAME_PIXELS-1];
  logic [DATA_W-1:0] pix_AA     [0:FRAME_PIXELS-1];
  logic [DATA_W-1:0] pix_BB     [0:FRAME_PIXELS-1];
  logic [DATA_W-1:0] pix_C      [0:FRAME_PIXELS-1];
  logic [DATA_W-1:0] pix_chk    [0:FRAME_PIXELS-1];
  logic [DATA_W-1:0] rd_frame   [0:FRAME_PIXELS-1];

  logic done_flag;
  logic [DATA_W-1:0] rd_pix;
  logic [DATA_W-1:0] rd_pix2;

  // --------------------------------------------------------
  //  Main Test Sequence
  // --------------------------------------------------------
  initial begin : tb_main

    // --- Drive all inputs to safe defaults ---
    dsp_rst_n  <= 1'b0;
    npu_rst_n  <= 1'b0;
    dsp_tvalid <= 1'b0;
    dsp_tdata  <= '0;
    dsp_tlast  <= 1'b0;
    npu_raddr  <= '0;
    npu_re     <= 1'b0;
    npu_waddr  <= '0;
    npu_wdata  <= '0;
    npu_we     <= 1'b0;
    host_raddr <= '0;
    host_re    <= 1'b0;

    // --- Synchronous reset (hold for 10 cycles of the slower clock) ---
    dsp_tick(10);
    npu_tick(10);
    dsp_rst_n <= 1'b1;
    npu_rst_n <= 1'b1;
    dsp_tick(4);
    npu_tick(4);

    // ============================================================
    //  TEST 1 — Basic write and read (no overlap)
    //           Ramp pattern: pixel[i] = i (0..255)
    // ============================================================
    $display("TB_BSCAN_BUF: Starting TEST 1 — Basic write and read");

    // Build ramp pixel array
    for (int i = 0; i < FRAME_PIXELS; i++)
      pix_ramp[i] = DATA_W'(i);

    // DSP writes full frame
    dsp_write_frame(pix_ramp, done_flag);

    if (!done_flag) begin
      $display("TB_BSCAN_BUF: FAIL — TEST1 dsp_frame_done never pulsed");
      $finish;
    end

    // Wait for NPU side to see the new frame
    wait_npu_frame_ready();

    // NPU reads all 256 pixels
    npu_read_frame(rd_frame);

    for (int i = 0; i < FRAME_PIXELS; i++) begin
      if (rd_frame[i] !== DATA_W'(i)) begin
        $display("TB_BSCAN_BUF: FAIL — TEST1 addr %0d got 8'h%02h expected 8'h%02h",
                 i, rd_frame[i], DATA_W'(i));
        $finish;
      end
    end

    $display("TB_BSCAN_BUF: TEST 1 PASS — ramp pattern verified across 256 pixels");
    dsp_tick(4);

    // ============================================================
    //  TEST 2 — Ping-pong: concurrent DSP write + NPU read
    //
    //  Phase 2a: DSP writes frame A (0xAA) → frame_done → swap.
    //  Phase 2b: Simultaneously DSP writes frame B (0xBB) into
    //            the newly freed bank while NPU reads frame A.
    //  Phase 2c: After frame B done + npu_frame_ready, NPU reads B.
    // ============================================================
    $display("TB_BSCAN_BUF: Starting TEST 2 — Ping-pong concurrent access");

    for (int i = 0; i < FRAME_PIXELS; i++) begin
      pix_AA[i] = 8'hAA;
      pix_BB[i] = 8'hBB;
    end

    // Phase 2a: write frame A
    dsp_write_frame(pix_AA, done_flag);
    if (!done_flag) begin
      $display("TB_BSCAN_BUF: FAIL — TEST2 frame A dsp_frame_done never pulsed");
      $finish;
    end

    // Phase 2b: fork DSP writing B  ||  NPU reading A
    fork
      begin : dsp_write_b
        dsp_write_frame(pix_BB, done_flag);
      end

      begin : npu_read_a
        // Wait until NPU can see frame A
        wait_npu_frame_ready();
        npu_read_frame(rd_frame);

        for (int i = 0; i < FRAME_PIXELS; i++) begin
          if (rd_frame[i] !== 8'hAA) begin
            $display("TB_BSCAN_BUF: FAIL — TEST2 frame A addr %0d got 8'h%02h expected 8'hAA",
                     i, rd_frame[i]);
            $finish;
          end
          // Cross-contamination guard
          if (rd_frame[i] === 8'hBB) begin
            $display("TB_BSCAN_BUF: FAIL — TEST2 cross-contamination at addr %0d (got 8'hBB)",
                     i);
            $finish;
          end
        end
      end
    join

    $display("TB_BSCAN_BUF: TEST 2a PASS — frame A = 8'hAA, no contamination from B");

    // Phase 2c: verify frame B
    wait_npu_frame_ready();
    npu_read_frame(rd_frame);

    for (int i = 0; i < FRAME_PIXELS; i++) begin
      if (rd_frame[i] !== 8'hBB) begin
        $display("TB_BSCAN_BUF: FAIL — TEST2 frame B addr %0d got 8'h%02h expected 8'hBB",
                 i, rd_frame[i]);
        $finish;
      end
    end

    $display("TB_BSCAN_BUF: TEST 2b PASS — frame B = 8'hBB intact");
    dsp_tick(4);

    // ============================================================
    //  TEST 3 — NPU write-back (overlay)
    //
    //  DSP writes frame C (all 0x10).
    //  NPU reads pixel 0 → verify 0x10.
    //  NPU writes pixel 0 = 0xFF.
    //  NPU reads pixel 0 again → verify 0xFF.
    //  NPU reads pixel 1        → verify unchanged 0x10.
    // ============================================================
    $display("TB_BSCAN_BUF: Starting TEST 3 — NPU write-back overlay");

    for (int i = 0; i < FRAME_PIXELS; i++)
      pix_C[i] = 8'h10;

    dsp_write_frame(pix_C, done_flag);
    if (!done_flag) begin
      $display("TB_BSCAN_BUF: FAIL — TEST3 frame C dsp_frame_done never pulsed");
      $finish;
    end

    wait_npu_frame_ready();

    // Read pixel 0 — expect 0x10
    npu_read_pixel(0, rd_pix);
    if (rd_pix !== 8'h10) begin
      $display("TB_BSCAN_BUF: FAIL — TEST3 initial read addr 0 got 8'h%02h expected 8'h10",
               rd_pix);
      $finish;
    end

    // Write-back pixel 0 = 0xFF
    npu_write_pixel(0, 8'hFF);
    npu_tick(2);

    // Re-read pixel 0 — must now be 0xFF
    npu_read_pixel(0, rd_pix);
    if (rd_pix !== 8'hFF) begin
      $display("TB_BSCAN_BUF: FAIL — TEST3 post-writeback addr 0 got 8'h%02h expected 8'hFF",
               rd_pix);
      $finish;
    end

    // Read pixel 1 — must still be 0x10
    npu_read_pixel(1, rd_pix2);
    if (rd_pix2 !== 8'h10) begin
      $display("TB_BSCAN_BUF: FAIL — TEST3 adjacent pixel addr 1 got 8'h%02h expected 8'h10",
               rd_pix2);
      $finish;
    end

    $display("TB_BSCAN_BUF: TEST 3 PASS — write-back 8'hFF at addr 0, pixel 1 unchanged");
    npu_tick(4);

    // ============================================================
    //  TEST 4 — Host read port
    //
    //  Frame C (with pixel 0 overwritten to 0xFF) is still in the
    //  NPU bank.  Read via host port and verify.
    //    pixel 0  → 0xFF  (written back in TEST 3)
    //    pixels 1..255 → 0x10
    // ============================================================
    $display("TB_BSCAN_BUF: Starting TEST 4 — Host read port");

    for (int i = 0; i < FRAME_PIXELS; i++) begin
      logic [DATA_W-1:0] expected_val;
      logic [DATA_W-1:0] hval;
      expected_val = (i == 0) ? 8'hFF : 8'h10;

      host_read_pixel(i, hval);

      if (hval !== expected_val) begin
        $display("TB_BSCAN_BUF: FAIL — TEST4 host read addr %0d got 8'h%02h expected 8'h%02h",
                 i, hval, expected_val);
        $finish;
      end
    end

    $display("TB_BSCAN_BUF: TEST 4 PASS — host port verified all 256 pixels");
    npu_tick(4);

    // ============================================================
    //  TEST 5 — Clock domain crossing integrity
    //
    //  Checkerboard pattern written from dsp_clk domain:
    //    pixel[i] = 0xA5 if i is even, 0x5A if i is odd.
    //  Read entirely from npu_clk domain; assert zero bit errors.
    // ============================================================
    $display("TB_BSCAN_BUF: Starting TEST 5 — CDC checkerboard integrity");

    for (int i = 0; i < FRAME_PIXELS; i++)
      pix_chk[i] = (i % 2 == 0) ? 8'hA5 : 8'h5A;

    dsp_write_frame(pix_chk, done_flag);
    if (!done_flag) begin
      $display("TB_BSCAN_BUF: FAIL — TEST5 checkerboard dsp_frame_done never pulsed");
      $finish;
    end

    wait_npu_frame_ready();
    npu_read_frame(rd_frame);

    begin : t5_check
      int err_count;
      err_count = 0;
      for (int i = 0; i < FRAME_PIXELS; i++) begin
        logic [DATA_W-1:0] exp_val;
        exp_val = (i % 2 == 0) ? 8'hA5 : 8'h5A;
        if (rd_frame[i] !== exp_val) begin
          $display("TB_BSCAN_BUF: FAIL — TEST5 CDC addr %0d got 8'h%02h expected 8'h%02h",
                   i, rd_frame[i], exp_val);
          err_count++;
          if (err_count >= 4) begin
            $display("TB_BSCAN_BUF: FAIL — TEST5 suppressing further CDC errors (%0d+)", err_count);
            $finish;
          end
        end
      end
      if (err_count > 0) begin
        $display("TB_BSCAN_BUF: FAIL — TEST5 %0d CDC bit error(s)", err_count);
        $finish;
      end
    end

    $display("TB_BSCAN_BUF: TEST 5 PASS — checkerboard zero CDC bit errors");
    npu_tick(4);

    // ============================================================
    //  All Tests Passed
    // ============================================================
    $display("TB_BSCAN_BUF: PASS");
    $finish;

  end : tb_main

endmodule : tb_bscan_buffer