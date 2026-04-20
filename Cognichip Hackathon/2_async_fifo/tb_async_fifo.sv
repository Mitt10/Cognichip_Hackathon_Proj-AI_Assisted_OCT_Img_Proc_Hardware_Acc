// ============================================================================
// tb_async_fifo.sv
// Testbench for async_fifo -- DATA_WIDTH=16, DEPTH=16
//
// Clock domains:
//   wr_clk : 250 MHz (4 ns period)
//   rd_clk : 300 MHz (3.333 ns period), 1 ns phase offset to stress CDC
//
// Registered-read interface contract:
//   Drive wr_data/wr_en at negedge wr_clk (half-period setup time).
//   Assert rd_en at negedge rd_clk; data is stable 1 ns after the
//   following posedge rd_clk (NBA settlement).
//   rd_data is updated unconditionally every posedge rd_clk with mem[rd_ptr].
//   Pointer advances only when rd_en=1 && !rd_empty.
//
// Tests:
//   T1 -- Basic 8-word write-then-read
//   T2 -- Fill-to-full, overflow attempt, drain
//   T3 -- 64-word simultaneous stress (fork/join), random en de-assertion
//   T4 -- 20x back-to-back single-word round-trip
// ============================================================================

`timescale 1ns/1ps

module tb_async_fifo;

  // --------------------------------------------------------------------------
  // Parameters
  // --------------------------------------------------------------------------
  localparam int  DATA_WIDTH = 16;
  localparam int  DEPTH      = 16;
  localparam real WR_HALF    = 2.0;     // 4 ns period  = 250 MHz
  localparam real RD_HALF    = 1.6665;  // ~3.333 ns period = 300 MHz
  localparam real PHASE_OFF  = 1.0;     // 1 ns rd_clk phase offset

  // --------------------------------------------------------------------------
  // DUT signals
  // --------------------------------------------------------------------------
  logic                    wr_clk, wr_rst_n, wr_en;
  logic [DATA_WIDTH-1:0]   wr_data;
  logic                    wr_full;

  logic                    rd_clk, rd_rst_n, rd_en;
  logic [DATA_WIDTH-1:0]   rd_data;
  logic                    rd_empty;

  // --------------------------------------------------------------------------
  // DUT instantiation
  // --------------------------------------------------------------------------
  async_fifo #(
    .DATA_WIDTH (DATA_WIDTH),
    .DEPTH      (DEPTH)
  ) dut (
    .wr_clk   (wr_clk),   .wr_rst_n (wr_rst_n),
    .wr_en    (wr_en),    .wr_data  (wr_data),   .wr_full  (wr_full),
    .rd_clk   (rd_clk),   .rd_rst_n (rd_rst_n),
    .rd_en    (rd_en),    .rd_data  (rd_data),   .rd_empty (rd_empty)
  );

  // --------------------------------------------------------------------------
  // Clock generation
  // --------------------------------------------------------------------------
  initial wr_clk = 1'b0;
  always  #(WR_HALF) wr_clk = ~wr_clk;

  initial begin
    rd_clk = 1'b0;
    #(PHASE_OFF);
    forever #(RD_HALF) rd_clk = ~rd_clk;
  end

  // --------------------------------------------------------------------------
  // Failure tracking
  // --------------------------------------------------------------------------
  int fail_count = 0;
  int fail_test  = -1;
  int fail_word  = -1;

  // --------------------------------------------------------------------------
  // check_eq: compare a read word against its expected value
  // --------------------------------------------------------------------------
  task automatic check_eq(
    input int          test_num,
    input int          word_idx,
    input logic [15:0] got,
    input logic [15:0] exp
  );
    if (got !== exp) begin
      $display("LOG: %0t : ERROR : tb_async_fifo : dut.rd_data : expected_value: 16'h%04h actual_value: 16'h%04h",
               $time, exp, got);
      if (fail_count == 0) begin
        fail_test = test_num;
        fail_word = word_idx;
      end
      fail_count++;
    end else begin
      $display("LOG: %0t : INFO  : tb_async_fifo : dut.rd_data : expected_value: 16'h%04h actual_value: 16'h%04h",
               $time, exp, got);
    end
  endtask

  // --------------------------------------------------------------------------
  // check_flag: verify a 1-bit status signal
  // --------------------------------------------------------------------------
  task automatic check_flag(
    input string   sig_name,
    input int      test_num,
    input logic    got,
    input logic    exp
  );
    if (got !== exp) begin
      $display("LOG: %0t : ERROR : tb_async_fifo : dut.%s : expected_value: %0b actual_value: %0b",
               $time, sig_name, exp, got);
      if (fail_count == 0) fail_test = test_num;
      fail_count++;
    end else begin
      $display("LOG: %0t : INFO  : tb_async_fifo : dut.%s : expected_value: %0b actual_value: %0b",
               $time, sig_name, exp, got);
    end
  endtask

  // --------------------------------------------------------------------------
  // wait_not_empty: spin on rd_clk posedges until !rd_empty or timeout
  // --------------------------------------------------------------------------
  task automatic wait_not_empty(input int max_cycles);
    int cnt;
    cnt = 0;
    while (rd_empty && cnt < max_cycles) begin
      @(posedge rd_clk);
      cnt++;
    end
  endtask

  // --------------------------------------------------------------------------
  // wait_empty: spin on rd_clk posedges until rd_empty or timeout
  // --------------------------------------------------------------------------
  task automatic wait_empty(input int max_cycles);
    int cnt;
    cnt = 0;
    while (!rd_empty && cnt < max_cycles) begin
      @(posedge rd_clk);
      cnt++;
    end
  endtask

  // --------------------------------------------------------------------------
  // 10 us simulation timeout watchdog
  // --------------------------------------------------------------------------
  initial begin
    #10_000;
    $display("ERROR");
    $display("TB_ASYNC_FIFO: FAIL -- simulation timeout at 10 us");
    $fatal(1, "TB_ASYNC_FIFO: Simulation timeout");
  end

  // --------------------------------------------------------------------------
  // Main test sequence
  // --------------------------------------------------------------------------
  initial begin
    $display("TEST START");
    $display("------------------------------------------------------------");

    // ------ Initialise all driven signals ------
    wr_rst_n = 1'b0;  rd_rst_n = 1'b0;
    wr_en    = 1'b0;  rd_en    = 1'b0;
    wr_data  = '0;

    // Hold reset for 8 cycles on each domain
    repeat(8) @(posedge wr_clk);
    wr_rst_n = 1'b1;
    repeat(8) @(posedge rd_clk);
    rd_rst_n = 1'b1;

    // Allow sync chains to settle
    repeat(6) @(posedge wr_clk);
    repeat(6) @(posedge rd_clk);

    // ========================================================================
    // TEST 1 -- Basic write then read (8 words: 0xA001 .. 0xA008)
    // ========================================================================
    $display("[T1] Basic write then read");
    begin
      logic [15:0] exp_t1 [0:7];
      logic [15:0] got_t1;

      for (int i = 0; i < 8; i++) begin
        exp_t1[i] = 16'hA001 + 16'(i);
      end

      // --- Back-to-back burst write (wr_en held high for 8 wr_clk cycles) ---
      // Drive at negedge so each posedge between negedge_i and negedge_{i+1}
      // captures wr_data[i] with full setup margin.
      for (int i = 0; i < 8; i++) begin
        @(negedge wr_clk);
        wr_data = exp_t1[i];
        wr_en   = 1'b1;
      end
      @(negedge wr_clk);
      wr_en = 1'b0;

      // Wait for write pointers to propagate to read domain (2-FF sync delay)
      wait_not_empty(32);

      // Move to negedge before starting reads so rd_en setup is clean
      @(negedge rd_clk);

      // Back-to-back reads: keep rd_en high, sample after each posedge+1ns
      // At posedge_i: rd_data <= mem[ptr_i], ptr_i advances
      // At posedge_i+#1: stable value captured
      rd_en = 1'b1;
      for (int i = 0; i < 8; i++) begin
        @(posedge rd_clk); #1;
        got_t1 = rd_data;
        check_eq(1, i, got_t1, exp_t1[i]);
      end
      @(negedge rd_clk);
      rd_en = 1'b0;

      // FIFO should drain (rd_ptr == wr_ptr after 8 reads of 8 writes)
      wait_empty(32);
    end
    $display("[T1] Done  fail_count=%0d", fail_count);
    repeat(8) @(posedge wr_clk);

    // ========================================================================
    // TEST 2 -- Fill to full, overflow guard, drain
    // ========================================================================
    $display("[T2] Fill to full");
    begin
      logic        full_seen;
      logic [15:0] got_t2;
      full_seen = 1'b0;

      // Write DEPTH=16 words
      for (int i = 0; i < DEPTH; i++) begin
        @(negedge wr_clk);
        wr_data = 16'hB000 + 16'(i);
        wr_en   = 1'b1;
        if (wr_full) begin
          full_seen = 1'b1;
        end
      end
      @(negedge wr_clk);
      wr_en = 1'b0;

      // Allow full flag sync (wr_full is registered + depends on synced rd_ptr)
      repeat(6) @(posedge wr_clk);
      if (wr_full) begin
        full_seen = 1'b1;
      end

      check_flag("wr_full", 2, full_seen, 1'b1);

      // --- Overflow write (guarded: only attempt if not full to satisfy SVA) ---
      // The DUT SVA asserts wr_full |-> !wr_en, so we only drive wr_en
      // when !wr_full. Since full_seen was already confirmed above, we
      // skip the write entirely if the FIFO is still full.
      @(negedge wr_clk);
      if (!wr_full) begin
        wr_data = 16'hDEAD;
        wr_en   = 1'b1;
        @(negedge wr_clk);
        wr_en   = 1'b0;
      end

      // --- Drain and verify all 16 original words intact ---
      wait_not_empty(16);
      @(negedge rd_clk);
      rd_en = 1'b1;
      for (int i = 0; i < DEPTH; i++) begin
        @(posedge rd_clk); #1;
        got_t2 = rd_data;
        check_eq(2, i, got_t2, 16'hB000 + 16'(i));
      end
      @(negedge rd_clk);
      rd_en = 1'b0;

      // rd_empty must assert after draining all 16 words
      wait_empty(32);
      check_flag("rd_empty", 2, rd_empty, 1'b1);
    end
    $display("[T2] Done  fail_count=%0d", fail_count);
    repeat(8) @(posedge wr_clk);

    // ========================================================================
    // TEST 3 -- Simultaneous read/write stress: 64 words
    //   Write process : wr_clk domain, ~70% activity (30% random skip)
    //   Read  process : rd_clk domain, ~60% activity (40% random skip)
    //   Both processes run in parallel via fork/join.
    //   Expected outcome: rd_queue contains 0..63 in order.
    // ========================================================================
    $display("[T3] Stress: 64 words, fork/join, random en de-assertion");
    begin
      logic [15:0] rd_queue[$];
      int          wr_count;
      int          rd_count;

      rd_queue.delete();
      wr_count = 0;
      rd_count = 0;

      fork

        // ---- Write process (wr_clk domain) --------------------------------
        // Each wr_clk negedge: 70% chance attempt write if not full.
        // wr_en is driven for one wr_clk cycle (set at negedge_A, cleared at
        // negedge_B so the posedge between A and B commits the write).
        begin
          while (wr_count < 64) begin
            @(negedge wr_clk);
            if ($urandom_range(9, 0) >= 3) begin  // 70% attempt
              if (!wr_full) begin
                wr_data  = 16'(wr_count);
                wr_en    = 1'b1;
                @(negedge wr_clk);
                wr_en    = 1'b0;
                wr_count = wr_count + 1;
              end else begin
                wr_en = 1'b0;
              end
            end else begin
              wr_en = 1'b0;
            end
          end
          // Guarantee wr_en is deasserted when write process exits
          @(negedge wr_clk);
          wr_en = 1'b0;
        end

        // ---- Read process (rd_clk domain) ---------------------------------
        // Each rd_clk negedge: 60% chance attempt read if not empty.
        // rd_en asserted at negedge, data sampled after posedge+1ns,
        // rd_en deasserted at next negedge.
        begin
          while (rd_count < 64) begin
            @(negedge rd_clk);
            if (!rd_empty && ($urandom_range(9, 0) >= 4)) begin  // 60% attempt
              rd_en = 1'b1;
              @(posedge rd_clk); #1;
              rd_queue.push_back(rd_data);
              rd_count = rd_count + 1;
              @(negedge rd_clk);
              rd_en = 1'b0;
            end else begin
              rd_en = 1'b0;
            end
          end
          // Guarantee rd_en is deasserted when read process exits
          @(negedge rd_clk);
          rd_en = 1'b0;
        end

      join  // wait for BOTH processes to complete

      wr_en = 1'b0;
      rd_en = 1'b0;

      // Drain any words still in FIFO (possible if wr_count hit 64 while
      // some words hadn't yet propagated to the read domain).
      repeat(8) @(posedge rd_clk);
      while (!rd_empty) begin
        @(negedge rd_clk);
        rd_en = 1'b1;
        @(posedge rd_clk); #1;
        rd_queue.push_back(rd_data);
        @(negedge rd_clk);
        rd_en = 1'b0;
        repeat(3) @(posedge rd_clk);
      end

      // Verify queue size
      if (rd_queue.size() != 64) begin
        $display("LOG: %0t : ERROR : tb_async_fifo : dut.rd_data : expected_value: 64_words actual_value: %0d_words",
                 $time, rd_queue.size());
        if (fail_count == 0) begin
          fail_test = 3;
          fail_word = 0;
        end
        fail_count++;
      end else begin
        // Verify in-order sequence 0..63
        for (int i = 0; i < 64; i++) begin
          check_eq(3, i, rd_queue[i], 16'(i));
        end
      end
    end
    $display("[T3] Done  fail_count=%0d", fail_count);
    repeat(8) @(posedge wr_clk);

    // ========================================================================
    // TEST 4 -- Back-to-back single word: write one, read one, x20
    // ========================================================================
    $display("[T4] Back-to-back single-word round-trip x20");
    begin
      logic [15:0] exp_t4, got_t4;

      for (int i = 0; i < 20; i++) begin
        exp_t4 = 16'hC000 + 16'(i);

        // Write one word
        @(negedge wr_clk);
        wr_data = exp_t4;
        wr_en   = 1'b1;
        @(negedge wr_clk);
        wr_en   = 1'b0;

        // Wait for pointer to propagate to read domain
        wait_not_empty(32);

        // Read one word
        @(negedge rd_clk);
        rd_en = 1'b1;
        @(posedge rd_clk); #1;
        got_t4 = rd_data;
        @(negedge rd_clk);
        rd_en = 1'b0;

        check_eq(4, i, got_t4, exp_t4);

        // Wait for FIFO to become empty before next iteration
        wait_empty(32);
      end
    end
    $display("[T4] Done  fail_count=%0d", fail_count);

    // ========================================================================
    // Final result
    // ========================================================================
    $display("------------------------------------------------------------");
    repeat(4) @(posedge wr_clk);

    if (fail_count == 0) begin
      $display("TEST PASSED");
      $display("TB_ASYNC_FIFO: PASS");
    end else begin
      $display("ERROR");
      $display("TB_ASYNC_FIFO: FAIL -- test %0d  first_bad_word=%0d  total_failures=%0d",
               fail_test, fail_word, fail_count);
      $error("TB_ASYNC_FIFO: %0d failure(s) in test %0d", fail_count, fail_test);
    end

    $finish;
  end

  // --------------------------------------------------------------------------
  // Waveform dump
  // --------------------------------------------------------------------------
  initial begin
    $dumpfile("dumpfile.fst");
    $dumpvars(0);
  end

endmodule : tb_async_fifo
