// ============================================================================
// tb_sd_oct_frontend.sv
// Testbench for sd_oct_frontend — SD-OCT Camera Receiver / ROI Cropper
//
// Clock : 200 MHz camera pixel clock
// Config: roi_start=512, roi_end=1535 (1024 active columns), gain_shift=2
//
// Tests:
//   TEST 1 — ROI cropping: 1024 words, correct gain-shifted values, tlast position
//   TEST 2 — ascan_idx increments: 0,1,2,3 on successive lines
//   TEST 3 — mode field in tuser: bits[3:2] == 2'b00 (SD-OCT)
//   TEST 4 — Backpressure: stall m_tready pre-ROI; all 1024 ROI words received
//   TEST 5 — Frame valid reset: ascan_idx resets after cam_fval drops
//
// FIFO depth is 16; m_tready is held high during ROI to prevent overflow.
// Backpressure test stalls during the pre-ROI (columns 0..511) window only.
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

`timescale 1ns/1ps

module tb_sd_oct_frontend;

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    localparam real CLK_PERIOD  = 5.0;   // 200 MHz
    localparam int  ROI_START   = 512;
    localparam int  ROI_END     = 1535;
    localparam int  ROI_SIZE    = ROI_END - ROI_START + 1; // 1024
    localparam int  GAIN_SHIFT  = 2;
    localparam int  LINE_PIXELS = 2048;
    localparam int  TIMEOUT_CYC = 100000; // watchdog

    // -----------------------------------------------------------------------
    // Clock / reset
    // -----------------------------------------------------------------------
    logic clk;
    logic rst_n;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -----------------------------------------------------------------------
    // DUT ports
    // -----------------------------------------------------------------------
    logic [15:0] cam_pixel;
    logic        cam_valid;
    logic        cam_lval;
    logic        cam_fval;

    logic [10:0] roi_start;
    logic [10:0] roi_end;
    logic [3:0]  gain_shift;

    logic [15:0] m_tdata;
    logic        m_tvalid;
    logic        m_tlast;
    logic [31:0] m_tuser;
    logic        m_tready;

    logic [7:0]  galvo_x;
    logic [7:0]  galvo_y;

    // -----------------------------------------------------------------------
    // DUT instantiation
    // -----------------------------------------------------------------------
    sd_oct_frontend dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .cam_pixel  (cam_pixel),
        .cam_valid  (cam_valid),
        .cam_lval   (cam_lval),
        .cam_fval   (cam_fval),
        .roi_start  (roi_start),
        .roi_end    (roi_end),
        .gain_shift (gain_shift),
        .m_tdata    (m_tdata),
        .m_tvalid   (m_tvalid),
        .m_tlast    (m_tlast),
        .m_tuser    (m_tuser),
        .m_tready   (m_tready),
        .galvo_x    (galvo_x),
        .galvo_y    (galvo_y)
    );

    // -----------------------------------------------------------------------
    // Waveform dump
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("dumpfile.fst");
        $dumpvars(0);
    end

    // -----------------------------------------------------------------------
    // Output capture queues (populated by concurrent collector)
    // -----------------------------------------------------------------------
    logic [15:0] out_data[$];
    logic [31:0] out_user[$];
    logic        out_last[$];

    always @(posedge clk) begin
        if (m_tvalid && m_tready) begin
            out_data.push_back(m_tdata);
            out_user.push_back(m_tuser);
            out_last.push_back(m_tlast);
        end
    end

    // -----------------------------------------------------------------------
    // Test state
    // -----------------------------------------------------------------------
    int    fail_count;
    string current_test;

    // -----------------------------------------------------------------------
    // Task: send_line
    //   Drives one camera line: 2048 pixels (values 0..2047).
    //   cam_valid = cam_lval. cam_fval must be driven by caller.
    //   m_tready must be managed by caller to avoid FIFO overflow.
    // -----------------------------------------------------------------------
    task automatic send_line();
        int col;
        // One idle cycle between lines
        @(posedge clk); #1;
        cam_lval  = 1'b1;
        cam_valid = 1'b1;

        for (col = 0; col < LINE_PIXELS; col++) begin
            cam_pixel = 16'(col);
            @(posedge clk); #1;
        end

        // De-assert lval at end of line
        cam_lval  = 1'b0;
        cam_valid = 1'b0;
        cam_pixel = 16'h0000;
    endtask

    // -----------------------------------------------------------------------
    // Task: drain_output
    //   Waits until the output queue has at least 'count' entries, or timeout.
    // -----------------------------------------------------------------------
    task automatic drain_output(input int count);
        int timeout;
        timeout = 0;
        while (out_data.size() < count) begin
            @(posedge clk);
            timeout++;
            if (timeout > TIMEOUT_CYC) begin
                $display("LOG: %0t : ERROR : tb_sd_oct_frontend : drain_output : expected_value: %0d actual_value: %0d (TIMEOUT)",
                         $time, count, out_data.size());
                fail_count++;
                return;
            end
        end
    endtask

    // -----------------------------------------------------------------------
    // Task: clear_queues
    // -----------------------------------------------------------------------
    task automatic clear_queues();
        out_data.delete();
        out_user.delete();
        out_last.delete();
    endtask

    // -----------------------------------------------------------------------
    // Main test sequence
    // -----------------------------------------------------------------------
    int          i;
    logic [15:0] exp_pix;
    logic [15:0] exp_shifted;
    int          tlast_cnt;
    int          tlast_idx;
    logic [15:0] ascan_indices[0:3];

    initial begin
        $display("TEST START");

        // ---- Initialise ----
        fail_count  = 0;
        rst_n       = 1'b0;
        cam_pixel   = 16'h0000;
        cam_valid   = 1'b0;
        cam_lval    = 1'b0;
        cam_fval    = 1'b0;
        m_tready    = 1'b0;
        roi_start   = 11'(ROI_START);
        roi_end     = 11'(ROI_END);
        gain_shift  = 4'(GAIN_SHIFT);
        galvo_x     = 8'h00;
        galvo_y     = 8'h00;

        // Release reset after 5 cycles
        repeat (5) @(posedge clk);
        #1; rst_n = 1'b1;
        repeat (3) @(posedge clk);

        // =================================================================
        // TEST 1 — ROI cropping: 1024 words, correct pixel values, tlast
        // =================================================================
        current_test = "TEST1_ROI_CROP";
        $display("LOG: %0t : INFO : tb_sd_oct_frontend : starting %s", $time, current_test);

        clear_queues();
        #1;
        cam_fval = 1'b1;
        m_tready = 1'b1;    // Accept data continuously

        send_line();        // Send 2048 pixels

        // Wait for all 1024 ROI words to arrive
        drain_output(ROI_SIZE);

        // Let pipeline flush (a few extra cycles)
        repeat (10) @(posedge clk);

        // Check word count
        if (out_data.size() != ROI_SIZE) begin
            $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tdata[count] : expected_value: %0d actual_value: %0d",
                     $time, ROI_SIZE, out_data.size());
            fail_count++;
        end else begin
            $display("LOG: %0t : INFO : tb_sd_oct_frontend : dut.m_tdata[count] : expected_value: %0d actual_value: %0d",
                     $time, ROI_SIZE, out_data.size());
        end

        // Check pixel values and tlast
        tlast_cnt = 0;
        for (i = 0; i < int'(out_data.size()) && i < ROI_SIZE; i++) begin
            // Original pixel value = ROI_START + i (column index)
            exp_pix     = 16'(ROI_START + i);
            // gain_shift=2: arithmetic right shift of signed value
            exp_shifted = 16'($signed(exp_pix) >>> GAIN_SHIFT);

            if (out_data[i] !== exp_shifted) begin
                $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tdata[word %0d] : expected_value: 16'h%04X actual_value: 16'h%04X",
                         $time, i, exp_shifted, out_data[i]);
                fail_count++;
                if (fail_count > 5) begin
                    $display("LOG: %0t : WARNING : tb_sd_oct_frontend : stopping pixel checks after 5 failures", $time);
                    break;
                end
            end

            // Count tlast assertions
            if (out_last[i]) begin
                tlast_cnt++;
                tlast_idx = i;
            end
        end

        // tlast must fire exactly once, on the last word
        if (tlast_cnt != 1) begin
            $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tlast[count] : expected_value: 1 actual_value: %0d",
                     $time, tlast_cnt);
            fail_count++;
        end else if (tlast_idx != ROI_SIZE - 1) begin
            $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tlast[position] : expected_value: %0d actual_value: %0d",
                     $time, ROI_SIZE-1, tlast_idx);
            fail_count++;
        end else begin
            $display("LOG: %0t : INFO : tb_sd_oct_frontend : dut.m_tlast : expected_value: word %0d actual_value: word %0d",
                     $time, ROI_SIZE-1, tlast_idx);
        end

        // =================================================================
        // TEST 2 — ascan_idx increments (4 lines → indices 0,1,2,3)
        // =================================================================
        current_test = "TEST2_ASCAN_IDX";
        $display("LOG: %0t : INFO : tb_sd_oct_frontend : starting %s", $time, current_test);

        clear_queues();
        m_tready = 1'b1;

        // cam_fval already high; send 4 lines
        repeat (4) begin
            send_line();
            repeat (5) @(posedge clk); // inter-line gap
        end

        // Collect at least 4*ROI_SIZE words
        drain_output(4 * ROI_SIZE);
        repeat (10) @(posedge clk);

        // Extract ascan_idx from each tlast word
        begin
            int line_n;
            line_n = 0;
            for (i = 0; i < int'(out_data.size()); i++) begin
                if (out_last[i]) begin
                    if (line_n < 4) begin
                        ascan_indices[line_n] = out_user[i][31:16];
                        $display("LOG: %0t : INFO : tb_sd_oct_frontend : dut.m_tuser.ascan_idx[line %0d] : expected_value: %0d actual_value: %0d",
                                 $time, line_n, line_n, ascan_indices[line_n]);
                        if (ascan_indices[line_n] !== 16'(line_n)) begin
                            $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tuser.ascan_idx[line %0d] : expected_value: %0d actual_value: %0d",
                                     $time, line_n, line_n, ascan_indices[line_n]);
                            fail_count++;
                        end
                    end
                    line_n++;
                end
            end
            if (line_n < 4) begin
                $display("LOG: %0t : ERROR : tb_sd_oct_frontend : tlast count : expected_value: >=4 actual_value: %0d",
                         $time, line_n);
                fail_count++;
            end
        end

        // =================================================================
        // TEST 3 — mode field in tuser: bits[3:2] == 2'b00 (SD-OCT)
        // =================================================================
        current_test = "TEST3_MODE_FIELD";
        $display("LOG: %0t : INFO : tb_sd_oct_frontend : starting %s", $time, current_test);

        // Check the first captured word (out_data still holds last burst)
        if (out_data.size() > 0) begin
            logic [1:0] mode_field;
            mode_field = out_user[0][3:2];
            if (mode_field !== 2'b00) begin
                $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tuser.mode : expected_value: 2'b00 actual_value: 2'b%02b",
                         $time, mode_field);
                fail_count++;
            end else begin
                $display("LOG: %0t : INFO : tb_sd_oct_frontend : dut.m_tuser.mode : expected_value: 2'b00 actual_value: 2'b%02b",
                         $time, mode_field);
            end
        end

        // =================================================================
        // TEST 4 — Backpressure
        //   Stall m_tready for 50 cycles during pre-ROI window (cols 0..511).
        //   ROI starts at column 512, so FIFO is empty when ROI pixels arrive.
        //   All 1024 ROI words must still be received.
        // =================================================================
        current_test = "TEST4_BACKPRESSURE";
        $display("LOG: %0t : INFO : tb_sd_oct_frontend : starting %s", $time, current_test);

        clear_queues();

        // --- Drive line manually to control m_tready timing ---
        @(posedge clk); #1;
        cam_lval  = 1'b1;
        cam_valid = 1'b1;
        m_tready  = 1'b0;  // Stall downstream from the start

        // Pre-ROI portion: columns 0..49 with m_tready=0 (50 cycles stall)
        for (i = 0; i < 50; i++) begin
            cam_pixel = 16'(i);
            @(posedge clk); #1;
        end

        // Release backpressure before ROI starts (columns 50..511)
        m_tready = 1'b1;
        for (i = 50; i < ROI_START; i++) begin
            cam_pixel = 16'(i);
            @(posedge clk); #1;
        end

        // ROI portion: columns 512..1535 (1024 pixels), m_tready=1
        for (i = ROI_START; i <= ROI_END; i++) begin
            cam_pixel = 16'(i);
            @(posedge clk); #1;
        end

        // Post-ROI portion: columns 1536..2047
        for (i = ROI_END + 1; i < LINE_PIXELS; i++) begin
            cam_pixel = 16'(i);
            @(posedge clk); #1;
        end

        cam_lval  = 1'b0;
        cam_valid = 1'b0;
        cam_pixel = 16'h0;

        // Drain all output
        drain_output(ROI_SIZE);
        repeat (10) @(posedge clk);

        // Verify no words were lost
        if (out_data.size() != ROI_SIZE) begin
            $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tdata[backpressure count] : expected_value: %0d actual_value: %0d",
                     $time, ROI_SIZE, out_data.size());
            fail_count++;
        end else begin
            $display("LOG: %0t : INFO : tb_sd_oct_frontend : dut.m_tdata[backpressure count] : expected_value: %0d actual_value: %0d",
                     $time, ROI_SIZE, out_data.size());
        end

        // Verify no duplicates/drops in pixel values
        for (i = 0; i < int'(out_data.size()) && i < ROI_SIZE; i++) begin
            exp_pix     = 16'(ROI_START + i);
            exp_shifted = 16'($signed(exp_pix) >>> GAIN_SHIFT);
            if (out_data[i] !== exp_shifted) begin
                $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tdata[bp word %0d] : expected_value: 16'h%04X actual_value: 16'h%04X",
                         $time, i, exp_shifted, out_data[i]);
                fail_count++;
                if (fail_count > 10) break;
            end
        end

        // =================================================================
        // TEST 5 — Frame valid reset of ascan_idx
        //   Send 3 lines, drop fval for 5 cycles, send 3 more lines.
        //   ascan_idx on tlast of second burst should start at 0.
        // =================================================================
        current_test = "TEST5_FVAL_RESET";
        $display("LOG: %0t : INFO : tb_sd_oct_frontend : starting %s", $time, current_test);

        clear_queues();
        m_tready = 1'b1;
        cam_fval = 1'b1;

        // First burst: 3 lines
        repeat (3) begin
            send_line();
            repeat (3) @(posedge clk);
        end
        drain_output(3 * ROI_SIZE);
        repeat (5) @(posedge clk);

        // Drop cam_fval to trigger ascan_idx reset
        #1; cam_fval = 1'b0;
        repeat (5) @(posedge clk);
        #1; cam_fval = 1'b1;
        repeat (3) @(posedge clk);

        // Clear the queue and send second burst (3 lines)
        clear_queues();
        repeat (3) begin
            send_line();
            repeat (3) @(posedge clk);
        end
        drain_output(3 * ROI_SIZE);
        repeat (10) @(posedge clk);

        // Check ascan_idx on each tlast in the second burst = 0, 1, 2
        begin
            int line_n;
            line_n = 0;
            for (i = 0; i < int'(out_data.size()); i++) begin
                if (out_last[i]) begin
                    if (line_n < 3) begin
                        logic [15:0] idx;
                        idx = out_user[i][31:16];
                        $display("LOG: %0t : INFO : tb_sd_oct_frontend : dut.m_tuser.ascan_idx[burst2 line %0d] : expected_value: %0d actual_value: %0d",
                                 $time, line_n, line_n, idx);
                        if (idx !== 16'(line_n)) begin
                            $display("LOG: %0t : ERROR : tb_sd_oct_frontend : dut.m_tuser.ascan_idx[burst2 line %0d] : expected_value: %0d actual_value: %0d",
                                     $time, line_n, line_n, idx);
                            fail_count++;
                        end
                    end
                    line_n++;
                end
            end
            if (line_n < 3) begin
                $display("LOG: %0t : ERROR : tb_sd_oct_frontend : fval_reset tlast count : expected_value: >=3 actual_value: %0d",
                         $time, line_n);
                fail_count++;
            end
        end

        // =================================================================
        // Final result
        // =================================================================
        #1; cam_fval = 1'b0;
        repeat (5) @(posedge clk);

        if (fail_count == 0) begin
            $display("TB_SD_FE: PASS");
            $display("TEST PASSED");
        end else begin
            $display("ERROR");
            $display("TB_SD_FE: FAIL — %0d failure(s), last test: %s", fail_count, current_test);
            $display("TEST FAILED");
            $error("TB_SD_FE: %0d failure(s)", fail_count);
        end

        $finish;
    end

    // -----------------------------------------------------------------------
    // Simulation timeout watchdog
    // -----------------------------------------------------------------------
    initial begin
        repeat (TIMEOUT_CYC) @(posedge clk);
        $display("ERROR");
        $display("TEST FAILED");
        $fatal(1, "TIMEOUT: simulation exceeded %0d clock cycles", TIMEOUT_CYC);
    end

endmodule : tb_sd_oct_frontend
