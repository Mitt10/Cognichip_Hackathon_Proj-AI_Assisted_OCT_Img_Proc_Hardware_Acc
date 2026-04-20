// ============================================================================
// tb_oct_ctrl_regs.sv
// Testbench for oct_ctrl_regs — AXI4-Lite Register File
//
// Tests:
//   TEST 1 — Reset values (CTRL == 0x41, STATUS == 0)
//   TEST 2 — Full CTRL write/read (known pattern)
//   TEST 3 — Byte-enable (WSTRB) on CTRL
//   TEST 4 — STATUS read-only: write ignored, status_in reflected
//   TEST 5 — AI_LATENCY read-only: reflects ai_latency_in, write ignored
//   TEST 6 — ASCAN_RATE read/write
//
// Register map (oct_ctrl_regs):
//   0x00  CTRL        R/W  reset = 32'h00000041
//   0x04  STATUS      RO   {30'b0, status_in[1:0]}
//   0x08  ASCAN_RATE  R/W  reset = 0
//   0x0C  AI_LATENCY  RO   = ai_latency_in
//
// CTRL bit layout (oct_ctrl_t [9:0]):
//   [9:8] mode_sel   [7:6] fft_size   [5:3] ai_task
//   [2]   bypass_filt  [1] bypass_disp  [0] bypass_npu
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

`timescale 1ns/1ps

module tb_oct_ctrl_regs;

    // -----------------------------------------------------------------------
    // Clock / reset
    // -----------------------------------------------------------------------
    localparam real CLK_PERIOD = 10.0; // 100 MHz

    logic        clk;
    logic        aresetn;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -----------------------------------------------------------------------
    // AXI4-Lite signals
    // -----------------------------------------------------------------------
    logic [5:0]  awaddr;
    logic        awvalid;
    logic        awready;

    logic [31:0] wdata;
    logic [3:0]  wstrb;
    logic        wvalid;
    logic        wready;

    logic [1:0]  bresp;
    logic        bvalid;
    logic        bready;

    logic [5:0]  araddr;
    logic        arvalid;
    logic        arready;

    logic [31:0] rdata;
    logic [1:0]  rresp;
    logic        rvalid;
    logic        rready;

    // -----------------------------------------------------------------------
    // PL sideband
    // -----------------------------------------------------------------------
    oct_ctrl_t   ctrl_out;
    logic [1:0]  status_in;
    logic [31:0] ai_latency_in;

    // -----------------------------------------------------------------------
    // DUT instantiation
    // -----------------------------------------------------------------------
    oct_ctrl_regs dut (
        .S_AXI_ACLK    (clk),
        .S_AXI_ARESETN (aresetn),
        .S_AXI_AWADDR  (awaddr),
        .S_AXI_AWVALID (awvalid),
        .S_AXI_AWREADY (awready),
        .S_AXI_WDATA   (wdata),
        .S_AXI_WSTRB   (wstrb),
        .S_AXI_WVALID  (wvalid),
        .S_AXI_WREADY  (wready),
        .S_AXI_BRESP   (bresp),
        .S_AXI_BVALID  (bvalid),
        .S_AXI_BREADY  (bready),
        .S_AXI_ARADDR  (araddr),
        .S_AXI_ARVALID (arvalid),
        .S_AXI_ARREADY (arready),
        .S_AXI_RDATA   (rdata),
        .S_AXI_RRESP   (rresp),
        .S_AXI_RVALID  (rvalid),
        .S_AXI_RREADY  (rready),
        .ctrl_out       (ctrl_out),
        .status_in      (status_in),
        .ai_latency_in  (ai_latency_in)
    );

    // -----------------------------------------------------------------------
    // Waveform dump
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("dumpfile.fst");
        $dumpvars(0);
    end

    // -----------------------------------------------------------------------
    // Test tracking
    // -----------------------------------------------------------------------
    int  fail_count;
    string current_test;

    // -----------------------------------------------------------------------
    // Task: axi_write
    //   Drives AW + W channels simultaneously, waits for BVALID, checks BRESP.
    // -----------------------------------------------------------------------
    task automatic axi_write(
        input logic [5:0]  addr,
        input logic [31:0] data,
        input logic [3:0]  strb
    );
        // Present address and data together (both channels simultaneously)
        @(posedge clk); #1;
        awaddr  = addr;
        awvalid = 1'b1;
        wdata   = data;
        wstrb   = strb;
        wvalid  = 1'b1;
        bready  = 1'b1;

        // Wait for AW handshake
        fork
            begin : aw_wait
                while (!(awvalid && awready)) @(posedge clk);
                #1; awvalid = 1'b0;
            end
            begin : w_wait
                while (!(wvalid && wready)) @(posedge clk);
                #1; wvalid = 1'b0;
            end
        join

        // Wait for write response
        while (!bvalid) @(posedge clk);

        // Check BRESP
        if (bresp !== 2'b00) begin
            $display("LOG: %0t : ERROR : tb_oct_ctrl_regs : dut.S_AXI_BRESP : expected_value: 2'b00 actual_value: %0b", $time, bresp);
            fail_count++;
        end

        @(posedge clk); #1;
        bready = 1'b0;

    endtask

    // -----------------------------------------------------------------------
    // Task: axi_read
    //   Drives AR channel, waits for RVALID, captures RDATA, checks RRESP.
    // -----------------------------------------------------------------------
    task automatic axi_read(
        input  logic [5:0]  addr,
        output logic [31:0] rd_data
    );
        @(posedge clk); #1;
        araddr  = addr;
        arvalid = 1'b1;
        rready  = 1'b1;

        // Wait for AR handshake
        while (!(arvalid && arready)) @(posedge clk);
        #1; arvalid = 1'b0;

        // Wait for read response
        while (!rvalid) @(posedge clk);

        rd_data = rdata;

        // Check RRESP
        if (rresp !== 2'b00) begin
            $display("LOG: %0t : ERROR : tb_oct_ctrl_regs : dut.S_AXI_RRESP : expected_value: 2'b00 actual_value: %0b", $time, rresp);
            fail_count++;
        end

        @(posedge clk); #1;
        rready = 1'b0;

    endtask

    // -----------------------------------------------------------------------
    // Task: check_eq — compare expected vs actual, log and count failures
    // -----------------------------------------------------------------------
    task automatic check_eq(
        input string  sig_name,
        input logic [31:0] expected,
        input logic [31:0] actual
    );
        if (expected !== actual) begin
            $display("LOG: %0t : ERROR : tb_oct_ctrl_regs : %s : expected_value: 32'h%08X actual_value: 32'h%08X",
                     $time, sig_name, expected, actual);
            fail_count++;
        end else begin
            $display("LOG: %0t : INFO : tb_oct_ctrl_regs : %s : expected_value: 32'h%08X actual_value: 32'h%08X",
                     $time, sig_name, expected, actual);
        end
    endtask

    // -----------------------------------------------------------------------
    // Main test sequence
    // -----------------------------------------------------------------------
    logic [31:0] rd_val;

    initial begin
        $display("TEST START");

        // Initialise all AXI master signals to idle
        fail_count   = 0;
        aresetn      = 1'b0;
        awaddr       = '0;
        awvalid      = 1'b0;
        wdata        = '0;
        wstrb        = 4'hF;
        wvalid       = 1'b0;
        bready       = 1'b0;
        araddr       = '0;
        arvalid      = 1'b0;
        rready       = 1'b0;
        status_in    = 2'b00;
        ai_latency_in = 32'h0000_0000;

        // Assert reset for 5 clock cycles then release
        repeat (5) @(posedge clk);
        #1; aresetn = 1'b1;
        repeat (2) @(posedge clk);

        // =================================================================
        // TEST 1 — Reset values
        // =================================================================
        current_test = "TEST1_RESET_VALUES";
        $display("LOG: %0t : INFO : tb_oct_ctrl_regs : starting %s", $time, current_test);

        // Read CTRL (0x00): expect 32'h00000041 (fft_size=1024, bypass_npu=1)
        axi_read(6'h00, rd_val);
        check_eq("dut.ctrl_reg[reset]", 32'h0000_0041, rd_val);

        // Read STATUS (0x04): status_in==0 → expect 32'h00000000
        axi_read(6'h04, rd_val);
        check_eq("dut.STATUS[reset]", 32'h0000_0000, rd_val);

        // =================================================================
        // TEST 2 — Full CTRL write/read
        // Pattern: mode_sel=SS-OCT(01), fft_size=1024(01), ai_task=denoise(001),
        //          bypass_filt=0, bypass_disp=0, bypass_npu=0
        //   oct_ctrl_t[9:0] = {2'b01, 2'b01, 3'b001, 1'b0, 1'b0, 1'b0}
        //                   = 10'b01_01_001_000 = 10'h148
        //   32-bit word    = 32'h0000_0148
        // =================================================================
        current_test = "TEST2_CTRL_WRITE_READ";
        $display("LOG: %0t : INFO : tb_oct_ctrl_regs : starting %s", $time, current_test);

        axi_write(6'h00, 32'h0000_0148, 4'hF);
        axi_read (6'h00, rd_val);
        // DUT masks upper 22 bits to 0 and byte1 to [9:8] (2 bits only)
        check_eq("dut.ctrl_reg[pattern]", 32'h0000_0148, rd_val);

        // =================================================================
        // TEST 3 — Byte-enable (WSTRB) on CTRL
        //   Step A: write 0xFF to byte 0 only (WSTRB=4'b0001)
        //   Step B: write 0xAB to byte 1 only (WSTRB=4'b0010)
        //   Expected readback: byte0=0xFF, byte1=masked to 2 LSBs → 0x03 & 0xAB
        //     oct_ctrl_t byte1 only carries bits[9:8] → masked to 8'h03
        //     0xAB & 0x03 = 0x03
        //   Full 32-bit: 32'h0000_03FF
        // =================================================================
        current_test = "TEST3_BYTE_ENABLE";
        $display("LOG: %0t : INFO : tb_oct_ctrl_regs : starting %s", $time, current_test);

        // First reset CTRL to known 0
        axi_write(6'h00, 32'h0000_0000, 4'hF);

        // Write byte 0 only: 0xFF
        axi_write(6'h00, 32'h0000_00FF, 4'b0001);

        // Write byte 1 only: 0xAB (DUT masks to 0x03 because only bits[9:8] are valid)
        axi_write(6'h00, 32'h0000_AB00, 4'b0010);

        axi_read(6'h00, rd_val);
        // byte0 = 0xFF, byte1 = 0xAB & 0x03 = 0x03, bytes 2-3 = 0x00
        check_eq("dut.ctrl_reg[byte_en]", 32'h0000_03FF, rd_val);

        // Spot-check individual byte fields
        if (rd_val[7:0] !== 8'hFF) begin
            $display("LOG: %0t : ERROR : tb_oct_ctrl_regs : dut.ctrl_reg[7:0] : expected_value: 8'hFF actual_value: 8'h%02X", $time, rd_val[7:0]);
            fail_count++;
        end
        if (rd_val[15:8] !== 8'h03) begin
            $display("LOG: %0t : ERROR : tb_oct_ctrl_regs : dut.ctrl_reg[15:8] : expected_value: 8'h03 actual_value: 8'h%02X", $time, rd_val[15:8]);
            fail_count++;
        end
        if (rd_val[31:16] !== 16'h0000) begin
            $display("LOG: %0t : ERROR : tb_oct_ctrl_regs : dut.ctrl_reg[31:16] : expected_value: 16'h0000 actual_value: 16'h%04X", $time, rd_val[31:16]);
            fail_count++;
        end

        // =================================================================
        // TEST 4 — STATUS read-only
        //   4a: Write 0xFFFFFFFF to STATUS; read back; assert unchanged (== 0)
        //   4b: Drive status_in=2'b11; read back; assert bits[1:0]==2'b11
        // =================================================================
        current_test = "TEST4_STATUS_READONLY";
        $display("LOG: %0t : INFO : tb_oct_ctrl_regs : starting %s", $time, current_test);

        // 4a: Write attempt — should be silently discarded
        axi_write(6'h04, 32'hFFFF_FFFF, 4'hF);
        axi_read (6'h04, rd_val);
        // status_in is still 2'b00, so expected readback = 32'h00000000
        check_eq("dut.STATUS[write_ignored]", 32'h0000_0000, rd_val);

        // 4b: Drive status_in = 2'b11
        #1; status_in = 2'b11;
        repeat (2) @(posedge clk);
        axi_read(6'h04, rd_val);
        check_eq("dut.STATUS[status_in_11]", 32'h0000_0003, rd_val);

        // Restore status_in
        #1; status_in = 2'b00;

        // =================================================================
        // TEST 5 — AI_LATENCY read-only
        //   5a: Drive ai_latency_in = 32'hDEAD_BEEF; read; assert match
        //   5b: Attempt write to 0x0C; read back; assert unchanged
        // =================================================================
        current_test = "TEST5_AI_LATENCY_READONLY";
        $display("LOG: %0t : INFO : tb_oct_ctrl_regs : starting %s", $time, current_test);

        // 5a: Drive ai_latency_in
        #1; ai_latency_in = 32'hDEAD_BEEF;
        repeat (2) @(posedge clk);
        axi_read(6'h0C, rd_val);
        check_eq("dut.AI_LATENCY[read]", 32'hDEAD_BEEF, rd_val);

        // 5b: Write attempt — should be silently discarded
        axi_write(6'h0C, 32'h1234_5678, 4'hF);
        axi_read (6'h0C, rd_val);
        // ai_latency_in is still 0xDEADBEEF (combinational passthrough)
        check_eq("dut.AI_LATENCY[write_ignored]", 32'hDEAD_BEEF, rd_val);

        // =================================================================
        // TEST 6 — ASCAN_RATE read/write
        // =================================================================
        current_test = "TEST6_ASCAN_RATE";
        $display("LOG: %0t : INFO : tb_oct_ctrl_regs : starting %s", $time, current_test);

        axi_write(6'h08, 32'd100000, 4'hF);
        axi_read (6'h08, rd_val);
        check_eq("dut.ascan_rate_reg[100000]", 32'd100000, rd_val);

        // =================================================================
        // Final result
        // =================================================================
        repeat (4) @(posedge clk);

        if (fail_count == 0) begin
            $display("TB_CTRL_REGS: PASS");
            $display("TEST PASSED");
        end else begin
            $display("ERROR");
            $display("TB_CTRL_REGS: FAIL — %0d check(s) failed, last test: %s", fail_count, current_test);
            $display("TEST FAILED");
            $error("TB_CTRL_REGS: %0d failure(s)", fail_count);
        end

        $finish;
    end

    // -----------------------------------------------------------------------
    // Simulation timeout watchdog (500 cycles)
    // -----------------------------------------------------------------------
    initial begin
        repeat (500) @(posedge clk);
        $display("ERROR");
        $display("TEST FAILED");
        $fatal(1, "TIMEOUT: simulation exceeded 500 clock cycles");
    end

endmodule : tb_oct_ctrl_regs
