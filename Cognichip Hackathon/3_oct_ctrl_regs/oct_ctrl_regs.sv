// ============================================================================
// oct_ctrl_regs.sv
// OCT AI Accelerator — AXI4-Lite Control Register File
//
// Implements a 32-bit AXI4-Lite slave with four word-addressed registers:
//   0x00  CTRL        — R/W  {22'b0, oct_ctrl_t}  (10 bits, prompt says ctrl_reg_t)
//   0x04  STATUS      — RO   {30'b0, overflow, locked} written by PL
//   0x08  ASCAN_RATE  — R/W  32-bit expected A-scan rate (Hz)
//   0x0C  AI_LATENCY  — RO   32-bit last NPU inference latency (DSP clk cycles)
//
// Note: The package defines the control struct as oct_ctrl_t (10 bits [9:0]).
//       The prompt aliases it as ctrl_reg_t; oct_ctrl_t is used here.
//
// AXI4-Lite compliance notes:
//   - Fully registered read/write paths — no combinational AXI response.
//   - BRESP / RRESP are always 2'b00 (OKAY).
//   - Write byte-enables (WSTRB[3:0]) are honoured on all R/W registers.
//   - CTRL reset: mode_sel=SD-OCT(0), fft_size=1024(1), bypass_npu=1 → 0x00000041
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module oct_ctrl_regs (
    // -----------------------------------------------------------------------
    // AXI4-Lite global signals
    // -----------------------------------------------------------------------
    input  logic        S_AXI_ACLK,
    input  logic        S_AXI_ARESETN,   // Active-low synchronous reset

    // -----------------------------------------------------------------------
    // Write address channel
    // -----------------------------------------------------------------------
    input  logic [5:0]  S_AXI_AWADDR,
    input  logic        S_AXI_AWVALID,
    output logic        S_AXI_AWREADY,

    // -----------------------------------------------------------------------
    // Write data channel
    // -----------------------------------------------------------------------
    input  logic [31:0] S_AXI_WDATA,
    input  logic [3:0]  S_AXI_WSTRB,
    input  logic        S_AXI_WVALID,
    output logic        S_AXI_WREADY,

    // -----------------------------------------------------------------------
    // Write response channel
    // -----------------------------------------------------------------------
    output logic [1:0]  S_AXI_BRESP,
    output logic        S_AXI_BVALID,
    input  logic        S_AXI_BREADY,

    // -----------------------------------------------------------------------
    // Read address channel
    // -----------------------------------------------------------------------
    input  logic [5:0]  S_AXI_ARADDR,
    input  logic        S_AXI_ARVALID,
    output logic        S_AXI_ARREADY,

    // -----------------------------------------------------------------------
    // Read data channel
    // -----------------------------------------------------------------------
    output logic [31:0] S_AXI_RDATA,
    output logic [1:0]  S_AXI_RRESP,
    output logic        S_AXI_RVALID,
    input  logic        S_AXI_RREADY,

    // -----------------------------------------------------------------------
    // PL-facing ports
    // -----------------------------------------------------------------------
    output oct_ctrl_t   ctrl_out,        // Registered CTRL value to PL fabric
    input  logic [1:0]  status_in,       // {overflow, locked} from DSP watchdog
    input  logic [31:0] ai_latency_in    // Last NPU inference latency
);

    // -----------------------------------------------------------------------
    // Local parameters — register offsets (word-addressed, 4-byte aligned)
    // -----------------------------------------------------------------------
    localparam logic [5:0] ADDR_CTRL       = 6'h00;
    localparam logic [5:0] ADDR_STATUS     = 6'h04;
    localparam logic [5:0] ADDR_ASCAN_RATE = 6'h08;
    localparam logic [5:0] ADDR_AI_LATENCY = 6'h0C;

    // Reset value for CTRL:
    //   oct_ctrl_t.mode_sel    = 2'b00  (SD-OCT)        → bits [9:8]
    //   oct_ctrl_t.fft_size    = 2'b01  (1024-pt FFT)   → bits [7:6]
    //   oct_ctrl_t.ai_task     = 3'b000 (off)            → bits [5:3]
    //   oct_ctrl_t.bypass_filt = 1'b0                    → bit  [2]
    //   oct_ctrl_t.bypass_disp = 1'b0                    → bit  [1]
    //   oct_ctrl_t.bypass_npu  = 1'b1  (NPU bypassed)   → bit  [0]
    //   → 10'b0001000001 = 32'h00000041
    localparam logic [31:0] CTRL_RESET_VAL = 32'h00000041;

    // -----------------------------------------------------------------------
    // Internal registers
    // -----------------------------------------------------------------------
    logic [31:0] ctrl_reg;       // Offset 0x00: R/W — bits[9:0] = oct_ctrl_t
    logic [31:0] ascan_rate_reg; // Offset 0x08: R/W

    // -----------------------------------------------------------------------
    // Write-channel handshake state
    // -----------------------------------------------------------------------
    // Strategy: latch AW address and W data independently;
    // perform the register write and issue BVALID once both are captured.
    // -----------------------------------------------------------------------
    logic        aw_done;        // AW address has been captured
    logic        w_done;         // W data / strobe has been captured
    logic [5:0]  aw_addr_lat;    // Latched write address
    logic [31:0] w_data_lat;     // Latched write data
    logic [3:0]  w_strb_lat;     // Latched write strobes

    // Both halves captured — trigger the actual register write
    wire  do_write = aw_done && w_done;

    // -----------------------------------------------------------------------
    // Write address channel
    //   AWREADY is driven high for exactly one cycle upon acceptance.
    //   The address is latched and aw_done is set; both clear after the write.
    // -----------------------------------------------------------------------
    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_AWREADY <= 1'b0;
            aw_done        <= 1'b0;
            aw_addr_lat    <= '0;
        end else begin
            if (S_AXI_AWVALID && S_AXI_AWREADY) begin
                // Address accepted this cycle — deassert ready, mark done
                S_AXI_AWREADY <= 1'b0;
                aw_done        <= 1'b1;
                aw_addr_lat    <= S_AXI_AWADDR;
            end else if (do_write) begin
                // Write has been executed — clear flag, re-open for next txn
                aw_done        <= 1'b0;
                S_AXI_AWREADY  <= 1'b1;
            end else if (!aw_done && !S_AXI_AWREADY) begin
                // Ready to accept a new address
                S_AXI_AWREADY  <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Write data channel
    //   WREADY is driven high for exactly one cycle upon acceptance.
    //   Data and strobes are latched; w_done is set; both clear after the write.
    // -----------------------------------------------------------------------
    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_WREADY <= 1'b0;
            w_done        <= 1'b0;
            w_data_lat    <= '0;
            w_strb_lat    <= '0;
        end else begin
            if (S_AXI_WVALID && S_AXI_WREADY) begin
                // Data accepted this cycle — deassert ready, mark done
                S_AXI_WREADY <= 1'b0;
                w_done        <= 1'b1;
                w_data_lat    <= S_AXI_WDATA;
                w_strb_lat    <= S_AXI_WSTRB;
            end else if (do_write) begin
                // Write has been executed — clear flag, re-open for next txn
                w_done        <= 1'b0;
                S_AXI_WREADY  <= 1'b1;
            end else if (!w_done && !S_AXI_WREADY) begin
                // Ready to accept new write data
                S_AXI_WREADY  <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Register write logic
    //   Executes one cycle after both AW and W have been captured.
    //   Byte-enables (w_strb_lat) are applied to R/W registers.
    //   Read-only registers (STATUS, AI_LATENCY) ignore AXI writes silently.
    // -----------------------------------------------------------------------
    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            ctrl_reg       <= CTRL_RESET_VAL;
            ascan_rate_reg <= '0;
        end else if (do_write) begin
            case (aw_addr_lat)
                ADDR_CTRL: begin
                    // Only bits [9:0] are meaningful; upper 22 bits are tied to 0
                    if (w_strb_lat[0]) ctrl_reg[ 7: 0] <= w_data_lat[ 7: 0];
                    if (w_strb_lat[1]) ctrl_reg[15: 8] <= w_data_lat[15: 8] & 8'h03;
                    // Bytes [3:2] map to bits [31:16]; upper bits always 0
                    // (mask strobe 2 and 3 to prevent stray bits being stored)
                    if (w_strb_lat[2]) ctrl_reg[23:16] <= '0;
                    if (w_strb_lat[3]) ctrl_reg[31:24] <= '0;
                end
                ADDR_ASCAN_RATE: begin
                    if (w_strb_lat[0]) ascan_rate_reg[ 7: 0] <= w_data_lat[ 7: 0];
                    if (w_strb_lat[1]) ascan_rate_reg[15: 8] <= w_data_lat[15: 8];
                    if (w_strb_lat[2]) ascan_rate_reg[23:16] <= w_data_lat[23:16];
                    if (w_strb_lat[3]) ascan_rate_reg[31:24] <= w_data_lat[31:24];
                end
                // ADDR_STATUS and ADDR_AI_LATENCY are read-only — writes are
                // silently discarded (no error response per AXI4-Lite spec).
                default: ; // Unmapped addresses silently ignored
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Write response channel
    //   BVALID asserted the cycle after both channels are captured (do_write).
    //   Cleared on BREADY acknowledgement.
    //   BRESP is hardwired to OKAY (2'b00).
    // -----------------------------------------------------------------------
    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_BVALID <= 1'b0;
        end else begin
            if (do_write) begin
                S_AXI_BVALID <= 1'b1;
            end else if (S_AXI_BVALID && S_AXI_BREADY) begin
                S_AXI_BVALID <= 1'b0;
            end
        end
    end

    assign S_AXI_BRESP = 2'b00; // OKAY

    // -----------------------------------------------------------------------
    // Read address channel
    //   ARREADY pulses high for one cycle to accept the address.
    //   Address is latched; read data is presented on the following cycle.
    //   ARREADY is blocked while a read response is in flight (RVALID=1).
    // -----------------------------------------------------------------------
    logic [5:0] ar_addr_lat;

    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_ARREADY <= 1'b0;
            ar_addr_lat    <= '0;
        end else begin
            if (S_AXI_ARVALID && S_AXI_ARREADY) begin
                // Accept address this cycle — deassert ready, latch address
                S_AXI_ARREADY <= 1'b0;
                ar_addr_lat    <= S_AXI_ARADDR;
            end else if (!S_AXI_RVALID) begin
                // Re-open once any in-flight read response has cleared
                S_AXI_ARREADY <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Read data channel
    //   RVALID asserted the cycle after ARREADY+ARVALID handshake.
    //   RDATA is registered from the register file.
    //   RRESP hardwired to OKAY (2'b00).
    // -----------------------------------------------------------------------
    logic ar_latched; // One-cycle pulse to trigger RVALID and register read

    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            ar_latched <= 1'b0;
        end else begin
            ar_latched <= S_AXI_ARVALID && S_AXI_ARREADY;
        end
    end

    always_ff @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_RVALID <= 1'b0;
            S_AXI_RDATA  <= '0;
        end else begin
            if (ar_latched) begin
                S_AXI_RVALID <= 1'b1;
                // Registered read: sample register bank on address acceptance
                case (ar_addr_lat)
                    ADDR_CTRL:       S_AXI_RDATA <= {22'b0, ctrl_reg[9:0]};
                    ADDR_STATUS:     S_AXI_RDATA <= {30'b0, status_in};
                    ADDR_ASCAN_RATE: S_AXI_RDATA <= ascan_rate_reg;
                    ADDR_AI_LATENCY: S_AXI_RDATA <= ai_latency_in;
                    default:         S_AXI_RDATA <= 32'hDEAD_BEEF; // Unmapped
                endcase
            end else if (S_AXI_RVALID && S_AXI_RREADY) begin
                S_AXI_RVALID <= 1'b0;
            end
        end
    end

    assign S_AXI_RRESP = 2'b00; // OKAY

    // -----------------------------------------------------------------------
    // PL output — combinationally driven from the registered CTRL value
    // -----------------------------------------------------------------------
    assign ctrl_out = oct_ctrl_t'(ctrl_reg[9:0]);

endmodule : oct_ctrl_regs
