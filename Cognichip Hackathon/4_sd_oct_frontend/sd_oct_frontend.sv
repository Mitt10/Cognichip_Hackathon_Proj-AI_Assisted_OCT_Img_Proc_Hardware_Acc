// ============================================================================
// sd_oct_frontend.sv
// OCT AI Accelerator — SD-OCT Front-End Receiver / ROI Cropper
//
// Target  : Xilinx Zynq UltraScale+ (XCZU7EV / ZCU104)
//
// Receives 16-bit pixels from a CameraLink/CoaXPress line-scan camera
// (already deserialized by Xilinx IP) and produces a cropped, gain-trimmed
// AXI4-Stream matching the downstream DSP interface.
//
// Pipeline overview:
//   Stage 1 — Column counter, ROI gate, gain trim (arithmetic right-shift)
//   Stage 2 — 16-deep fall-through FIFO (distributed RAM) for backpressure
//   Stage 3 — FIFO read-side drives AXI4-Stream output
//
// FIFO word layout [48:0]:
//   [48:33] tdata  (16-bit gain-trimmed pixel)
//   [   32] tlast  (asserted on roi_end sample)
//   [31: 0] tuser  (oct_tuser32_t packed)
//
// oct_tuser32_t packing (mode = SD-OCT = 2'b00):
//   [31:16] ascan_idx — resets on fval falling edge, increments on each tlast
//   [15: 8] galvo_x
//   [ 7: 4] galvo_y   (upper 4 bits of 8-bit input; oct_tuser32_t is 4-bit)
//   [ 3: 2] mode      = 2'b00 (SD-OCT)
//   [ 1: 0] valid     = 2'b00 (no error flags at source)
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module sd_oct_frontend (
    // -----------------------------------------------------------------------
    // Clock / reset (active-low synchronous)
    // -----------------------------------------------------------------------
    input  logic        clk,        // Camera pixel clock (up to 250 MHz)
    input  logic        rst_n,

    // -----------------------------------------------------------------------
    // Camera pixel input (from Xilinx CameraLink IP / LVDS deserializer)
    // -----------------------------------------------------------------------
    input  logic [15:0] cam_pixel,  // Raw 16-bit pixel sample
    input  logic        cam_valid,  // Pixel data valid qualifier
    input  logic        cam_lval,   // Line valid — high during active pixels
    input  logic        cam_fval,   // Frame valid

    // -----------------------------------------------------------------------
    // ROI configuration (written once at boot via register interface)
    // -----------------------------------------------------------------------
    input  logic [10:0] roi_start,  // First active column index (0–2047)
    input  logic [10:0] roi_end,    // Last  active column index (0–2047)
    input  logic [3:0]  gain_shift, // Arithmetic right-shift for gain trim

    // -----------------------------------------------------------------------
    // AXI4-Stream master output
    // -----------------------------------------------------------------------
    output logic [15:0] m_tdata,
    output logic        m_tvalid,
    output logic        m_tlast,    // Asserted on the roi_end sample of each A-scan
    output logic [31:0] m_tuser,    // oct_tuser32_t packed (see above)
    input  logic        m_tready,

    // -----------------------------------------------------------------------
    // Galvo / sync sideband (from control plane)
    // -----------------------------------------------------------------------
    input  logic [7:0]  galvo_x,
    input  logic [7:0]  galvo_y
);

    // -----------------------------------------------------------------------
    // FIFO parameters
    // -----------------------------------------------------------------------
    localparam int unsigned FIFO_DEPTH  = 16;
    localparam int unsigned FIFO_AW     = 4;   // log2(FIFO_DEPTH)
    localparam int unsigned FIFO_DW     = 49;  // tdata(16) + tlast(1) + tuser(32)

    // -----------------------------------------------------------------------
    // Pipeline stage 1 — Camera signal edge detection
    // -----------------------------------------------------------------------
    logic cam_lval_q, cam_fval_q;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cam_lval_q <= 1'b0;
            cam_fval_q <= 1'b0;
        end else begin
            cam_lval_q <= cam_lval;
            cam_fval_q <= cam_fval;
        end
    end

    wire lval_rise = cam_lval  & ~cam_lval_q; // Rising  edge of line-valid
    wire fval_fall = ~cam_fval & cam_fval_q;  // Falling edge of frame-valid

    // -----------------------------------------------------------------------
    // Pipeline stage 1 — Column counter
    //   Resets to 0 on the rising edge of cam_lval (start of each line).
    //   Increments while cam_lval is high and cam_valid is asserted.
    // -----------------------------------------------------------------------
    logic [10:0] col_cnt;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            col_cnt <= '0;
        end else if (lval_rise) begin
            col_cnt <= '0;
        end else if (cam_valid && cam_lval) begin
            col_cnt <= col_cnt + 11'd1;
        end
    end

    // -----------------------------------------------------------------------
    // Pipeline stage 1 — ROI gate and gain trim
    //   in_roi  : column is within the configured region of interest
    //   px_valid: pixel should be forwarded downstream
    //   px_last : this pixel is the last sample of the A-scan (roi_end column)
    // -----------------------------------------------------------------------
    wire in_roi   = (col_cnt >= roi_start) && (col_cnt <= roi_end);
    wire px_valid = cam_valid && cam_lval && in_roi;
    wire px_last  = cam_valid && cam_lval && (col_cnt == roi_end);

    // Arithmetic right-shift: treat the raw pixel as a signed value.
    // gain_shift = 0 → no change; gain_shift = N → divide by 2^N.
    wire signed [15:0] pixel_signed  = $signed(cam_pixel);
    wire        [15:0] pixel_trimmed = 16'(pixel_signed >>> gain_shift);

    // -----------------------------------------------------------------------
    // Pipeline stage 1 — A-scan index counter
    //   Increments on every tlast (end of ROI line).
    //   Resets to 0 on the falling edge of cam_fval (end of frame).
    //   Snapshot captured at the moment px_last is asserted (same cycle as
    //   the pixel write into the FIFO) to ensure tuser is consistent.
    // -----------------------------------------------------------------------
    logic [15:0] ascan_idx;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ascan_idx <= '0;
        end else if (fval_fall) begin
            ascan_idx <= '0;
        end else if (px_last) begin
            ascan_idx <= ascan_idx + 16'd1;
        end
    end

    // -----------------------------------------------------------------------
    // Pipeline stage 1 — TUSER construction (oct_tuser32_t)
    //   mode  = 2'b00 (SD-OCT constant)
    //   valid = 2'b00 (no error flags at this stage)
    //   galvo_y is 8-bit input; oct_tuser32_t only carries 4 bits — take [7:4]
    // -----------------------------------------------------------------------
    wire [31:0] tuser_word = {
        ascan_idx,        // [31:16]  A-scan index
        galvo_x,          // [15: 8]  Galvanometer X
        galvo_y[7:4],     // [ 7: 4]  Galvanometer Y (4 MSBs, normalised)
        2'b00,            // [ 3: 2]  mode = SD-OCT
        2'b00             // [ 1: 0]  valid / error flags
    };

    // -----------------------------------------------------------------------
    // Stage 2 — 16-deep fall-through (FWFT) FIFO
    //
    //   Word layout: {pixel_trimmed[15:0], px_last, tuser_word[31:0]}
    //
    //   (* ram_style="distributed" *) directs Vivado to implement the
    //   storage array in UltraScale+ LUT RAM (LUTRAM) rather than BRAM,
    //   which is appropriate for small, shallow FIFOs and avoids BRAM
    //   output-register latency.
    //
    //   Fall-through behaviour: the read pointer advances one cycle after
    //   a read handshake (fifo_rd), and the output is taken directly from
    //   memory[rd_ptr] so the first entry is visible without an extra read.
    // -----------------------------------------------------------------------

    (* ram_style = "distributed" *)
    logic [FIFO_DW-1:0] fifo_mem [0:FIFO_DEPTH-1];

    logic [FIFO_AW-1:0] wr_ptr, rd_ptr;
    logic [FIFO_AW  :0] fifo_count; // One extra bit for full/empty detection

    wire fifo_full  = (fifo_count == FIFO_AW'(FIFO_DEPTH));
    wire fifo_empty = (fifo_count == '0);

    // Write: accept pixel when valid and FIFO not full
    wire fifo_wr = px_valid && !fifo_full;

    // Read: advance when downstream accepts (FWFT — read immediately on ready)
    wire fifo_rd = m_tready && !fifo_empty;

    // Write port
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (fifo_wr) begin
            fifo_mem[wr_ptr] <= {pixel_trimmed, px_last, tuser_word};
            wr_ptr           <= wr_ptr + 1'b1;
        end
    end

    // Read pointer and occupancy counter
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr     <= '0;
            fifo_count <= '0;
        end else begin
            case ({fifo_wr, fifo_rd})
                2'b10:   begin rd_ptr <= rd_ptr;         fifo_count <= fifo_count + 1'b1; end
                2'b01:   begin rd_ptr <= rd_ptr + 1'b1;  fifo_count <= fifo_count - 1'b1; end
                2'b11:   begin rd_ptr <= rd_ptr + 1'b1;  fifo_count <= fifo_count;        end
                default: begin rd_ptr <= rd_ptr;         fifo_count <= fifo_count;        end
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Stage 3 — AXI4-Stream output (fall-through: data sourced directly
    //           from FIFO storage, no extra output register needed)
    // -----------------------------------------------------------------------
    assign {m_tdata, m_tlast, m_tuser} = fifo_mem[rd_ptr];
    assign m_tvalid                    = !fifo_empty;

endmodule : sd_oct_frontend
