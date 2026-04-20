// ============================================================================
// ss_oct_frontend.sv
// OCT AI Accelerator — SS-OCT Front-End (k-Clock ADC Interface & Galvo DAC)
//
// Target  : Xilinx Zynq UltraScale+ (XCZU7EV / ZCU104)
//
// Interfaces a 14-bit LVDS ADC at up to 500 MS/s, performs k-clock gating
// from an MZI fringe zero-crossing detector, drives galvanometer X/Y DACs
// via SPI, and produces an AXI4-Stream output format-identical to
// sd_oct_frontend (oct_tuser32_t packing, mode = 2'b01 SS-OCT).
//
// Block overview:
//   adc_clk domain : MZI edge detect, k-clock gate, sample counter,
//                    laser_trig, async-FIFO write (17-bit word)
//   async_fifo      : 17-bit x 32-deep Gray-code CDC FIFO
//                     bit[16] = tlast sideband, bits[15:0] = sample
//   sys_clk domain  : FWFT FIFO read -> AXI4-Stream, galvo accumulators,
//                     16-bit SPI transmitter (CPOL=0 CPHA=1 @ sys_clk/8)
//
// oct_tuser32_t packing (mode = SS-OCT = 2'b01):
//   [31:16] ascan_idx
//   [15: 8] galvo_x_acc[15:8]
//   [ 7: 4] galvo_y_acc[15:12]
//   [ 3: 2] 2'b01
//   [ 1: 0] 2'b00  (no error flags at source)
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module ss_oct_frontend (
    // -----------------------------------------------------------------------
    // Clocks / reset (active-low asynchronous)
    // -----------------------------------------------------------------------
    input  logic        adc_clk,
    input  logic        sys_clk,
    input  logic        rst_n,

    // -----------------------------------------------------------------------
    // LVDS ADC (deserialized by IOSERDES in top-level)
    // -----------------------------------------------------------------------
    input  logic [13:0] adc_data,
    input  logic        adc_valid,

    // -----------------------------------------------------------------------
    // MZI fringe reference — single-bit positive zero-crossing detect
    // -----------------------------------------------------------------------
    input  logic        mzi_fringe,

    // -----------------------------------------------------------------------
    // Galvo SPI output (CPOL=0, CPHA=1, 16-bit word, sys_clk/8)
    // -----------------------------------------------------------------------
    output logic        galvo_sclk,
    output logic        galvo_cs_n,
    output logic        galvo_mosi,

    // -----------------------------------------------------------------------
    // Laser sweep trigger (output to laser controller)
    // -----------------------------------------------------------------------
    output logic        laser_trig,

    // -----------------------------------------------------------------------
    // Configuration (quasi-static)
    // -----------------------------------------------------------------------
    input  logic [10:0] ascan_depth,   // k-linear samples per A-scan (512-2048)
    input  logic [15:0] sweep_period,  // sweep period in adc_clk cycles
    input  logic [15:0] galvo_x_step,  // galvo X increment per A-scan (16.0 fp)
    input  logic [15:0] galvo_y_step,  // galvo Y increment per B-scan (16.0 fp)

    // -----------------------------------------------------------------------
    // AXI4-Stream master output
    // -----------------------------------------------------------------------
    output logic [15:0] m_tdata,
    output logic        m_tvalid,
    output logic        m_tlast,
    output logic [31:0] m_tuser,
    input  logic        m_tready
);

    // =========================================================================
    // adc_clk domain — MZI fringe positive-edge detector
    //   adc_gate: one adc_clk wide pulse on each positive zero-crossing
    // =========================================================================

    logic mzi_q;

    always_ff @(posedge adc_clk or negedge rst_n) begin
        if (!rst_n) mzi_q <= 1'b0;
        else        mzi_q <= mzi_fringe;
    end

    wire adc_gate = mzi_fringe & ~mzi_q;

    // =========================================================================
    // adc_clk domain — k-clock sample counter
    //   Counts 0 .. ascan_depth-1, wraps to 0 after the last sample.
    // =========================================================================

    logic [10:0] samp_cnt;

    always_ff @(posedge adc_clk or negedge rst_n) begin
        if (!rst_n) begin
            samp_cnt <= '0;
        end else if (adc_gate && adc_valid) begin
            if (samp_cnt == ascan_depth - 11'd1)
                samp_cnt <= '0;
            else
                samp_cnt <= samp_cnt + 11'd1;
        end
    end

    // samp_last: final k-clocked sample of the A-scan
    wire samp_last = adc_gate && adc_valid &&
                     (samp_cnt == ascan_depth - 11'd1);

    // =========================================================================
    // adc_clk domain — Laser trigger
    //   One adc_clk pulse registered one cycle after samp_last.
    // =========================================================================

    always_ff @(posedge adc_clk or negedge rst_n) begin
        if (!rst_n) laser_trig <= 1'b0;
        else        laser_trig <= samp_last;
    end

    // =========================================================================
    // adc_clk domain — FIFO write path
    //   14-bit ADC sign-extended to 16-bit.
    //   FIFO word [16] = tlast, [15:0] = signed sample.
    // =========================================================================

    wire [15:0] adc_sx      = {{2{adc_data[13]}}, adc_data};
    wire        fifo_wr_full;
    wire        fifo_wr_en   = adc_gate && adc_valid && !fifo_wr_full;
    wire [16:0] fifo_wr_data = {samp_last, adc_sx};

    // =========================================================================
    // Async FIFO — adc_clk write / sys_clk read
    //
    //   DATA_WIDTH = 17  (16-bit sample + 1-bit tlast sideband)
    //   DEPTH      = 32
    //
    //   Assumed to be first-word-fall-through (FWFT): rd_data is valid when
    //   !rd_empty; rd_en is the consume strobe (matches sd_oct_frontend).
    // =========================================================================

    wire [16:0] fifo_rd_data;
    wire        fifo_rd_empty;
    wire        fifo_rd_en;

    async_fifo #(
        .DATA_WIDTH (17),
        .DEPTH      (32)
    ) u_kclk_fifo (
        .wr_clk   (adc_clk),
        .wr_rst_n (rst_n),
        .wr_en    (fifo_wr_en),
        .wr_data  (fifo_wr_data),
        .wr_full  (fifo_wr_full),
        .rd_clk   (sys_clk),
        .rd_rst_n (rst_n),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_rd_data),
        .rd_empty (fifo_rd_empty)
    );

    // =========================================================================
    // sys_clk domain — AXI4-Stream output (FWFT)
    // =========================================================================

    assign fifo_rd_en = m_tready && !fifo_rd_empty;
    assign m_tvalid   = !fifo_rd_empty;
    assign m_tdata    = fifo_rd_data[15:0];
    assign m_tlast    = fifo_rd_data[16];

    // =========================================================================
    // sys_clk domain — A-scan / B-scan index counters
    // =========================================================================

    logic [15:0] ascan_idx;

    wire tlast_fire = m_tvalid && m_tready && m_tlast;
    wire bscan_done = tlast_fire &&
                      (ascan_idx == 16'(oct_pkg::BSCAN_LINES - 1));

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            ascan_idx <= '0;
        else if (tlast_fire)
            ascan_idx <= ascan_idx + 16'd1;
    end

    // =========================================================================
    // sys_clk domain — Galvo accumulators (16.0 fixed-point)
    //   galvo_x_acc += galvo_x_step  on every A-scan (tlast_fire)
    //   galvo_y_acc += galvo_y_step  on every B-scan (bscan_done)
    // =========================================================================

    logic [15:0] galvo_x_acc;
    logic [15:0] galvo_y_acc;

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            galvo_x_acc <= '0;
            galvo_y_acc <= '0;
        end else begin
            if (tlast_fire)
                galvo_x_acc <= galvo_x_acc + galvo_x_step;
            if (bscan_done)
                galvo_y_acc <= galvo_y_acc + galvo_y_step;
        end
    end

    // =========================================================================
    // sys_clk domain — TUSER (SS-OCT mode = 2'b01)
    // =========================================================================

    assign m_tuser = {
        ascan_idx,           // [31:16]
        galvo_x_acc[15:8],   // [15: 8]
        galvo_y_acc[15:12],  // [ 7: 4]
        2'b01,               // [ 3: 2]  mode = SS-OCT
        2'b00                // [ 1: 0]  error flags
    };

    // =========================================================================
    // sys_clk domain — Galvo SPI transmitter
    //
    //   Protocol : CPOL=0, CPHA=1 (SPI Mode 1)
    //              SCLK idles low; master drives MOSI on SCLK rising edge;
    //              slave (DAC) latches on SCLK falling edge.
    //   Bit rate  : sys_clk / 8 = 31.25 MHz (half-period = 4 sys_clk cycles)
    //   Word size : 16 bits, MSB first
    //   Sequencer : on each tlast_fire -> queue galvo_x_acc (X transfer)
    //               on each bscan_done -> also queue galvo_y_acc (Y transfer)
    //               Transfers serialised: X first, Y follows if also pending.
    //
    //   spi_cnt  : 7-bit counter 0..127 (16 bits * 8 sys_clk cycles per bit)
    //   Bit index: 15 - spi_cnt[6:3]   (MSB first)
    //   Phase    : spi_cnt[2:0]
    //     0 : SCLK rising  — drive MOSI with current bit
    //     4 : SCLK falling — DAC samples MOSI (no master action required)
    // =========================================================================

    localparam int unsigned SPI_TOTAL_TICKS = 128; // 16 bits x 8 cycles/bit

    logic [15:0] spi_word;       // word currently being shifted out
    logic [6:0]  spi_cnt;        // running tick counter 0..127
    logic        spi_run;        // 1 = transfer in progress
    logic [1:0]  spi_pending;    // [1]=X queued  [0]=Y queued

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_word    <= '0;
            spi_cnt     <= '0;
            spi_run     <= 1'b0;
            spi_pending <= 2'b00;
            galvo_sclk  <= 1'b0;
            galvo_cs_n  <= 1'b1;
            galvo_mosi  <= 1'b0;
        end else begin

            // ------------------------------------------------------------------
            // Capture pending transfer requests (set while SPI is busy)
            // ------------------------------------------------------------------
            if (spi_run) begin
                if (tlast_fire) spi_pending[1] <= 1'b1;
                if (bscan_done) spi_pending[0] <= 1'b1;
            end

            // ------------------------------------------------------------------
            // SPI idle — pick up next transfer
            // ------------------------------------------------------------------
            if (!spi_run) begin
                galvo_sclk <= 1'b0;
                galvo_cs_n <= 1'b1;

                // Prioritise X over Y; also handle same-cycle trigger
                if (spi_pending[1] || tlast_fire) begin
                    // Latch X accumulator; clear X pending
                    spi_word       <= galvo_x_acc;
                    spi_cnt        <= 7'd0;
                    spi_run        <= 1'b1;
                    spi_pending[1] <= 1'b0;
                    galvo_cs_n     <= 1'b0;
                    // If bscan_done also fires this cycle, queue Y
                    if (bscan_done) spi_pending[0] <= 1'b1;
                end else if (spi_pending[0] || bscan_done) begin
                    // Latch Y accumulator; clear Y pending
                    spi_word       <= galvo_y_acc;
                    spi_cnt        <= 7'd0;
                    spi_run        <= 1'b1;
                    spi_pending[0] <= 1'b0;
                    galvo_cs_n     <= 1'b0;
                end

            // ------------------------------------------------------------------
            // SPI active — shift out bits
            // ------------------------------------------------------------------
            end else begin
                galvo_cs_n <= 1'b0;
                spi_cnt    <= spi_cnt + 7'd1;

                // CPHA=1 timing:
                //   rising edge  (phase 0) — master drives MOSI
                //   falling edge (phase 4) — DAC latches MOSI (implicit)
                case (spi_cnt[2:0])
                    3'd0: begin
                        galvo_sclk <= 1'b1;
                        // Bit index: MSB first = 15 - (spi_cnt[6:3])
                        // At this point spi_cnt is still the old value before
                        // the increment above; use it directly for bit select.
                        galvo_mosi <= spi_word[4'(15) - spi_cnt[6:3]];
                    end
                    3'd4: begin
                        galvo_sclk <= 1'b0;   // falling edge — DAC samples
                    end
                    default: ;
                endcase

                // End of 16-bit transfer
                if (spi_cnt == 7'(SPI_TOTAL_TICKS - 1)) begin
                    spi_run    <= 1'b0;
                    galvo_cs_n <= 1'b1;
                    galvo_sclk <= 1'b0;
                end
            end
        end
    end

    // sweep_period is forwarded to the top level for timing; no local logic
    // required — port retained to match the specified interface.
    logic _unused_sweep_period;
    assign _unused_sweep_period = |sweep_period;

endmodule : ss_oct_frontend
