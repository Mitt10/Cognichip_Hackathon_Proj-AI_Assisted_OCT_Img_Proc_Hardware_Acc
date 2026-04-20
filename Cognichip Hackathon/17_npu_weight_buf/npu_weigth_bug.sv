`timescale 1ns/1ps
import oct_pkg::*;

//------------------------------------------------------------------------------
// npu_weight_buf
//------------------------------------------------------------------------------
// UltraRAM-backed NPU weight buffer.
//
// Function:
//   - DMA writes 64-bit weight words from ARM PS
//   - PE-side reads 64-bit weight words with 2-cycle read latency
//   - One logical buffer is striped across N_URAM banks
//
// Addressing:
//   - Global address space is 256K x 64-bit = 2 MiB
//   - Address decode:
//       bank_idx  = addr[17 -: BANK_SEL_W]
//       bank_addr = addr[BANK_ADDR_W-1:0]
//
// Implementation notes:
//   1) This version uses explicit per-bank storage arrays with
//        (* ram_style = "ultra" *)
//      so Vivado maps them to UltraRAM.
//   2) The prompt mentions "URAM288" / "256Kx72 per URAM", but a single
//      UltraRAM block does not directly implement the full 256Kx72 depth.
//      This module instead builds one logical 256Kx64 bank by striping
//      the address space across N_URAM independently addressable URAM-backed
//      banks. This is the practical synthesizable organization.
//   3) N_URAM should be a power of 2 for the simple bank decode below.
//   4) rd_valid pulses 2 cycles after rd_en.
//   5) load_done pulses when dma_we deasserts after one or more write cycles.
//
// Cascading / scaling:
//   - Increase N_URAM up to 48 as device capacity allows.
//   - The generate loop below shows how banks are replicated.
//------------------------------------------------------------------------------

module npu_weight_buf #(
  parameter int N_URAM = 4   // configurable up to 48; power-of-2 recommended
)(
  input  logic         clk,
  input  logic         rst_n,

  // DMA write path
  input  logic [17:0]  dma_waddr,   // 256K locations
  input  logic [63:0]  dma_wdata,   // 8 INT8 weights
  input  logic         dma_we,

  // Weight read path
  input  logic [17:0]  rd_addr,
  output logic [63:0]  rd_data,
  input  logic         rd_en,
  output logic         rd_valid,    // valid 2 cycles after rd_en

  // Status
  output logic         load_done
);

  localparam int ADDR_W      = 18;
  localparam int DATA_W      = 64;
  localparam int TOTAL_WORDS = (1 << ADDR_W);            // 256K words
  localparam int BANK_SEL_W  = (N_URAM <= 1) ? 1 : $clog2(N_URAM);
  localparam int BANK_ADDR_W = ADDR_W - BANK_SEL_W;
  localparam int BANK_DEPTH  = (1 << BANK_ADDR_W);

  //--------------------------------------------------------------------------
  // Static sanity note:
  // This decode assumes N_URAM is a power of 2.
  //--------------------------------------------------------------------------

  // DMA decode
  logic [BANK_SEL_W-1:0] dma_bank_sel;
  logic [BANK_ADDR_W-1:0] dma_bank_addr;

  // Read decode
  logic [BANK_SEL_W-1:0] rd_bank_sel_s0;
  logic [BANK_ADDR_W-1:0] rd_bank_addr_s0;

  assign dma_bank_sel  = dma_waddr[ADDR_W-1 -: BANK_SEL_W];
  assign dma_bank_addr = dma_waddr[BANK_ADDR_W-1:0];

  assign rd_bank_sel_s0  = rd_addr[ADDR_W-1 -: BANK_SEL_W];
  assign rd_bank_addr_s0 = rd_addr[BANK_ADDR_W-1:0];

  //--------------------------------------------------------------------------
  // Per-bank signals
  //--------------------------------------------------------------------------
  logic [DATA_W-1:0] bank_rdata [0:N_URAM-1];
  logic [N_URAM-1:0] bank_dma_we;
  logic [N_URAM-1:0] bank_rd_en;

  logic [BANK_ADDR_W-1:0] bank_dma_addr;
  logic [BANK_ADDR_W-1:0] bank_rd_addr;

  // Read pipeline bookkeeping
  logic [BANK_SEL_W-1:0] rd_bank_sel_s1;
  logic [BANK_SEL_W-1:0] rd_bank_sel_s2;
  logic                  rd_valid_s1;
  logic                  rd_valid_s2;
  logic [DATA_W-1:0]     rd_data_s1;
  logic [DATA_W-1:0]     rd_data_s2;

  // load_done generation
  logic dma_we_d;

  assign bank_dma_addr = dma_bank_addr;
  assign bank_rd_addr  = rd_bank_addr_s0;

  genvar g;
  generate
    for (g = 0; g < N_URAM; g = g + 1) begin : G_URAM_BANKS
      assign bank_dma_we[g] = dma_we && (dma_bank_sel == g[BANK_SEL_W-1:0]);
      assign bank_rd_en[g]  = rd_en  && (rd_bank_sel_s0 == g[BANK_SEL_W-1:0]);

      npu_weight_buf_uram_bank #(
        .ADDR_W (BANK_ADDR_W),
        .DATA_W (DATA_W)
      ) u_bank (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_addr  (bank_dma_addr),
        .wr_data  (dma_wdata),
        .wr_en    (bank_dma_we[g]),
        .rd_addr  (bank_rd_addr),
        .rd_en    (bank_rd_en[g]),
        .rd_data  (bank_rdata[g])
      );
    end
  endgenerate

  //--------------------------------------------------------------------------
  // Read latency pipeline: total 2 cycles after rd_en
  //
  // Cycle 0: capture rd_en / bank select / bank addr
  // Cycle 1: selected bank produces bank_rdata[*]
  // Cycle 2: register final output + rd_valid
  //--------------------------------------------------------------------------

  // Stage 1 bookkeeping
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_bank_sel_s1 <= '0;
      rd_valid_s1    <= 1'b0;
    end else begin
      rd_bank_sel_s1 <= rd_bank_sel_s0;
      rd_valid_s1    <= rd_en;
    end
  end

  // Stage 1 selected data mux
  always_comb begin
    rd_data_s1 = '0;
    rd_data_s1 = bank_rdata[rd_bank_sel_s1];
  end

  // Stage 2 output register
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_bank_sel_s2 <= '0;
      rd_valid_s2    <= 1'b0;
      rd_data_s2     <= '0;
    end else begin
      rd_bank_sel_s2 <= rd_bank_sel_s1;
      rd_valid_s2    <= rd_valid_s1;
      rd_data_s2     <= rd_data_s1;
    end
  end

  assign rd_data  = rd_data_s2;
  assign rd_valid = rd_valid_s2;

  //--------------------------------------------------------------------------
  // load_done pulse
  // Pulses for 1 cycle when a DMA write burst ends (dma_we falls).
  //--------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dma_we_d  <= 1'b0;
      load_done <= 1'b0;
    end else begin
      dma_we_d  <= dma_we;
      load_done <= dma_we_d && !dma_we;
    end
  end

endmodule


//------------------------------------------------------------------------------
// Per-bank UltraRAM-backed storage
//------------------------------------------------------------------------------
// Synthesizable URAM bank.
// - 1-cycle synchronous read from the selected bank
// - 1-cycle write
// - Vivado should map this to UltraRAM because of ram_style="ultra"
//
// For direct primitive instantiation, replace this module body with URAM288_*
// primitives and preserve the same interface.
//------------------------------------------------------------------------------
module npu_weight_buf_uram_bank #(
  parameter int ADDR_W = 16,
  parameter int DATA_W = 64
)(
  input  logic               clk,
  input  logic               rst_n,
  input  logic [ADDR_W-1:0]  wr_addr,
  input  logic [DATA_W-1:0]  wr_data,
  input  logic               wr_en,
  input  logic [ADDR_W-1:0]  rd_addr,
  input  logic               rd_en,
  output logic [DATA_W-1:0]  rd_data
);

  (* ram_style = "ultra" *) logic [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];
  logic [DATA_W-1:0] rd_data_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_data_q <= '0;
    end else begin
      if (wr_en)
        mem[wr_addr] <= wr_data;

      if (rd_en)
        rd_data_q <= mem[rd_addr];
    end
  end

  assign rd_data = rd_data_q;

endmodule