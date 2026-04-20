`timescale 1ns/1ps
import oct_pkg::*;

//------------------------------------------------------------------------------
// bscan_buffer
//------------------------------------------------------------------------------
// Double-buffered (ping-pong) 1 MB B-scan frame buffer.
//
// Purpose:
//   - DSP writes B-scan N+1 in dsp_clk domain
//   - NPU reads/writes B-scan N in npu_clk domain
//   - Host reads the currently published frame in dsp_clk domain
//
// Geometry:
//   - 1024 x 1024 x 8-bit per bank = 1,048,576 bits = 1 Mbit = 128 KiB
//   - Two banks total = 2 Mbit = 256 KiB
//
// Notes:
//   1) The prompt references "swap only after npu_frame_done is acknowledged",
//      but no explicit npu_frame_done / npu_done_ack input is present.
//      This implementation derives NPU completion when the NPU reads the final
//      pixel address of the current frame:
//          npu_re && (npu_raddr == LAST_ADDR)
//   2) bank_sel is 1-bit, so Gray coding is identical to binary. The CDC is
//      still implemented as a 2-FF synchronizer on that Gray-coded bit.
//   3) The memories are modeled as inferred block RAM arrays. Exact BRAM36
//      tiling is left to synthesis/Pblock floorplanning.
//   4) Read-during-write behavior to the same address is device/tool specific.
//      System-level arbitration should avoid same-address collisions.
//
// Swap FSM in dsp_clk domain:
//   IDLE -> WAIT_NPU_DONE -> SWAP
//
// Bank ownership:
//   dsp_wr_bank  : bank DSP currently fills
//   pub_rd_bank  : bank published to NPU/Host
//------------------------------------------------------------------------------
module bscan_buffer #(
  parameter int BSCAN_W = 1024,
  parameter int BSCAN_H = 1024,
  parameter int PIX_W   = 8
)(
  // DSP write port (250 MHz domain)
  input  logic              dsp_clk,
  input  logic              dsp_rst_n,
  input  logic [PIX_W-1:0]  dsp_tdata,
  input  logic              dsp_tvalid,
  input  logic              dsp_tlast,
  input  logic [31:0]       dsp_tuser,
  output logic              dsp_tready,
  output logic              dsp_frame_done,

  // NPU read port (300 MHz domain)
  input  logic              npu_clk,
  input  logic              npu_rst_n,
  output logic [PIX_W-1:0]  npu_rdata,
  input  logic [19:0]       npu_raddr,
  input  logic              npu_re,

  // NPU write-back port
  input  logic [PIX_W-1:0]  npu_wdata,
  input  logic [19:0]       npu_waddr,
  input  logic              npu_we,
  output logic              npu_frame_ready,

  // Host DMA read (250 MHz domain)
  input  logic [19:0]       host_raddr,
  output logic [PIX_W-1:0]  host_rdata,
  input  logic              host_re
);

  localparam int DEPTH  = BSCAN_W * BSCAN_H;
  localparam int ADDR_W = $clog2(DEPTH);

  localparam logic [ADDR_W-1:0] LAST_ADDR = DEPTH - 1;

  typedef enum logic [1:0] {
    ST_IDLE          = 2'd0,
    ST_WAIT_NPU_DONE = 2'd1,
    ST_SWAP          = 2'd2
  } swap_state_e;

  //--------------------------------------------------------------------------
  // Frame memories
  //--------------------------------------------------------------------------
  (* ram_style = "block" *) logic [PIX_W-1:0] bank0_mem [0:DEPTH-1];
  (* ram_style = "block" *) logic [PIX_W-1:0] bank1_mem [0:DEPTH-1];

  //--------------------------------------------------------------------------
  // DSP-domain write-side raster counters
  //--------------------------------------------------------------------------
  logic [9:0] dsp_col_cnt;
  logic [9:0] dsp_row_cnt;
  logic [ADDR_W-1:0] dsp_waddr;
  logic dsp_accept;

  assign dsp_tready = 1'b1;
  assign dsp_accept = dsp_tvalid & dsp_tready;

  // row*1024 + col for BSCAN_W=1024. Since BSCAN_W is parameterized but fixed
  // by the stated architecture, keep the shift-based addressing for timing.
  always_comb begin
    dsp_waddr = {dsp_row_cnt, dsp_col_cnt};
  end

  //--------------------------------------------------------------------------
  // Bank control in DSP domain
  //--------------------------------------------------------------------------
  logic        dsp_wr_bank_bin;      // bank currently written by DSP
  logic        pub_rd_bank_bin_dsp;  // bank currently published to NPU/Host
  logic        pub_rd_bank_gray_dsp; // Gray-coded published bank (1-bit)

  swap_state_e swap_state;
  logic        swap_req_pending;

  //--------------------------------------------------------------------------
  // NPU done event crossing: npu_clk -> dsp_clk
  //--------------------------------------------------------------------------
  logic npu_done_tog_npu;
  logic npu_done_tog_dsp_ff1, npu_done_tog_dsp_ff2;
  logic npu_done_pulse_dsp;

  //--------------------------------------------------------------------------
  // Frame-ready event crossing: dsp_clk -> npu_clk
  //--------------------------------------------------------------------------
  logic frame_ready_tog_dsp;
  logic frame_ready_tog_npu_ff1, frame_ready_tog_npu_ff2;
  logic npu_frame_ready_pulse_npu;

  //--------------------------------------------------------------------------
  // Published bank crossing: dsp_clk -> npu_clk
  //--------------------------------------------------------------------------
  logic pub_rd_bank_gray_npu_ff1, pub_rd_bank_gray_npu_ff2;
  logic pub_rd_bank_bin_npu;

  //--------------------------------------------------------------------------
  // DSP write path
  //--------------------------------------------------------------------------
  always_ff @(posedge dsp_clk or negedge dsp_rst_n) begin
    if (!dsp_rst_n) begin
      dsp_col_cnt      <= '0;
      dsp_row_cnt      <= '0;
      dsp_frame_done   <= 1'b0;
      swap_req_pending <= 1'b0;
    end else begin
      dsp_frame_done <= 1'b0;

      if (dsp_accept) begin
        // Write pixel into active DSP bank
        if (dsp_wr_bank_bin == 1'b0)
          bank0_mem[dsp_waddr] <= dsp_tdata;
        else
          bank1_mem[dsp_waddr] <= dsp_tdata;

        // End-of-frame detection:
        // frame ends on final pixel of final row, coincident with tlast
        if ((dsp_col_cnt == BSCAN_W-1) &&
            (dsp_row_cnt == BSCAN_H-1) &&
            dsp_tlast) begin
          dsp_col_cnt      <= '0;
          dsp_row_cnt      <= '0;
          dsp_frame_done   <= 1'b1;
          swap_req_pending <= 1'b1;
        end
        else if (dsp_tlast) begin
          dsp_col_cnt <= '0;
          dsp_row_cnt <= dsp_row_cnt + 10'd1;
        end
        else begin
          dsp_col_cnt <= dsp_col_cnt + 10'd1;
        end
      end
    end
  end

  //--------------------------------------------------------------------------
  // NPU derived frame-done event
  //--------------------------------------------------------------------------
  always_ff @(posedge npu_clk or negedge npu_rst_n) begin
    if (!npu_rst_n) begin
      npu_done_tog_npu <= 1'b0;
    end else if (npu_re && (npu_raddr == LAST_ADDR)) begin
      npu_done_tog_npu <= ~npu_done_tog_npu;
    end
  end

  always_ff @(posedge dsp_clk or negedge dsp_rst_n) begin
    if (!dsp_rst_n) begin
      npu_done_tog_dsp_ff1 <= 1'b0;
      npu_done_tog_dsp_ff2 <= 1'b0;
    end else begin
      npu_done_tog_dsp_ff1 <= npu_done_tog_npu;
      npu_done_tog_dsp_ff2 <= npu_done_tog_dsp_ff1;
    end
  end

  assign npu_done_pulse_dsp = npu_done_tog_dsp_ff1 ^ npu_done_tog_dsp_ff2;

  //--------------------------------------------------------------------------
  // Swap FSM
  //--------------------------------------------------------------------------
  always_ff @(posedge dsp_clk or negedge dsp_rst_n) begin
    if (!dsp_rst_n) begin
      swap_state         <= ST_IDLE;
      dsp_wr_bank_bin    <= 1'b0;  // DSP starts filling bank 0
      pub_rd_bank_bin_dsp  <= 1'b1;  // NPU/Host initially point to bank 1
      pub_rd_bank_gray_dsp <= 1'b1;  // 1-bit Gray == binary
      frame_ready_tog_dsp <= 1'b0;
    end else begin
      case (swap_state)
        ST_IDLE: begin
          if (swap_req_pending)
            swap_state <= ST_WAIT_NPU_DONE;
        end

        ST_WAIT_NPU_DONE: begin
          if (npu_done_pulse_dsp)
            swap_state <= ST_SWAP;
        end

        ST_SWAP: begin
          // Publish the just-written bank to NPU/Host
          pub_rd_bank_bin_dsp  <= dsp_wr_bank_bin;
          pub_rd_bank_gray_dsp <= dsp_wr_bank_bin;

          // DSP moves to the other bank for next frame
          dsp_wr_bank_bin      <= ~dsp_wr_bank_bin;

          // Notify NPU of bank swap
          frame_ready_tog_dsp  <= ~frame_ready_tog_dsp;

          swap_req_pending     <= 1'b0;
          swap_state           <= ST_IDLE;
        end

        default: begin
          swap_state <= ST_IDLE;
        end
      endcase
    end
  end

  //--------------------------------------------------------------------------
  // Published bank CDC into NPU clock domain
  //--------------------------------------------------------------------------
  always_ff @(posedge npu_clk or negedge npu_rst_n) begin
    if (!npu_rst_n) begin
      pub_rd_bank_gray_npu_ff1 <= 1'b0;
      pub_rd_bank_gray_npu_ff2 <= 1'b0;
    end else begin
      pub_rd_bank_gray_npu_ff1 <= pub_rd_bank_gray_dsp;
      pub_rd_bank_gray_npu_ff2 <= pub_rd_bank_gray_npu_ff1;
    end
  end

  assign pub_rd_bank_bin_npu = pub_rd_bank_gray_npu_ff2; // 1-bit Gray decode

  //--------------------------------------------------------------------------
  // Frame-ready pulse in NPU domain
  //--------------------------------------------------------------------------
  always_ff @(posedge npu_clk or negedge npu_rst_n) begin
    if (!npu_rst_n) begin
      frame_ready_tog_npu_ff1 <= 1'b0;
      frame_ready_tog_npu_ff2 <= 1'b0;
    end else begin
      frame_ready_tog_npu_ff1 <= frame_ready_tog_dsp;
      frame_ready_tog_npu_ff2 <= frame_ready_tog_npu_ff1;
    end
  end

  assign npu_frame_ready_pulse_npu = frame_ready_tog_npu_ff1 ^ frame_ready_tog_npu_ff2;

  always_ff @(posedge npu_clk or negedge npu_rst_n) begin
    if (!npu_rst_n)
      npu_frame_ready <= 1'b0;
    else
      npu_frame_ready <= npu_frame_ready_pulse_npu;
  end

  //--------------------------------------------------------------------------
  // NPU read / write-back path
  //
  // NPU accesses the currently published bank.
  //--------------------------------------------------------------------------
  logic [PIX_W-1:0] npu_rdata_q;

  always_ff @(posedge npu_clk or negedge npu_rst_n) begin
    if (!npu_rst_n) begin
      npu_rdata_q <= '0;
    end else begin
      if (npu_we) begin
        if (pub_rd_bank_bin_npu == 1'b0)
          bank0_mem[npu_waddr] <= npu_wdata;
        else
          bank1_mem[npu_waddr] <= npu_wdata;
      end

      if (npu_re) begin
        if (pub_rd_bank_bin_npu == 1'b0)
          npu_rdata_q <= bank0_mem[npu_raddr];
        else
          npu_rdata_q <= bank1_mem[npu_raddr];
      end
    end
  end

  assign npu_rdata = npu_rdata_q;

  //--------------------------------------------------------------------------
  // Host read path
  //
  // Host reads the currently published bank in dsp_clk domain.
  //--------------------------------------------------------------------------
  logic [PIX_W-1:0] host_rdata_q;

  always_ff @(posedge dsp_clk or negedge dsp_rst_n) begin
    if (!dsp_rst_n) begin
      host_rdata_q <= '0;
    end else if (host_re) begin
      if (pub_rd_bank_bin_dsp == 1'b0)
        host_rdata_q <= bank0_mem[host_raddr];
      else
        host_rdata_q <= bank1_mem[host_raddr];
    end
  end

  assign host_rdata = host_rdata_q;

endmodule