`timescale 1ns/1ps
import oct_pkg::*;

//------------------------------------------------------------------------------
// npu_scheduler
//------------------------------------------------------------------------------
// Layer-by-layer execution controller for the OCT NPU.
//
// Microcode ROM:
//   - 32 entries max by default
//   - per-task program starts at task_sel * 8 entries
//
// Layer descriptor format (128-bit packed):
//   [7:0]    layer_type
//   [19:8]   in_ch
//   [31:20]  out_ch
//   [43:32]  feat_h
//   [55:44]  feat_w
//   [63:56]  stride
//   [71:64]  act_fn
//   [79:72]  requant_shift
//   [95:80]  weight_base
//   [127:96] bias_base
//
// FSM:
//   IDLE -> LOAD_LAYER_DESC -> ISSUE_WEIGHTS -> STREAM_ACT -> WAIT_PE
//        -> POST_OP -> NEXT_LAYER -> DONE
//
// Notes / assumptions:
// 1) No explicit PE-done or postop-done handshake is present in the interface,
//    so this scheduler uses fixed internal cycle counts derived from the layer
//    descriptor to advance between phases.
// 2) A layer descriptor with all-zero fields is treated as end-of-program.
// 3) weight_rd_addr iterates from weight_base upward, one 64-bit word/cycle.
// 4) act_bank_sel toggles once per layer to support activation ping-pong.
// 5) pool_en_out is asserted only for layer_type == 4 (pool).
// 6) bias_base is decoded but not exposed because no bias-memory port exists
//    in this interface.
//
// This module is synthesizable. Debug messages are wrapped in
// synthesis translate_off guards.
//------------------------------------------------------------------------------
module npu_scheduler #(
  parameter int MAX_LAYERS   = 32,
  parameter int LAYER_DESC_W = 128
)(
  input  logic              clk,
  input  logic              rst_n,

  // Trigger
  input  logic              start,
  input  logic [2:0]        task_sel,
  output logic              busy,
  output logic              done,

  // Microcode ROM write
  input  logic              mc_we,
  input  logic [4:0]        mc_addr,
  input  logic [127:0]      mc_wdata,

  // PE array control
  output logic              weight_load,
  output logic [17:0]       weight_rd_addr,
  output logic              act_clear,
  output logic              act_valid_out,

  // Post-op control
  output logic [1:0]        act_fn_out,
  output logic [7:0]        requant_shift_out,
  output logic              pool_en_out,

  // Activation buffer control
  output logic              act_bank_sel,
  output logic [19:0]       act_rd_addr,
  output logic [19:0]       act_wr_addr,

  // Latency counter
  output logic [31:0]       inference_cycles
);

  localparam int TASK_STRIDE = 8;

  typedef enum logic [2:0] {
    ST_IDLE            = 3'd0,
    ST_LOAD_LAYER_DESC = 3'd1,
    ST_ISSUE_WEIGHTS   = 3'd2,
    ST_STREAM_ACT      = 3'd3,
    ST_WAIT_PE         = 3'd4,
    ST_POST_OP         = 3'd5,
    ST_NEXT_LAYER      = 3'd6,
    ST_DONE            = 3'd7
  } state_t;

  //--------------------------------------------------------------------------
  // Microcode storage
  //--------------------------------------------------------------------------
  logic [LAYER_DESC_W-1:0] mc_rom [0:MAX_LAYERS-1];

  //--------------------------------------------------------------------------
  // State / control
  //--------------------------------------------------------------------------
  state_t state;

  logic [4:0] prog_base;
  logic [4:0] layer_idx;
  logic [4:0] curr_mc_addr;

  logic [127:0] layer_desc_reg;
  logic         layer_valid;

  // decoded descriptor fields
  logic [7:0]   layer_type_d;
  logic [11:0]  in_ch_d;
  logic [11:0]  out_ch_d;
  logic [11:0]  feat_h_d;
  logic [11:0]  feat_w_d;
  logic [7:0]   stride_d;
  logic [7:0]   act_fn_d;
  logic [7:0]   requant_shift_d;
  logic [15:0]  weight_base_d;
  logic [31:0]  bias_base_d;

  // per-layer derived counts
  logic [31:0]  weight_words_total;
  logic [31:0]  act_elems_total;
  logic [31:0]  wait_pe_cycles_total;
  logic [31:0]  postop_cycles_total;

  logic [31:0]  weight_cnt;
  logic [31:0]  act_cnt;
  logic [31:0]  wait_cnt;
  logic [31:0]  postop_cnt;

  logic [15:0]  tile_offset_words;

  logic         start_d;
  logic         start_pulse;

  integer i;

  //--------------------------------------------------------------------------
  // Start pulse detect
  //--------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      start_d <= 1'b0;
    else
      start_d <= start;
  end

  assign start_pulse = start & ~start_d;

  //--------------------------------------------------------------------------
  // Microcode ROM write
  //--------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (mc_we)
      mc_rom[mc_addr] <= mc_wdata;
  end

  //--------------------------------------------------------------------------
  // Descriptor decode
  //--------------------------------------------------------------------------
  always_comb begin
    layer_type_d      = layer_desc_reg[7:0];
    in_ch_d           = layer_desc_reg[19:8];
    out_ch_d          = layer_desc_reg[31:20];
    feat_h_d          = layer_desc_reg[43:32];
    feat_w_d          = layer_desc_reg[55:44];
    stride_d          = layer_desc_reg[63:56];
    act_fn_d          = layer_desc_reg[71:64];
    requant_shift_d   = layer_desc_reg[79:72];
    weight_base_d     = layer_desc_reg[95:80];
    bias_base_d       = layer_desc_reg[127:96];

    layer_valid       = (layer_desc_reg != 128'd0);

    // Approximate number of 64-bit weight words for one layer.
    // 8 INT8 weights per word.
    // For pool/add layers, no weights are needed.
    if ((layer_type_d == 8'd4) || (layer_type_d == 8'd5))
      weight_words_total = 32'd0;
    else
      weight_words_total = ((in_ch_d * out_ch_d) + 32'd7) >> 3;

    // Approximate activation stream count:
    // one address per spatial point
    act_elems_total = feat_h_d * feat_w_d;

    // Fixed-latency placeholders for PE drain / postop completion.
    wait_pe_cycles_total = 32'd16;
    postop_cycles_total  = 32'd4;
  end

  //--------------------------------------------------------------------------
  // Main FSM
  //--------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state              <= ST_IDLE;
      busy               <= 1'b0;
      done               <= 1'b0;

      prog_base          <= '0;
      layer_idx          <= '0;
      curr_mc_addr       <= '0;
      layer_desc_reg     <= '0;

      weight_load        <= 1'b0;
      weight_rd_addr     <= '0;
      act_clear          <= 1'b0;
      act_valid_out      <= 1'b0;

      act_fn_out         <= 2'b00;
      requant_shift_out  <= 8'd0;
      pool_en_out        <= 1'b0;

      act_bank_sel       <= 1'b0;
      act_rd_addr        <= '0;
      act_wr_addr        <= '0;

      inference_cycles   <= '0;

      weight_cnt         <= '0;
      act_cnt            <= '0;
      wait_cnt           <= '0;
      postop_cnt         <= '0;
      tile_offset_words  <= '0;
    end
    else begin
      // defaults
      done          <= 1'b0;
      weight_load   <= 1'b0;
      act_clear     <= 1'b0;
      act_valid_out <= 1'b0;

      if (busy)
        inference_cycles <= inference_cycles + 32'd1;

      case (state)
        //--------------------------------------------------------------------
        ST_IDLE: begin
          busy             <= 1'b0;
          weight_cnt       <= '0;
          act_cnt          <= '0;
          wait_cnt         <= '0;
          postop_cnt       <= '0;
          tile_offset_words<= '0;
          act_rd_addr      <= '0;
          act_wr_addr      <= '0;

          if (start_pulse) begin
            busy             <= 1'b1;
            inference_cycles <= 32'd0;
            prog_base        <= task_sel * TASK_STRIDE;
            layer_idx        <= 5'd0;
            curr_mc_addr     <= task_sel * TASK_STRIDE;
            state            <= ST_LOAD_LAYER_DESC;

            // synthesis translate_off
            $display("NPU_SCHED: start task_sel=%0d base=%0d @ t=%0t",
                     task_sel, task_sel*TASK_STRIDE, $time);
            // synthesis translate_on
          end
        end

        //--------------------------------------------------------------------
        ST_LOAD_LAYER_DESC: begin
          layer_desc_reg <= mc_rom[curr_mc_addr];

          // synthesis translate_off
          $display("NPU_SCHED: load desc addr=%0d data=%h @ t=%0t",
                   curr_mc_addr, mc_rom[curr_mc_addr], $time);
          // synthesis translate_on

          state <= ST_ISSUE_WEIGHTS;
        end

       ST_ISSUE_WEIGHTS: begin
  if (!layer_valid) begin
    state <= ST_DONE;
  end
  else begin
    act_fn_out        <= act_fn_d[1:0];
    requant_shift_out <= requant_shift_d;
    pool_en_out       <= (layer_type_d == 8'd4);

    if (weight_cnt == 32'd0) begin
      weight_rd_addr <= {2'b00, weight_base_d};
      act_clear      <= 1'b1;
      weight_load    <= (weight_words_total != 0);

      // synthesis translate_off
      $display("NPU_SCHED: layer=%0d type=%0d in_ch=%0d out_ch=%0d feat=%0dx%0d stride=%0d act=%0d rq=%0d wbase=%0d bbase=%0d",
                layer_idx, layer_type_d, in_ch_d, out_ch_d, feat_h_d, feat_w_d,
                stride_d, act_fn_d, requant_shift_d, weight_base_d, bias_base_d);
      // synthesis translate_on
    end

    if (weight_cnt < weight_words_total) begin
      weight_load    <= 1'b1;
      weight_rd_addr <= {2'b00, weight_base_d} + weight_cnt[17:0];
      weight_cnt     <= weight_cnt + 32'd1;
    end
    else begin
      weight_cnt  <= 32'd0;
      act_rd_addr <= 20'd0;
      act_wr_addr <= 20'd0;
      state       <= ST_STREAM_ACT;
    end
  end
end

        //--------------------------------------------------------------------
        ST_STREAM_ACT: begin
          if (act_cnt < act_elems_total) begin
            act_valid_out <= 1'b1;

            // Read current bank, write opposite bank
            act_rd_addr <= act_cnt[19:0];
            act_wr_addr <= act_cnt[19:0];

            act_cnt <= act_cnt + 32'd1;
          end
          else begin
            act_valid_out <= 1'b0;
            act_cnt       <= 32'd0;
            wait_cnt      <= 32'd0;
            state         <= ST_WAIT_PE;
          end
        end

        //--------------------------------------------------------------------
        ST_WAIT_PE: begin
          if (wait_cnt < wait_pe_cycles_total) begin
            wait_cnt <= wait_cnt + 32'd1;
          end
          else begin
            wait_cnt   <= 32'd0;
            postop_cnt <= 32'd0;
            state      <= ST_POST_OP;
          end
        end

        //--------------------------------------------------------------------
        ST_POST_OP: begin
          if (postop_cnt < postop_cycles_total) begin
            postop_cnt <= postop_cnt + 32'd1;
          end
          else begin
            postop_cnt <= 32'd0;
            state      <= ST_NEXT_LAYER;
          end
        end

        //--------------------------------------------------------------------
        ST_NEXT_LAYER: begin
          layer_idx      <= layer_idx + 5'd1;
          curr_mc_addr   <= prog_base + layer_idx + 5'd1;
          act_bank_sel   <= ~act_bank_sel;  // ping-pong swap per layer
          tile_offset_words <= '0;
          state          <= ST_LOAD_LAYER_DESC;

          // synthesis translate_off
          $display("NPU_SCHED: next layer=%0d bank_sel=%0d @ t=%0t",
                   layer_idx + 1, ~act_bank_sel, $time);
          // synthesis translate_on
        end

        //--------------------------------------------------------------------
        ST_DONE: begin
          busy  <= 1'b0;
          done  <= 1'b1;
          state <= ST_IDLE;

          // synthesis translate_off
          $display("NPU_SCHED: done cycles=%0d @ t=%0t", inference_cycles, $time);
          // synthesis translate_on
        end

        default: begin
          state <= ST_IDLE;
        end
      endcase
    end
  end

endmodule