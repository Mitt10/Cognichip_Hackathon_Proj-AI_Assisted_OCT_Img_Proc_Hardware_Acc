`timescale 1ns/1ps
import oct_pkg::*;

//------------------------------------------------------------------------------
// npu_pe_array
//------------------------------------------------------------------------------
// 16x16 weight-stationary systolic INT8 MAC array.
//
// Each PE:
//   acc[row][col] += signed(weight_reg[row][col]) * signed(act_pipe[row][col])
//
// Architecture:
// - Weights are stationary: each PE has a local weight register loaded by
//   weight_load before a layer/tile starts.
// - Activations flow left-to-right across each row with a 1-cycle register
//   between column boundaries (true systolic skew).
// - Each PE maintains a local accumulator.
// - Final row outputs are the sum across all column accumulators in that row.
// - out_valid pulses ROWS cycles after act_last, per the requested spec.
//
// DSP mapping note:
// - The multiply in each PE is written as:
//       (* use_dsp = "yes" *) prod = $signed(weight_reg) * $signed(act)
// - On UltraScale+, Vivado will typically infer DSP48E2 for the signed INT8
//   multiply when timing/resource constraints favor it.
// - If stricter control is needed, replace the inferred multiply with an
//   explicit DSP48E2 wrapper per PE.
//
// Placement/routing note for XCZU7EV:
// - For best routing at 300 MHz, split this 16x16 array into 4 sub-arrays of
//   size 8x8:
//      * rows [0:7],   cols [0:7]
//      * rows [0:7],   cols [8:15]
//      * rows [8:15],  cols [0:7]
//      * rows [8:15],  cols [8:15]
// - Floorplan each 8x8 block near one DSP column region / clock region.
// - Keep activation pipeline registers local to each sub-array boundary and
//   register cross-subarray signals at the split lines.
// - This reduces long horizontal routes and improves DSP cascade locality.
//
//------------------------------------------------------------------------------
module npu_pe_array #(
  parameter int ROWS    = 16,
  parameter int COLS    = 16,
  parameter int DATA_W  = 8,
  parameter int ACCUM_W = 32
)(
  input  logic                           clk,
  input  logic                           rst_n,

  // Weight load (one ROWS x COLS tile, loaded before each layer starts)
  input  logic [DATA_W-1:0]              weight_in [0:ROWS-1][0:COLS-1],
  input  logic                           weight_load,

  // Activation input (COLS activations per cycle, streamed in)
  input  logic [DATA_W-1:0]              act_in [0:COLS-1],
  input  logic                           act_valid,
  input  logic                           act_last,

  // Accumulator output (ROWS accumulators, valid when out_valid)
  output logic [ACCUM_W-1:0]             acc_out [0:ROWS-1],
  output logic                           out_valid,

  // Clear accumulator (between output tiles)
  input  logic                           acc_clear
);

  //----------------------------------------------------------------------------
  // Internal storage
  //----------------------------------------------------------------------------
  logic signed [DATA_W-1:0]  weight_reg [0:ROWS-1][0:COLS-1];
  logic signed [DATA_W-1:0]  act_pipe   [0:ROWS-1][0:COLS-1];
  logic                      vld_pipe   [0:ROWS-1][0:COLS-1];

  logic signed [ACCUM_W-1:0] pe_acc     [0:ROWS-1][0:COLS-1];
  logic signed [2*DATA_W-1:0] pe_prod   [0:ROWS-1][0:COLS-1];

  logic                      act_last_dly [0:ROWS-1];

  logic signed [ACCUM_W-1:0] row_sum_comb [0:ROWS-1];

  integer r, c;

  //----------------------------------------------------------------------------
  // Weight registers
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        for (c = 0; c < COLS; c = c + 1) begin
          weight_reg[r][c] <= '0;
        end
      end
    end
    else if (weight_load) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        for (c = 0; c < COLS; c = c + 1) begin
          weight_reg[r][c] <= signed'(weight_in[r][c]);
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Activation systolic pipeline
  //
  // act_in[col] is injected at column 0 of every row.
  // Each column boundary adds one pipeline cycle.
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        for (c = 0; c < COLS; c = c + 1) begin
          act_pipe[r][c] <= '0;
          vld_pipe[r][c] <= 1'b0;
        end
      end
    end
    else begin
      for (r = 0; r < ROWS; r = r + 1) begin
        act_pipe[r][0] <= signed'(act_in[r]);
        vld_pipe[r][0] <= act_valid;

        for (c = 1; c < COLS; c = c + 1) begin
          act_pipe[r][c] <= act_pipe[r][c-1];
          vld_pipe[r][c] <= vld_pipe[r][c-1];
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // act_last delay line
  //
  // Per user spec, out_valid pulses ROWS cycles after act_last.
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        act_last_dly[r] <= 1'b0;
      end
    end
    else begin
      act_last_dly[0] <= act_last;
      for (r = 1; r < ROWS; r = r + 1) begin
        act_last_dly[r] <= act_last_dly[r-1];
      end
    end
  end

  assign out_valid = act_last_dly[ROWS-1];

  //----------------------------------------------------------------------------
  // PE multiply terms
  //
  // Synthesis hint:
  //   For explicit DSP48E2 mapping, replace this inferred multiply with a
  //   small PE wrapper instantiating DSP48E2 directly.
  //----------------------------------------------------------------------------
  genvar gr, gc;
  generate
    for (gr = 0; gr < ROWS; gr = gr + 1) begin : G_ROW_PROD
      for (gc = 0; gc < COLS; gc = gc + 1) begin : G_COL_PROD
        always_comb begin
          (* use_dsp = "yes" *)
          pe_prod[gr][gc] = $signed(weight_reg[gr][gc]) * $signed(act_pipe[gr][gc]);
        end
      end
    end
  endgenerate

  //----------------------------------------------------------------------------
  // PE accumulators
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        for (c = 0; c < COLS; c = c + 1) begin
          pe_acc[r][c] <= '0;
        end
      end
    end
    else begin
      for (r = 0; r < ROWS; r = r + 1) begin
        for (c = 0; c < COLS; c = c + 1) begin
          if (acc_clear) begin
            pe_acc[r][c] <= '0;
          end
          else if (vld_pipe[r][c]) begin
            pe_acc[r][c] <= pe_acc[r][c] + {{(ACCUM_W-(2*DATA_W)){pe_prod[r][c][2*DATA_W-1]}}, pe_prod[r][c]};
          end
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Row reduction across columns
  //
  // All COLS PE accumulators in a row are summed to form acc_out[row].
  //----------------------------------------------------------------------------
  integer rr, cc;
  always_comb begin
    for (rr = 0; rr < ROWS; rr = rr + 1) begin
      row_sum_comb[rr] = '0;
      for (cc = 0; cc < COLS; cc = cc + 1) begin
        row_sum_comb[rr] = row_sum_comb[rr] + pe_acc[rr][cc];
      end
    end
  end

  //----------------------------------------------------------------------------
  // Registered outputs
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        acc_out[r] <= '0;
      end
    end
    else if (out_valid) begin
      for (r = 0; r < ROWS; r = r + 1) begin
        acc_out[r] <= row_sum_comb[r];
      end
    end
  end

endmodule