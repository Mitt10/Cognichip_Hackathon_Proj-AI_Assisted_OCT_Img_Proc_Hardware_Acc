`timescale 1ns/1ps
import oct_pkg::*;

//------------------------------------------------------------------------------
// npu_postop
//------------------------------------------------------------------------------
// Post-processing pipeline after the PE array:
//
//   Stage 1 : acc + bias
//   Stage 2 : batch-norm scale multiply, then >> 15
//   Stage 3 : + batch-norm shift
//   Stage 4 : activation
//   Stage 5 : requantize to INT8 with saturation
//   Stage 6 : optional 2x2 max-pool (implemented as 2-cycle wrapper)
//
// Notes / assumptions:
// 1) The requested interface presents all CHANNELS in parallel and asks for
//    7-cycle latency without pooling. That conflicts with a strict
//    time-multiplexed 1-DSP-over-CHANNELS-cycles implementation.
//    This RTL therefore implements the datapath in parallel per channel so the
//    stated latency is preserved.
// 2) The requested 2x2 max-pool needs spatial row/column metadata, which is
//    not present on the interface. This RTL assumes each acc_valid corresponds
//    to one spatial sample in raster order and pools across each set of four
//    successive vectors forming a 2x2 window:
//        sample0 = row0,col0
//        sample1 = row0,col1
//        sample2 = row1,col0
//        sample3 = row1,col1
//    Output is emitted on every 4th valid when pool_en=1.
// 3) bn_scale is treated as signed Q0.15.
//    bn_shift is provided as SCALE_W bits in the spec; here it is sign-extended
//    and treated as a post-scale additive term in the accumulator domain.
// 4) GELU-approx uses a 256-entry sigmoid LUT indexed by x[31:24] with
//      y = x * sigmoid(1.702*x)
//    approximated as:
//      gelu = (x * sigmoid_lut[idx]) >> 8
//
// Resource/timing note:
// - This version is written for synthesis cleanliness and high Fmax.
// - If you truly want 2-4 DSP48E2 total, the interface must be serialized or
//   widened with a ready/valid micro-engine handshake.
//------------------------------------------------------------------------------
module npu_postop #(
  parameter int CHANNELS = 16,
  parameter int ACCUM_W  = 32,
  parameter int SCALE_W  = 16,
  parameter int BIAS_W   = 32,
  parameter int OUT_W    = 8
)(
  input  logic                           clk,
  input  logic                           rst_n,

  // From PE array
  input  logic [ACCUM_W-1:0]             acc_in      [0:CHANNELS-1],
  input  logic                           acc_valid,

  // Per-channel params
  input  logic [BIAS_W-1:0]              bias        [0:CHANNELS-1],
  input  logic [SCALE_W-1:0]             bn_scale    [0:CHANNELS-1],
  input  logic [SCALE_W-1:0]             bn_shift    [0:CHANNELS-1],
  input  logic [7:0]                     requant_shift,
  input  logic [1:0]                     act_fn,      // 0=linear,1=ReLU,2=ReLU6,3=GELU
  input  logic                           pool_en,

  // Output
  output logic [OUT_W-1:0]               out_data    [0:CHANNELS-1],
  output logic                           out_valid
);

  //----------------------------------------------------------------------------
  // Local params / types
  //----------------------------------------------------------------------------
  localparam int MUL_W = ACCUM_W + SCALE_W;  // 48 for 32x16

  typedef logic signed [ACCUM_W-1:0] sacc_t;
  typedef logic signed [MUL_W-1:0]   smul_t;
  typedef logic signed [OUT_W-1:0]   sout_t;

  //----------------------------------------------------------------------------
  // Helper functions
  //----------------------------------------------------------------------------
  function automatic sacc_t sat_signed_accum_from_mul(input smul_t x);
    logic signed [ACCUM_W-1:0] maxp;
    logic signed [ACCUM_W-1:0] minn;
    logic signed [MUL_W-1:0]   xmax;
    logic signed [MUL_W-1:0]   xmin;
    begin
      maxp = {1'b0, {(ACCUM_W-1){1'b1}}};
      minn = {1'b1, {(ACCUM_W-1){1'b0}}};
      xmax = {{(MUL_W-ACCUM_W){maxp[ACCUM_W-1]}}, maxp};
      xmin = {{(MUL_W-ACCUM_W){minn[ACCUM_W-1]}}, minn};

      if (x > xmax)
        sat_signed_accum_from_mul = maxp;
      else if (x < xmin)
        sat_signed_accum_from_mul = minn;
      else
        sat_signed_accum_from_mul = x[ACCUM_W-1:0];
    end
  endfunction

  function automatic sacc_t relu_fn(input sacc_t x);
    begin
      if (x[ACCUM_W-1])
        relu_fn = '0;
      else
        relu_fn = x;
    end
  endfunction

  function automatic sacc_t relu6_fn(input sacc_t x, input logic [7:0] rq_shift);
    sacc_t six_lim;
    begin
      six_lim = sacc_t'(32'sd6 <<< rq_shift);
      if (x[ACCUM_W-1])
        relu6_fn = '0;
      else if (x > six_lim)
        relu6_fn = six_lim;
      else
        relu6_fn = x;
    end
  endfunction

  function automatic sout_t requant_sat_int8(
    input sacc_t x,
    input logic [7:0] rq_shift
  );
    logic signed [ACCUM_W-1:0] shifted;
    begin
      shifted = x >>> rq_shift;
      if (shifted > 32'sd127)
        requant_sat_int8 = 8'sd127;
      else if (shifted < -32'sd128)
        requant_sat_int8 = -8'sd128;
      else
        requant_sat_int8 = shifted[OUT_W-1:0];
    end
  endfunction

  //----------------------------------------------------------------------------
  // GELU sigmoid LUT
  //
  // LUT stores sigmoid(1.702*x_norm) in unsigned Q0.8 over index 0..255,
  // where index is formed from x[31:24] interpreted as signed.
  // This is a coarse approximation intended for hardware efficiency.
  //----------------------------------------------------------------------------
  (* rom_style = "distributed" *) logic [7:0] gelu_sigmoid_lut [0:255];

  function automatic logic [7:0] sigmoid_lut_gen(input int idx);
    real x_norm;
    real z;
    real s;
    int  q;
    begin
      // map idx 0..255 to signed range about [-8, +7.9375]
      x_norm = (idx - 128) / 16.0;
      z      = 1.702 * x_norm;
      s      = 1.0 / (1.0 + $exp(-z));
      q      = $rtoi(s * 255.0 + 0.5);
      if (q < 0)   q = 0;
      if (q > 255) q = 255;
      sigmoid_lut_gen = q[7:0];
    end
  endfunction

  integer lut_i;
  initial begin
    for (lut_i = 0; lut_i < 256; lut_i = lut_i + 1)
      gelu_sigmoid_lut[lut_i] = sigmoid_lut_gen(lut_i);
  end

  //----------------------------------------------------------------------------
  // Pipeline registers
  //----------------------------------------------------------------------------
  sacc_t s1_addbias [0:CHANNELS-1];
  logic  s1_valid;

  smul_t s2_mul     [0:CHANNELS-1];
  sacc_t s2_scaled  [0:CHANNELS-1];
  logic  s2_valid;

  sacc_t s3_shifted [0:CHANNELS-1];
  logic  s3_valid;

  sacc_t s4_acted   [0:CHANNELS-1];
  logic  s4_valid;

  sout_t s5_quant   [0:CHANNELS-1];
  logic  s5_valid;

  // pool regs
  sout_t p_hold0    [0:CHANNELS-1];
  sout_t p_hold1    [0:CHANNELS-1];
  sout_t p_hold2    [0:CHANNELS-1];
  sout_t p_max01    [0:CHANNELS-1];
  sout_t p_max23    [0:CHANNELS-1];
  sout_t p_max0123  [0:CHANNELS-1];

  logic [1:0] pool_phase;
  logic       pool_out_valid;

  integer ch;

  //----------------------------------------------------------------------------
  // Stage 1: acc + bias
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s1_valid <= 1'b0;
      for (ch = 0; ch < CHANNELS; ch = ch + 1)
        s1_addbias[ch] <= '0;
    end else begin
      s1_valid <= acc_valid;
      if (acc_valid) begin
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
          s1_addbias[ch] <= $signed(acc_in[ch]) + $signed(bias[ch]);
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Stage 2: * bn_scale then >> 15
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s2_valid <= 1'b0;
      for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
        s2_mul[ch]    <= '0;
        s2_scaled[ch] <= '0;
      end
    end else begin
      s2_valid <= s1_valid;
      if (s1_valid) begin
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
          (* use_dsp = "yes" *)
          s2_mul[ch]    <= $signed(s1_addbias[ch]) * $signed(bn_scale[ch]);
          s2_scaled[ch] <= sat_signed_accum_from_mul(
                             ($signed(s1_addbias[ch]) * $signed(bn_scale[ch])) >>> 15
                           );
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Stage 3: + bn_shift
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s3_valid <= 1'b0;
      for (ch = 0; ch < CHANNELS; ch = ch + 1)
        s3_shifted[ch] <= '0;
    end else begin
      s3_valid <= s2_valid;
      if (s2_valid) begin
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
          s3_shifted[ch] <= s2_scaled[ch] +
                            {{(ACCUM_W-SCALE_W){bn_shift[ch][SCALE_W-1]}}, bn_shift[ch]};
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Stage 4: activation
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s4_valid <= 1'b0;
      for (ch = 0; ch < CHANNELS; ch = ch + 1)
        s4_acted[ch] <= '0;
    end else begin
      s4_valid <= s3_valid;
      if (s3_valid) begin
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
          unique case (act_fn)
            2'd0: begin
              s4_acted[ch] <= s3_shifted[ch];
            end

            2'd1: begin
              s4_acted[ch] <= relu_fn(s3_shifted[ch]);
            end

            2'd2: begin
              s4_acted[ch] <= relu6_fn(s3_shifted[ch], requant_shift);
            end

            2'd3: begin
              // GELU approx: x * sigmoid(1.702*x)
              // sigmoid LUT is Q0.8, so >>8 after multiply
              s4_acted[ch] <= sat_signed_accum_from_mul(
                                ($signed(s3_shifted[ch]) *
                                 $signed({1'b0, gelu_sigmoid_lut[s3_shifted[ch][31:24]]})) >>> 8
                              );
            end

            default: begin
              s4_acted[ch] <= s3_shifted[ch];
            end
          endcase
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Stage 5: requantize to INT8
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s5_valid <= 1'b0;
      for (ch = 0; ch < CHANNELS; ch = ch + 1)
        s5_quant[ch] <= '0;
    end else begin
      s5_valid <= s4_valid;
      if (s4_valid) begin
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
          s5_quant[ch] <= requant_sat_int8(s4_acted[ch], requant_shift);
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // Stage 6/7: optional 2x2 max-pool
  //
  // Assumption:
  //   Four consecutive valid vectors correspond to one 2x2 spatial window.
  //   Output is emitted on the 4th vector when pool_en=1.
  //
  //   phase 0 -> hold sample0
  //   phase 1 -> compute max(sample0, sample1)
  //   phase 2 -> hold sample2
  //   phase 3 -> compute max(max01, sample2, sample3), emit
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pool_phase    <= 2'd0;
      pool_out_valid <= 1'b0;
      out_valid     <= 1'b0;
      for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
        p_hold0[ch]   <= '0;
        p_hold1[ch]   <= '0;
        p_hold2[ch]   <= '0;
        p_max01[ch]   <= '0;
        p_max23[ch]   <= '0;
        p_max0123[ch] <= '0;
        out_data[ch]  <= '0;
      end
    end else begin
      pool_out_valid <= 1'b0;
      out_valid      <= 1'b0;

      if (!pool_en) begin
        if (s5_valid) begin
          for (ch = 0; ch < CHANNELS; ch = ch + 1)
            out_data[ch] <= s5_quant[ch];
          out_valid <= 1'b1;
        end
      end else begin
        if (s5_valid) begin
          unique case (pool_phase)
            2'd0: begin
              for (ch = 0; ch < CHANNELS; ch = ch + 1)
                p_hold0[ch] <= s5_quant[ch];
              pool_phase <= 2'd1;
            end

            2'd1: begin
              for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
                p_hold1[ch] <= s5_quant[ch];
                p_max01[ch] <= ($signed(p_hold0[ch]) > $signed(s5_quant[ch])) ?
                               p_hold0[ch] : s5_quant[ch];
              end
              pool_phase <= 2'd2;
            end

            2'd2: begin
              for (ch = 0; ch < CHANNELS; ch = ch + 1)
                p_hold2[ch] <= s5_quant[ch];
              pool_phase <= 2'd3;
            end

            2'd3: begin
              for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
                p_max23[ch] <= ($signed(p_hold2[ch]) > $signed(s5_quant[ch])) ?
                               p_hold2[ch] : s5_quant[ch];
                p_max0123[ch] <=
                  ($signed(p_max01[ch]) >
                   $signed(($signed(p_hold2[ch]) > $signed(s5_quant[ch])) ? p_hold2[ch] : s5_quant[ch]))
                  ? p_max01[ch]
                  : (($signed(p_hold2[ch]) > $signed(s5_quant[ch])) ? p_hold2[ch] : s5_quant[ch]);

                out_data[ch] <=
                  ($signed(p_max01[ch]) >
                   $signed(($signed(p_hold2[ch]) > $signed(s5_quant[ch])) ? p_hold2[ch] : s5_quant[ch]))
                  ? p_max01[ch]
                  : (($signed(p_hold2[ch]) > $signed(s5_quant[ch])) ? p_hold2[ch] : s5_quant[ch]);
              end
              out_valid   <= 1'b1;
              pool_phase  <= 2'd0;
            end

            default: begin
              pool_phase <= 2'd0;
            end
          endcase
        end
      end
    end
  end

endmodule