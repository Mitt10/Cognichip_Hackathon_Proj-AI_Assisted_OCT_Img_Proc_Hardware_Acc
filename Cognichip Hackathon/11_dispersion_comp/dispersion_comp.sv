// ============================================================================
// dispersion_comp.sv
// Complex-valued dispersion compensation (pre-FFT phase-vector multiply)
//
// Reference : Shannon et al. / Tang et al. (OCT dispersion compensation)
//
// Target    : Xilinx UltraScale+ @ 250 MHz
// Resources : 1x BRAM36, 2x DSP48E2, ~250 LUT
//
// Operation : out[k] = in[k] * (cos phi[k] + j*sin phi[k])
//             real part : in[k] * cos phi[k]   (DSP0)
//             imag part : in[k] * sin phi[k]   (DSP1)
//             Phase coefficients are Q1.17 signed (PHASE_W=18).
//
// Pipeline depth : 3 clock cycles
//
//   Stage 1 : Input capture + BRAM registered read  (s_tdata, cos, sin)
//   Stage 2 : DSP48E2 multiply x2  (re = data*cos, im = data*sin)
//   Stage 3 : Round Q1.17 to int16, saturate, pack, output register
//
// Bypass    : when bypass==1, passes {16'b0, s_tdata} with identical
//             3-cycle latency so downstream sees no pipeline gap.
// ============================================================================

`ifndef OCT_PKG_SV
  `include "../1_oct_pkg/oct_pkg.sv"
`endif

import oct_pkg::*;

module dispersion_comp #(
  parameter int N       = 1024,  // samples per A-scan
  parameter int DATA_W  = 16,    // real input width (signed)
  parameter int PHASE_W = 18,    // Q1.17 phase coefficient width (cos/sin)
  parameter int OUT_W   = 16     // per-component output width
)(
  input  logic              clk,
  input  logic              rst_n,

  // AXI4-Stream slave (real 16-bit samples)
  input  logic [DATA_W-1:0] s_tdata,
  input  logic              s_tvalid,
  input  logic              s_tlast,
  input  logic [31:0]       s_tuser,
  output logic              s_tready,

  // AXI4-Stream master (complex 32-bit packed {imag[15:0], real[15:0]})
  output logic [31:0]       m_tdata,
  output logic              m_tvalid,
  output logic              m_tlast,
  output logic [31:0]       m_tuser,
  input  logic              m_tready,

  // Dispersion phase BRAM write port (from ARM PS at calibration)
  input  logic              phase_we,
  input  logic [9:0]        phase_addr,   // sample index 0-1023
  input  logic [35:0]       phase_wdata,  // {cos_q17[17:0], sin_q17[17:0]}

  // Bypass control (from oct_ctrl_t.bypass_disp)
  input  logic              bypass        // 1 = pass-through with 3-cycle latency
);

  // --------------------------------------------------------------------------
  // Local parameters
  // --------------------------------------------------------------------------
  localparam int ADDR_W = $clog2(N);           // 10
  localparam int PROD_W = DATA_W + PHASE_W;    // 34  (signed product width)
  localparam int SHIFT  = PHASE_W - 1;         // 17  (bits to shift to undo Q1.17)

  // Saturated output width for one component (should equal OUT_W)
  localparam int SAT_MAX = (1 << (OUT_W - 1)) - 1;   //  32767
  localparam int SAT_MIN = -(1 << (OUT_W - 1));       // -32768

  // --------------------------------------------------------------------------
  // 1. Pipeline enable
  //    Stall the entire pipeline when the output register holds valid data
  //    that the downstream has not yet accepted.
  // --------------------------------------------------------------------------
  logic pipe_en;
  assign pipe_en  = !m_tvalid || m_tready;
  assign s_tready = pipe_en;

  // --------------------------------------------------------------------------
  // 2. Sample counter  (0 .. N-1)
  //    Tracks which phase coefficient corresponds to the current sample.
  //    Uses pre-increment value at each clock edge so phase_bram[samp_cnt]
  //    and s_tdata are always aligned in Stage 1.
  // --------------------------------------------------------------------------
  logic [ADDR_W-1:0] samp_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      samp_cnt <= '0;
    else if (s_tvalid && s_tready) begin
      if (s_tlast) samp_cnt <= '0;
      else         samp_cnt <= samp_cnt + 1'b1;
    end
  end

  // --------------------------------------------------------------------------
  // 3. Phase BRAM  (1024 x 36-bit = 1x BRAM36)
  //    Write port : ARM PS via phase_we / phase_addr / phase_wdata
  //    Read port  : registered read keyed by samp_cnt (simple dual-port BRAM)
  //
  //    The read is issued combinatorially from samp_cnt and registered in
  //    Stage 1.  At each posedge, samp_cnt still holds the PRE-increment
  //    address k for the sample being accepted, so phase_bram[k] and
  //    s_tdata[k] land in p1 registers on the same clock edge.
  // --------------------------------------------------------------------------
  (* ram_style = "block" *)
  logic [35:0] phase_bram [0:N-1];

  always_ff @(posedge clk) begin
    if (phase_we) phase_bram[phase_addr] <= phase_wdata;
  end

  // --------------------------------------------------------------------------
  // Stage 1 : Input capture + BRAM registered read
  // --------------------------------------------------------------------------
  logic signed [DATA_W-1:0]  p1_data;
  logic signed [PHASE_W-1:0] p1_cos;    // cos phi[k]  Q1.17
  logic signed [PHASE_W-1:0] p1_sin;    // sin phi[k]  Q1.17
  logic                       p1_valid;
  logic                       p1_last;
  logic [31:0]                p1_tuser;
  logic                       p1_bypass;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p1_valid  <= 1'b0;
      p1_data   <= '0;
      p1_cos    <= '0;
      p1_sin    <= '0;
      p1_last   <= 1'b0;
      p1_tuser  <= '0;
      p1_bypass <= 1'b0;
    end else if (pipe_en) begin
      p1_valid  <= s_tvalid;
      p1_data   <= signed'(s_tdata);
      // BRAM registered read: address = samp_cnt (pre-increment value k).
      // Synthesis infers simple-dual-port BRAM with (* ram_style="block" *).
      p1_cos    <= signed'(phase_bram[samp_cnt][35:18]);  // upper 18 bits
      p1_sin    <= signed'(phase_bram[samp_cnt][17:0]);   // lower 18 bits
      p1_last   <= s_tlast;
      p1_tuser  <= s_tuser;
      p1_bypass <= bypass;
    end
  end

  // --------------------------------------------------------------------------
  // Stage 2 : DSP48E2 multiply
  //   prod_re = data * cos   (Q16.17, 34-bit signed)
  //   prod_im = data * sin   (Q16.17, 34-bit signed)
  //   data bypass is propagated for the bypass mux in Stage 3.
  // --------------------------------------------------------------------------
  (* use_dsp = "yes" *)
  logic signed [PROD_W-1:0]  p2_prod_re;
  (* use_dsp = "yes" *)
  logic signed [PROD_W-1:0]  p2_prod_im;
  logic signed [DATA_W-1:0]  p2_data;      // raw data for bypass path
  logic                       p2_valid;
  logic                       p2_last;
  logic [31:0]                p2_tuser;
  logic                       p2_bypass;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p2_valid   <= 1'b0;
      p2_prod_re <= '0;
      p2_prod_im <= '0;
      p2_data    <= '0;
      p2_last    <= 1'b0;
      p2_tuser   <= '0;
      p2_bypass  <= 1'b0;
    end else if (pipe_en) begin
      p2_valid   <= p1_valid;
      // Signed widened multiply: DATA_W(16) sign-extended to PROD_W(34),
      // PHASE_W(18) sign-extended to PROD_W(34).  Both operands are declared
      // signed so the cast preserves sign (sign-extension).
      p2_prod_re <= PROD_W'(p1_data) * PROD_W'(p1_cos);
      p2_prod_im <= PROD_W'(p1_data) * PROD_W'(p1_sin);
      p2_data    <= p1_data;
      p2_last    <= p1_last;
      p2_tuser   <= p1_tuser;
      p2_bypass  <= p1_bypass;
    end
  end

  // --------------------------------------------------------------------------
  // Stage 3 : Round Q1.17 -> 16-bit integer, saturate, pack, output register
  //
  //   Product is PROD_W=34-bit signed, format Q(DATA_W-1+1).(PHASE_W-1)
  //   = Q16.17.  Shifting right by SHIFT=17 recovers bits[33:17] as a
  //   17-bit signed integer.  Round by adding 0.5 LSB (1<<16) before shift.
  //   Saturate the 17-bit result to OUT_W=16 bits.
  // --------------------------------------------------------------------------

  // Combinational round-and-saturate function
  function automatic logic signed [OUT_W-1:0] round_sat(
    input logic signed [PROD_W-1:0] p
  );
    // Extended width to detect overflow after rounding
    localparam int EXT_W = PROD_W + 1;   // 35
    logic signed [EXT_W-1:0] rounded;
    logic signed [EXT_W-1:0] shifted;
    logic signed [OUT_W-1:0] result;

    // Add 0.5 LSB at fractional position 16 (= 2^16) for round-to-nearest
    rounded = EXT_W'(p) + EXT_W'(1 << (SHIFT - 1));
    // Arithmetic right-shift by SHIFT=17
    shifted = rounded >>> SHIFT;

    // Saturate to OUT_W bits
    if      (shifted > EXT_W'(SAT_MAX))
      result = OUT_W'(SAT_MAX);
    else if (shifted < EXT_W'(signed'(SAT_MIN)))
      result = OUT_W'(signed'(SAT_MIN));
    else
      result = OUT_W'(shifted);

    return result;
  endfunction

  // Compute rounded/saturated components combinationally from Stage 2
  logic signed [OUT_W-1:0] s3_re, s3_im;

  always_comb begin : stage3_round
    s3_re = round_sat(p2_prod_re);
    s3_im = round_sat(p2_prod_im);
  end

  // Output register (Stage 3 flip-flop)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_tdata  <= '0;
      m_tvalid <= 1'b0;
      m_tlast  <= 1'b0;
      m_tuser  <= '0;
    end else if (pipe_en) begin
      m_tvalid <= p2_valid;
      m_tlast  <= p2_last;
      m_tuser  <= p2_tuser;
      if (p2_bypass)
        // Bypass: {zero imaginary, original real data}
        m_tdata <= {16'b0, OUT_W'(p2_data)};
      else
        // Normal: {imag[15:0], real[15:0]}
        m_tdata <= {OUT_W'(s3_im), OUT_W'(s3_re)};
    end
  end

endmodule : dispersion_comp
