
`ifndef OCT_PKG_SV
  `include "../1_oct_pkg/oct_pkg.sv"
`endif

import oct_pkg::*;

// ============================================================================
// klinear_resampler.sv
// Calibration-aware 5-tap cubic B-spline FIR resampler
//
// Reference : Desjardins et al. / Tang et al. (2019, 2021)
//             ~15 dB PSF improvement over linear interpolation
//
// Target    : Xilinx UltraScale+ @ 250 MHz
// Resources : 4× BRAM36, 5× DSP48E2, ~600 LUT
//
// Pipeline depth : 10 clock cycles (LUT rd → output valid)
//
//   Stage 0  : output counter + LUT read address
//   Stage 1  : LUT read data registered (int_idx, frac)
//   Stage 2  : BRAM sample read address generation (×5)
//   Stage 3  : BRAM read data registered (sample[0..4])
//   Stage 4  : Coefficient ROM read address (= frac, pipelined)
//   Stage 5  : Coefficient ROM data registered (c[0..4])
//   Stage 6  : DSP multiply  s[i] × c[i]  (×5)
//   Stage 7  : Adder level 1  (p01, p23, p4)
//   Stage 8  : Adder level 2  (p0123, p4)
//   Stage 9  : Final sum + saturate + output register
// ============================================================================

module klinear_resampler #(
  parameter int IN_LEN   = 2048,
  parameter int OUT_LEN  = 1024,
  parameter int DATA_W   = 16,
  parameter int IDX_W    = 11,
  parameter int FRAC_W   = 9,
  parameter int COEFF_W  = 16
)(
  input  logic                      clk,
  input  logic                      rst_n,

  // AXI4-Stream slave
  input  logic [DATA_W-1:0]         s_tdata,
  input  logic                      s_tvalid,
  input  logic                      s_tlast,
  input  logic [31:0]               s_tuser,
  output logic                      s_tready,

  // AXI4-Stream master
  output logic [DATA_W-1:0]         m_tdata,
  output logic                      m_tvalid,
  output logic                      m_tlast,
  output logic [31:0]               m_tuser,
  input  logic                      m_tready,

  // Resample index LUT write port
  input  logic                      lut_we,
  input  logic [9:0]                lut_addr,
  input  logic [IDX_W+FRAC_W-1:0]   lut_wdata
);

  // --------------------------------------------------------------------------
  // Local parameters
  // --------------------------------------------------------------------------
  localparam int ADDR_W     = $clog2(IN_LEN);
  localparam int OUT_ADDR_W = $clog2(OUT_LEN);
  localparam int PIPE       = 10;

  // --------------------------------------------------------------------------
  // 1. Ping-pong BRAM banks
  // --------------------------------------------------------------------------
  (* ram_style = "block" *)
  logic signed [DATA_W-1:0] bram_a [0:IN_LEN-1];
  (* ram_style = "block" *)
  logic signed [DATA_W-1:0] bram_b [0:IN_LEN-1];

  logic        wr_sel;
  logic        rd_bank_pending;   // bank of most recently completed input scan
  logic        rd_bank_active;    // bank currently being read for this output scan

  logic [ADDR_W-1:0] wr_addr;
  logic              wr_en_a, wr_en_b;

  assign wr_en_a = s_tvalid && s_tready && (wr_sel == 1'b0);
  assign wr_en_b = s_tvalid && s_tready && (wr_sel == 1'b1);

  always_ff @(posedge clk) begin
    if (wr_en_a) bram_a[wr_addr] <= signed'(s_tdata);
    if (wr_en_b) bram_b[wr_addr] <= signed'(s_tdata);
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_addr <= '0;
      wr_sel  <= 1'b0;
    end else if (s_tvalid && s_tready) begin
      if (s_tlast) begin
        wr_addr <= '0;
        wr_sel  <= ~wr_sel;
      end else begin
        wr_addr <= wr_addr + 1'b1;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_bank_pending <= 1'b1;
    end else if (s_tvalid && s_tready && s_tlast) begin
      rd_bank_pending <= wr_sel;   // bank that just finished being written
    end
  end

  assign s_tready = 1'b1;

  // --------------------------------------------------------------------------
  // 2. Index LUT
  // --------------------------------------------------------------------------
  (* ram_style = "block" *)
  logic [IDX_W+FRAC_W-1:0] idx_lut [0:OUT_LEN-1];

  always_ff @(posedge clk) begin
    if (lut_we) idx_lut[lut_addr] <= lut_wdata;
  end

  // --------------------------------------------------------------------------
  // 3. Coefficient ROM
  // --------------------------------------------------------------------------
  (* rom_style = "block" *)
  logic signed [COEFF_W-1:0] coeff_rom [0:511][0:4];

  function automatic logic signed [COEFF_W-1:0] bspline_coeff(
    input int frac_i,
    input int tap
  );
    real t, b;
    t = real'(frac_i) / 512.0;
    case (tap)
      0: b = (-t*t*t + 3.0*t*t - 3.0*t + 1.0) / 6.0;
      1: b = ( 3.0*t*t*t - 6.0*t*t + 4.0) / 6.0;
      2: b = (-3.0*t*t*t + 3.0*t*t + 3.0*t + 1.0) / 6.0;
      3: b =   t*t*t / 6.0;
      default: b = 0.0;
    endcase
    begin
      int raw;
      raw = $rtoi(b * 32768.0 + 0.5);
      if (raw >  32767) raw =  32767;
      if (raw < -32768) raw = -32768;
      return COEFF_W'(raw);
    end
  endfunction

  generate
    genvar gi, gj;
    for (gi = 0; gi < 512; gi++) begin : gen_rom_row
      for (gj = 0; gj < 5; gj++) begin : gen_rom_col
        initial coeff_rom[gi][gj] = bspline_coeff(gi, gj);
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // 4. Read-side FSM & pipeline
  // --------------------------------------------------------------------------
  logic scan_ready;
  logic [OUT_ADDR_W-1:0] out_cnt;
  logic                  rd_active;
  logic                  pipe_adv;

  assign pipe_adv = (!m_tvalid) || m_tready;

  logic [31:0] tuser_lat;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      tuser_lat <= '0;
    else if (s_tvalid && s_tready && s_tlast)
      tuser_lat <= s_tuser;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      scan_ready     <= 1'b0;
      rd_active      <= 1'b0;
      out_cnt        <= '0;
      rd_bank_active <= 1'b1;
    end else begin
      if (s_tvalid && s_tready && s_tlast)
        scan_ready <= 1'b1;

      if (pipe_adv) begin
        if (!rd_active && scan_ready) begin
          rd_active       <= 1'b1;
          out_cnt         <= '0;
          scan_ready      <= 1'b0;
          rd_bank_active  <= rd_bank_pending;
        end else if (rd_active) begin
          if (out_cnt == OUT_ADDR_W'(OUT_LEN - 1)) begin
            rd_active <= 1'b0;
            out_cnt   <= '0;
          end else begin
            out_cnt <= out_cnt + 1'b1;
          end
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Pipeline stage registers
  // --------------------------------------------------------------------------

  // Stage 0 -> 1
  logic [OUT_ADDR_W-1:0]      p0_out_cnt;
  logic                       p0_valid;
  logic [31:0]                p0_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p0_valid   <= 1'b0;
      p0_out_cnt <= '0;
      p0_tuser   <= '0;
    end else if (pipe_adv) begin
      p0_valid   <= rd_active;
      p0_out_cnt <= out_cnt;
      p0_tuser   <= tuser_lat;
    end
  end

  // Stage 1
  logic [IDX_W+FRAC_W-1:0]    p1_lut_data;
  logic                       p1_valid;
  logic [OUT_ADDR_W-1:0]      p1_out_cnt;
  logic [31:0]                p1_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p1_valid    <= 1'b0;
      p1_lut_data <= '0;
      p1_out_cnt  <= '0;
      p1_tuser    <= '0;
    end else if (pipe_adv) begin
      p1_valid    <= p0_valid;
      p1_lut_data <= idx_lut[p0_out_cnt];
      p1_out_cnt  <= p0_out_cnt;
      p1_tuser    <= p0_tuser;
    end
  end

  logic [IDX_W-1:0]           p1_int_idx;
  logic [FRAC_W-1:0]          p1_frac;
  assign p1_int_idx = p1_lut_data[IDX_W+FRAC_W-1:FRAC_W];
  assign p1_frac    = p1_lut_data[FRAC_W-1:0];

  // Stage 2
  function automatic logic [ADDR_W-1:0] clamp_addr(
    input logic signed [IDX_W:0] a
  );
    if (a < 0)
      return '0;
    else if (a > $signed((IDX_W+1)'(IN_LEN-1)))
      return ADDR_W'(IN_LEN-1);
    else
      return a[ADDR_W-1:0];
  endfunction

  logic [ADDR_W-1:0]          p2_addr [0:4];
  logic [FRAC_W-1:0]          p2_frac;
  logic                       p2_valid;
  logic [OUT_ADDR_W-1:0]      p2_out_cnt;
  logic [31:0]                p2_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p2_valid   <= 1'b0;
      p2_frac    <= '0;
      p2_out_cnt <= '0;
      p2_tuser   <= '0;
      for (int k = 0; k < 5; k++) p2_addr[k] <= '0;
    end else if (pipe_adv) begin
      p2_valid   <= p1_valid;
      p2_frac    <= p1_frac;
      p2_out_cnt <= p1_out_cnt;
      p2_tuser   <= p1_tuser;

      p2_addr[0] <= clamp_addr($signed({1'b0, p1_int_idx}) - 1);
      p2_addr[1] <= clamp_addr($signed({1'b0, p1_int_idx}));
      p2_addr[2] <= clamp_addr($signed({1'b0, p1_int_idx}) + 1);
      p2_addr[3] <= clamp_addr($signed({1'b0, p1_int_idx}) + 2);
      p2_addr[4] <= clamp_addr($signed({1'b0, p1_int_idx}) + 3);
    end
  end

  // Stage 3
  logic signed [DATA_W-1:0]   p3_samp [0:4];
  logic [FRAC_W-1:0]          p3_frac;
  logic                       p3_valid;
  logic [OUT_ADDR_W-1:0]      p3_out_cnt;
  logic [31:0]                p3_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p3_valid   <= 1'b0;
      p3_frac    <= '0;
      p3_out_cnt <= '0;
      p3_tuser   <= '0;
      for (int k = 0; k < 5; k++) p3_samp[k] <= '0;
    end else if (pipe_adv) begin
      p3_valid   <= p2_valid;
      p3_frac    <= p2_frac;
      p3_out_cnt <= p2_out_cnt;
      p3_tuser   <= p2_tuser;
      for (int k = 0; k < 5; k++) begin
        if (rd_bank_active == 1'b0)
          p3_samp[k] <= bram_a[p2_addr[k]];
        else
          p3_samp[k] <= bram_b[p2_addr[k]];
      end
    end
  end

  // Stage 4
  logic [FRAC_W-1:0]          p4_frac;
  logic signed [DATA_W-1:0]   p4_samp [0:4];
  logic                       p4_valid;
  logic [OUT_ADDR_W-1:0]      p4_out_cnt;
  logic [31:0]                p4_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p4_valid   <= 1'b0;
      p4_frac    <= '0;
      p4_out_cnt <= '0;
      p4_tuser   <= '0;
      for (int k = 0; k < 5; k++) p4_samp[k] <= '0;
    end else if (pipe_adv) begin
      p4_valid   <= p3_valid;
      p4_frac    <= p3_frac;
      p4_out_cnt <= p3_out_cnt;
      p4_tuser   <= p3_tuser;
      for (int k = 0; k < 5; k++) p4_samp[k] <= p3_samp[k];
    end
  end

  // Stage 5
  logic signed [COEFF_W-1:0]  p5_coeff [0:4];
  logic signed [DATA_W-1:0]   p5_samp  [0:4];
  logic                       p5_valid;
  logic [OUT_ADDR_W-1:0]      p5_out_cnt;
  logic [31:0]                p5_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p5_valid   <= 1'b0;
      p5_out_cnt <= '0;
      p5_tuser   <= '0;
      for (int k = 0; k < 5; k++) begin
        p5_coeff[k] <= '0;
        p5_samp[k]  <= '0;
      end
    end else if (pipe_adv) begin
      p5_valid   <= p4_valid;
      p5_out_cnt <= p4_out_cnt;
      p5_tuser   <= p4_tuser;
      for (int k = 0; k < 5; k++) begin
        p5_coeff[k] <= coeff_rom[p4_frac][k];
        p5_samp[k]  <= p4_samp[k];
      end
    end
  end

  // Stage 6
  localparam int PROD_W = DATA_W + COEFF_W;

  (* use_dsp = "yes" *)
  logic signed [PROD_W-1:0]   p6_prod [0:4];
  logic                       p6_valid;
  logic [OUT_ADDR_W-1:0]      p6_out_cnt;
  logic [31:0]                p6_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p6_valid   <= 1'b0;
      p6_out_cnt <= '0;
      p6_tuser   <= '0;
      for (int k = 0; k < 5; k++) p6_prod[k] <= '0;
    end else if (pipe_adv) begin
      p6_valid   <= p5_valid;
      p6_out_cnt <= p5_out_cnt;
      p6_tuser   <= p5_tuser;
      for (int k = 0; k < 5; k++)
        p6_prod[k] <= PROD_W'(p5_samp[k]) * PROD_W'(p5_coeff[k]);
    end
  end

  // Stage 7
  localparam int SUM1_W = PROD_W + 1;
  localparam int SUM2_W = SUM1_W + 1;

  logic signed [SUM1_W-1:0]   p7_s01, p7_s23;
  logic signed [PROD_W-1:0]   p7_s4;
  logic                       p7_valid;
  logic [OUT_ADDR_W-1:0]      p7_out_cnt;
  logic [31:0]                p7_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p7_valid   <= 1'b0;
      p7_s01     <= '0;
      p7_s23     <= '0;
      p7_s4      <= '0;
      p7_out_cnt <= '0;
      p7_tuser   <= '0;
    end else if (pipe_adv) begin
      p7_valid   <= p6_valid;
      p7_out_cnt <= p6_out_cnt;
      p7_tuser   <= p6_tuser;
      p7_s01     <= SUM1_W'(p6_prod[0]) + SUM1_W'(p6_prod[1]);
      p7_s23     <= SUM1_W'(p6_prod[2]) + SUM1_W'(p6_prod[3]);
      p7_s4      <= p6_prod[4];
    end
  end

  // Stage 8
  logic signed [SUM2_W-1:0]   p8_s0123;
  logic signed [PROD_W-1:0]   p8_s4;
  logic                       p8_valid;
  logic [OUT_ADDR_W-1:0]      p8_out_cnt;
  logic [31:0]                p8_tuser;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p8_valid   <= 1'b0;
      p8_s0123   <= '0;
      p8_s4      <= '0;
      p8_out_cnt <= '0;
      p8_tuser   <= '0;
    end else if (pipe_adv) begin
      p8_valid   <= p7_valid;
      p8_out_cnt <= p7_out_cnt;
      p8_tuser   <= p7_tuser;
      p8_s0123   <= SUM2_W'(p7_s01) + SUM2_W'(p7_s23);
      p8_s4      <= p7_s4;
    end
  end

  // Stage 9 combinational
  localparam int FINAL_W = SUM2_W + 1;

  logic signed [FINAL_W-1:0] p9_sum_full;
  logic signed [DATA_W-1:0]  p9_sat;

  always_comb begin : sat_round
    automatic logic signed [FINAL_W-1:0] sum_shifted;
    p9_sum_full = FINAL_W'(p8_s0123) + FINAL_W'(p8_s4);
    sum_shifted = (p9_sum_full + FINAL_W'(1 << 14)) >>> 15;

    if (sum_shifted > $signed(FINAL_W'((1 << (DATA_W-1)) - 1)))
      p9_sat = DATA_W'(signed'((1 << (DATA_W-1)) - 1));
    else if (sum_shifted < $signed(FINAL_W'(-(1 << (DATA_W-1)))))
      p9_sat = DATA_W'(signed'(-(1 << (DATA_W-1))));
    else
      p9_sat = DATA_W'(sum_shifted);
  end

  // Output register
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_tdata  <= '0;
      m_tvalid <= 1'b0;
      m_tlast  <= 1'b0;
      m_tuser  <= '0;
    end else if (pipe_adv) begin
      m_tdata  <= p9_sat;
      m_tvalid <= p8_valid;
      m_tlast  <= p8_valid && (p8_out_cnt == OUT_ADDR_W'(OUT_LEN - 1));
      m_tuser  <= p8_tuser;
    end
  end

endmodule : klinear_resampler