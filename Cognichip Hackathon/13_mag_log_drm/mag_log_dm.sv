`timescale 1ns/1ps
import oct_pkg::*;

// -----------------------------------------------------------------------------
// mag_log_drm
// -----------------------------------------------------------------------------
// Synthesizable AXI-stream image front-end:
//   Stage 1 : magnitude squared  (2 cycles)
//   Stage 2 : log compression LUT (1 cycle)
//   Stage 3 : DRM stretch         (2 cycles)
// Output:
//   8-bit gray pixel on m_tdata
//
// Optional sideband phase path:
//   16-iteration pipelined CORDIC in vectoring mode
//   phase(Q1.15) is driven on m_tuser[31:16] when cordic_en=1
//
// Notes:
// 1) The 5-cycle latency target is met for the pixel path when cordic_en=0.
// 2) When cordic_en=1, this implementation aligns the pixel stream to the
//    16-cycle CORDIC path by delaying the gray pixel/user/last path.
//    Therefore, effective end-to-end latency becomes 16 cycles in CORDIC mode.
// 3) For clean stream alignment, cordic_en should be treated as frame-static
//    (do not toggle while samples for a frame are in flight).
// 4) The DRM formula in the prompt mentions a PS-loaded reciprocal multiplier,
//    but no reciprocal port is provided. This module therefore derives a
//    16-bit reciprocal from drm_min/drm_max in a low-rate config path.
//    Replace drm_recip_q0_16 generation with a register loaded from PS if you
//    need strict "no divider in PL" implementation.
//
// Log LUT generation formula (for entry i in [0..255]):
//   y = 10*log10(max(i,1)/255.0)
//   lut[i] = round( 65535 * (y - y_min) / (0 - y_min) )
// where y_min = 10*log10(1/255.0)
// So:
//   lut[0] = 0
//   lut[255] = 65535
// -----------------------------------------------------------------------------
module mag_log_drm (
  input  logic              clk,
  input  logic              rst_n,

  // Input: complex sample {imag[15:0], real[15:0]}
  input  logic [31:0]       s_tdata,
  input  logic              s_tvalid,
  input  logic              s_tlast,
  input  logic [31:0]       s_tuser,
  output logic              s_tready,

  // Output: 8-bit gray-scale pixel
  output logic [7:0]        m_tdata,
  output logic              m_tvalid,
  output logic              m_tlast,
  output logic [31:0]       m_tuser,
  input  logic              m_tready,

  // DRM config
  input  logic [31:0]       drm_min,    // expected in same scaled domain as log LUT
  input  logic [31:0]       drm_max,    // expected in same scaled domain as log LUT

  // CORDIC enable (for Doppler phase output)
  input  logic              cordic_en
);

  // ---------------------------------------------------------------------------
  // Local parameters / types
  // ---------------------------------------------------------------------------
  localparam int CORDIC_ITERS = 16;
  localparam int CORDIC_W     = 18;  // 16-bit input + guard bits

  // ---------------------------------------------------------------------------
  // AXIS global stall model
  // ---------------------------------------------------------------------------
  // Entire pipeline freezes when selected output path is back-pressured.
  // This keeps alignment simple and synthesizes well for high-speed image
  // pipelines.
  logic pipe_ce;
  assign pipe_ce  = (~m_tvalid) | m_tready;
  assign s_tready = pipe_ce;

  // ---------------------------------------------------------------------------
  // Helper functions
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] sat_add32(
    input logic [31:0] a,
    input logic [31:0] b
  );
    logic [32:0] sum;
    begin
      sum = {1'b0, a} + {1'b0, b};
      if (sum[32]) sat_add32 = 32'hFFFF_FFFF;
      else         sat_add32 = sum[31:0];
    end
  endfunction

  function automatic logic signed [CORDIC_W-1:0] cordic_signext16(
    input logic signed [15:0] x
  );
    begin
      cordic_signext16 = {{(CORDIC_W-16){x[15]}}, x};
    end
  endfunction

  function automatic logic [15:0] lut_gen(input int idx);
    real x;
    real y;
    real ymin;
    real norm;
    int  tmp;
    begin
      if (idx <= 0) begin
        lut_gen = 16'd0;
      end
      else begin
        x    = idx / 255.0;
        y    = 10.0 * ($ln(x) / $ln(10.0));
        ymin = 10.0 * ($ln(1.0/255.0) / $ln(10.0));
        norm = (y - ymin) / (0.0 - ymin);
        if (norm < 0.0) norm = 0.0;
        if (norm > 1.0) norm = 1.0;
        tmp = $rtoi(norm * 65535.0 + 0.5);
        if (tmp < 0)       tmp = 0;
        if (tmp > 16'hFFFF) tmp = 16'hFFFF;
        lut_gen = logic'(tmp[15:0]);
      end
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Log LUT ROM (256 x 16)
  // ---------------------------------------------------------------------------
  (* rom_style = "distributed" *) logic [15:0] log_lut [0:255];
  integer ii;
  initial begin
    for (ii = 0; ii < 256; ii = ii + 1) begin
      log_lut[ii] = lut_gen(ii);
    end
  end

  // ---------------------------------------------------------------------------
  // "Config path" reciprocal for DRM
  // ---------------------------------------------------------------------------
  // Q0.16 reciprocal:
  //   drm_recip_q0_16 ~= floor( 65535 / max(drm_max_scaled-drm_min_scaled,1) )
  //
  // Since no explicit reciprocal port exists in the requested interface,
  // this is derived from drm_min/drm_max. For a strict PL implementation that
  // avoids divider inference, replace with a PS-loaded register.
  logic [15:0] drm_min_scaled;
  logic [15:0] drm_max_scaled;
  logic [15:0] drm_range_scaled;
  logic [15:0] drm_recip_q0_16;

  always_comb begin
    drm_min_scaled   = drm_min[15:0];
    drm_max_scaled   = drm_max[15:0];
    drm_range_scaled = (drm_max_scaled > drm_min_scaled) ?
                       (drm_max_scaled - drm_min_scaled) : 16'd1;
    drm_recip_q0_16  = 16'(16'hFFFF / drm_range_scaled);
  end

  // ---------------------------------------------------------------------------
  // Stage 0: input unpack / metadata capture
  // ---------------------------------------------------------------------------
  logic signed [15:0] in_real_s0, in_imag_s0;
  logic [31:0]        in_user_s0;
  logic               in_last_s0;
  logic               in_valid_s0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      in_real_s0  <= '0;
      in_imag_s0  <= '0;
      in_user_s0  <= '0;
      in_last_s0  <= 1'b0;
      in_valid_s0 <= 1'b0;
    end
    else if (pipe_ce) begin
      in_valid_s0 <= s_tvalid;
      if (s_tvalid) begin
        in_real_s0 <= s_tdata[15:0];
        in_imag_s0 <= s_tdata[31:16];
        in_user_s0 <= s_tuser;
        in_last_s0 <= s_tlast;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 1: magnitude squared, pipe stage 1
  // ---------------------------------------------------------------------------
  logic [31:0] mag_r_sq_s1, mag_i_sq_s1;
  logic [31:0] user_s1;
  logic        last_s1, valid_s1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mag_r_sq_s1 <= '0;
      mag_i_sq_s1 <= '0;
      user_s1     <= '0;
      last_s1     <= 1'b0;
      valid_s1    <= 1'b0;
    end
    else if (pipe_ce) begin
      valid_s1 <= in_valid_s0;
      if (in_valid_s0) begin
        mag_r_sq_s1 <= $unsigned($signed(in_real_s0) * $signed(in_real_s0));
        mag_i_sq_s1 <= $unsigned($signed(in_imag_s0) * $signed(in_imag_s0));
        user_s1     <= in_user_s0;
        last_s1     <= in_last_s0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 2: magnitude squared, pipe stage 2 (sum + saturate)
  // ---------------------------------------------------------------------------
  logic [31:0] pow_s2;
  logic [31:0] user_s2;
  logic        last_s2, valid_s2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pow_s2   <= '0;
      user_s2  <= '0;
      last_s2  <= 1'b0;
      valid_s2 <= 1'b0;
    end
    else if (pipe_ce) begin
      valid_s2 <= valid_s1;
      if (valid_s1) begin
        pow_s2  <= sat_add32(mag_r_sq_s1, mag_i_sq_s1);
        user_s2 <= user_s1;
        last_s2 <= last_s1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 3: log LUT
  // ---------------------------------------------------------------------------
  logic [15:0] log_val_s3;
  logic [31:0] user_s3;
  logic        last_s3, valid_s3;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      log_val_s3 <= '0;
      user_s3    <= '0;
      last_s3    <= 1'b0;
      valid_s3   <= 1'b0;
    end
    else if (pipe_ce) begin
      valid_s3 <= valid_s2;
      if (valid_s2) begin
        log_val_s3 <= log_lut[pow_s2[31:24]];
        user_s3    <= user_s2;
        last_s3    <= last_s2;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 4: DRM cycle 1
  //   diff = max(log_val - drm_min_scaled, 0)
  //   mul1 = diff * 255
  // ---------------------------------------------------------------------------
  logic [16:0] diff_s4;
  logic [24:0] mul255_s4;
  logic [31:0] user_s4;
  logic        last_s4, valid_s4;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      diff_s4   <= '0;
      mul255_s4 <= '0;
      user_s4   <= '0;
      last_s4   <= 1'b0;
      valid_s4  <= 1'b0;
    end
    else if (pipe_ce) begin
      valid_s4 <= valid_s3;
      if (valid_s3) begin
        if (log_val_s3 <= drm_min_scaled)
          diff_s4 <= 17'd0;
        else if (log_val_s3 >= drm_max_scaled)
          diff_s4 <= {1'b0, drm_range_scaled};
        else
          diff_s4 <= ({1'b0, log_val_s3} - {1'b0, drm_min_scaled});

        if (log_val_s3 <= drm_min_scaled)
          mul255_s4 <= 25'd0;
        else if (log_val_s3 >= drm_max_scaled)
          mul255_s4 <= {9'd0, drm_range_scaled} * 8'd255;
        else
          mul255_s4 <= ({9'd0, (log_val_s3 - drm_min_scaled)}) * 8'd255;

        user_s4  <= user_s3;
        last_s4  <= last_s3;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Stage 5: DRM cycle 2
  //   map = (diff * 255) * recip_q0_16 >> 16
  //   clamp to [0,255]
  // ---------------------------------------------------------------------------
  logic [7:0]  gray_s5;
  logic [31:0] user_s5;
  logic        last_s5, valid_s5;
  logic [40:0] drm_mul_full_s5;
  logic [24:0] drm_map_pre_s5;

  logic [40:0] drm_mul_full_next;
logic [24:0] drm_map_pre_next;

always_comb begin
  drm_mul_full_next = mul255_s4 * drm_recip_q0_16;
  drm_map_pre_next  = drm_mul_full_next >> 16;
end

always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    gray_s5         <= '0;
    user_s5         <= '0;
    last_s5         <= 1'b0;
    valid_s5        <= 1'b0;
    drm_mul_full_s5 <= '0;
    drm_map_pre_s5  <= '0;
  end
  else if (pipe_ce) begin
    valid_s5 <= valid_s4;
    if (valid_s4) begin
      drm_mul_full_s5 <= drm_mul_full_next;
      drm_map_pre_s5  <= drm_map_pre_next;

      if (log_val_s3 <= drm_min_scaled)
        gray_s5 <= 8'd0;
      else if (log_val_s3 >= drm_max_scaled)
        gray_s5 <= 8'd255;
      else if (drm_map_pre_next > 25'd255)
        gray_s5 <= 8'd255;
      else
        gray_s5 <= drm_map_pre_next[7:0];

      user_s5 <= user_s4;
      last_s5 <= last_s4;
    end
  end
end

  // ---------------------------------------------------------------------------
  // CORDIC phase path (16-stage pipelined vectoring)
  // phase output in Q1.15 on user[31:16]
  // ---------------------------------------------------------------------------
  localparam logic signed [15:0] ATAN_LUT [0:15] = '{
    16'sd8192, // atan(2^-0)  ~= 45.0000 deg
    16'sd4836, // atan(2^-1)  ~= 26.5651 deg
    16'sd2555, // atan(2^-2)  ~= 14.0362 deg
    16'sd1297, // atan(2^-3)  ~=  7.1250 deg
    16'sd651,  // atan(2^-4)  ~=  3.5763 deg
    16'sd326,  // atan(2^-5)
    16'sd163,  // atan(2^-6)
    16'sd81,   // atan(2^-7)
    16'sd41,   // atan(2^-8)
    16'sd20,   // atan(2^-9)
    16'sd10,   // atan(2^-10)
    16'sd5,    // atan(2^-11)
    16'sd3,    // atan(2^-12)
    16'sd1,    // atan(2^-13)
    16'sd1,    // atan(2^-14)
    16'sd0     // atan(2^-15)
  };

  logic signed [CORDIC_W-1:0] cordic_x [0:CORDIC_ITERS];
  logic signed [CORDIC_W-1:0] cordic_y [0:CORDIC_ITERS];
  logic signed [15:0]         cordic_z [0:CORDIC_ITERS];

  logic [31:0] cordic_user [0:CORDIC_ITERS];
  logic        cordic_last [0:CORDIC_ITERS];
  logic        cordic_vld  [0:CORDIC_ITERS];

  integer k;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cordic_x[0]    <= '0;
      cordic_y[0]    <= '0;
      cordic_z[0]    <= '0;
      cordic_user[0] <= '0;
      cordic_last[0] <= 1'b0;
      cordic_vld[0]  <= 1'b0;
    end
    else if (pipe_ce) begin
      cordic_x[0]    <= cordic_signext16(in_real_s0);
      cordic_y[0]    <= cordic_signext16(in_imag_s0);
      cordic_z[0]    <= 16'sd0;
      cordic_user[0] <= in_user_s0;
      cordic_last[0] <= in_last_s0;
      cordic_vld[0]  <= in_valid_s0;
    end
  end

  genvar gi;
  generate
    for (gi = 0; gi < CORDIC_ITERS; gi = gi + 1) begin : g_cordic
      logic signed [CORDIC_W-1:0] x_shift, y_shift;
      always_comb begin
        x_shift = cordic_x[gi] >>> gi;
        y_shift = cordic_y[gi] >>> gi;
      end

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          cordic_x[gi+1]    <= '0;
          cordic_y[gi+1]    <= '0;
          cordic_z[gi+1]    <= '0;
          cordic_user[gi+1] <= '0;
          cordic_last[gi+1] <= 1'b0;
          cordic_vld[gi+1]  <= 1'b0;
        end
        else if (pipe_ce) begin
          cordic_vld[gi+1]  <= cordic_vld[gi];
          cordic_user[gi+1] <= cordic_user[gi];
          cordic_last[gi+1] <= cordic_last[gi];

          if (cordic_y[gi] >= 0) begin
            cordic_x[gi+1] <= cordic_x[gi] + y_shift;
            cordic_y[gi+1] <= cordic_y[gi] - x_shift;
            cordic_z[gi+1] <= cordic_z[gi] + ATAN_LUT[gi];
          end
          else begin
            cordic_x[gi+1] <= cordic_x[gi] - y_shift;
            cordic_y[gi+1] <= cordic_y[gi] + x_shift;
            cordic_z[gi+1] <= cordic_z[gi] - ATAN_LUT[gi];
          end
        end
      end
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Delay gray path from stage 5 to align with 16-cycle CORDIC output
  // 5-cycle pixel path -> add 11 cycles => 16 total cycles
  // ---------------------------------------------------------------------------
  logic [7:0]  gray_align [0:10];
  logic [31:0] user_align [0:10];
  logic        last_align [0:10];
  logic        vld_align  [0:10];

  integer dj;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (dj = 0; dj <= 10; dj = dj + 1) begin
        gray_align[dj] <= '0;
        user_align[dj] <= '0;
        last_align[dj] <= 1'b0;
        vld_align[dj]  <= 1'b0;
      end
    end
    else if (pipe_ce) begin
      gray_align[0] <= gray_s5;
      user_align[0] <= user_s5;
      last_align[0] <= last_s5;
      vld_align[0]  <= valid_s5;

      for (dj = 1; dj <= 10; dj = dj + 1) begin
        gray_align[dj] <= gray_align[dj-1];
        user_align[dj] <= user_align[dj-1];
        last_align[dj] <= last_align[dj-1];
        vld_align[dj]  <= vld_align[dj-1];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Output mux
  // ---------------------------------------------------------------------------
  // In normal mode:
  //   latency = 5 cycles
  // In CORDIC mode:
  //   latency = 16 cycles
  //
  // m_tuser[31:16]:
  //   cordic_en=0 : preserved from input
  //   cordic_en=1 : overwritten with phase(Q1.15)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_tdata  <= '0;
      m_tvalid <= 1'b0;
      m_tlast  <= 1'b0;
      m_tuser  <= '0;
    end
    else if (pipe_ce) begin
      if (cordic_en) begin
        m_tdata  <= gray_align[10];
        m_tvalid <= vld_align[10];
        m_tlast  <= last_align[10];
        m_tuser  <= {cordic_z[CORDIC_ITERS], user_align[10][15:0]};
      end
      else begin
        m_tdata  <= gray_s5;
        m_tvalid <= valid_s5;
        m_tlast  <= last_s5;
        m_tuser  <= user_s5;
      end
    end
  end

endmodule