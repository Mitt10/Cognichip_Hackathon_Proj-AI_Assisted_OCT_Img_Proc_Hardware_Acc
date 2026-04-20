// ============================================================================
// oct_fft.sv
// Memory-based Radix-2^2 DIT FFT Processor
//
// Reference : Tang et al. (2014) "Multimode Memory-Based FFT Processor"
// Target    : Xilinx UltraScale+ @ 250 MHz
// Resources : ~8 BRAM36, 6 DSP48E2, ~500 LUT  (1024-pt mode)
//
// Architecture
// -------------------------------------------------------------------------
//  Ping-pong input buffers  2 x 2048x32-bit (4 BRAM36)
//  Four-bank compute SRAM   4 x  512x32-bit (2 BRAM36)
//  Twiddle ROM              512x36-bit       (1 BRAM36)
//  Butterfly PE             3-cycle pipeline (1 BRAM-read + 2 DSP stages)
//    W_N^k = c-jd (c=cos, d=sin)
//    (a+jb)(c-jd) = (ac+bd) + j(bc-ad)  -> 4 DSP48E2
//  FSM: IDLE -> LOAD -> COMPUTE -> OUTPUT -> IDLE
//  Bit-reversal absorbed into OUTPUT address generation.
// ============================================================================

`ifndef OCT_PKG_SV
  `include "../1_oct_pkg/oct_pkg.sv"
`endif

import oct_pkg::*;

module oct_fft #(
  parameter int MAX_N     = 2048,
  parameter int DATA_W    = 32,
  parameter int TWIDDLE_W = 18
)(
  input  logic        clk,
  input  logic        rst_n,

  input  logic [31:0] s_tdata,
  input  logic        s_tvalid,
  input  logic        s_tlast,
  input  logic [31:0] s_tuser,
  output logic        s_tready,

  output logic [31:0] m_tdata,
  output logic        m_tvalid,
  output logic        m_tlast,
  output logic [31:0] m_tuser,
  input  logic        m_tready,

  input  logic [ 1:0] fft_size   // 00=512, 01=1024, 10=2048
);

  // -------------------------------------------------------------------------
  // Local parameters
  // -------------------------------------------------------------------------
  localparam int HALF_W     = DATA_W / 2;
  localparam int ADDR_W     = $clog2(MAX_N) + 1;  // +1 so fft_n can hold MAX_N itself (not just 0..MAX_N-1)
  localparam int BANK_DEPTH = MAX_N / 4;
  localparam int BANK_AW    = $clog2(BANK_DEPTH);
  localparam int PROD_W     = HALF_W + TWIDDLE_W;
  localparam int TW_SHIFT   = TWIDDLE_W - 1;

  // -------------------------------------------------------------------------
  // Runtime FFT-size decode
  // -------------------------------------------------------------------------
  logic [ADDR_W-1:0] fft_n;
  logic [3:0]        log2_n;
  logic [3:0]        num_passes;

  always_comb begin
    case (fft_size)
      2'b00  : begin fft_n = 12'd512;  log2_n = 4'd9;  num_passes = 4'd5; end
      2'b01  : begin fft_n = 12'd1024; log2_n = 4'd10; num_passes = 4'd5; end
      default: begin fft_n = 12'd2048; log2_n = 4'd11; num_passes = 4'd6; end
    endcase
  end

  logic [ADDR_W-1:0] fft_n_half;
  assign fft_n_half = fft_n >> 1;

  // -------------------------------------------------------------------------
  // Helper functions
  // -------------------------------------------------------------------------
  function automatic logic [1:0] bank_sel_f(input logic [ADDR_W-1:0] idx);
    return idx[1:0];
  endfunction

  function automatic logic [BANK_AW-1:0] bank_addr_f(input logic [ADDR_W-1:0] idx);
    return idx[ADDR_W-1:2];
  endfunction

  function automatic logic [ADDR_W-1:0] bit_rev_f(
    input logic [ADDR_W-1:0] addr,
    input logic [3:0]        width
  );
    logic [ADDR_W-1:0] rev;
    rev = '0;
    for (int i = 0; i < ADDR_W; i++)
      if (i < int'(width)) rev[i] = addr[int'(width)-1-i];
    return rev;
  endfunction

  // -------------------------------------------------------------------------
  // Ping-pong input buffers  (2 x MAX_N x 32-bit  -> 4 BRAM36)
  // -------------------------------------------------------------------------
  (* ram_style = "block" *)
  logic [DATA_W-1:0] pp_buf [0:1][0:MAX_N-1];

  logic              pp_wr_sel;
  logic [ADDR_W-1:0] pp_wr_addr;
  logic [31:0]       frame_tuser;
  logic              frame_ready;

  logic [ADDR_W-1:0] pp_rd_addr;
  logic [DATA_W-1:0] pp_rd_data;

  assign s_tready = 1'b1;

  always_ff @(posedge clk) begin
    if (s_tvalid)
      pp_buf[pp_wr_sel][pp_wr_addr] <= s_tdata;
  end

  always_ff @(posedge clk)
    pp_rd_data <= pp_buf[~pp_wr_sel][pp_rd_addr];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pp_wr_sel   <= 1'b0;
      pp_wr_addr  <= '0;
      frame_tuser <= '0;
      frame_ready <= 1'b0;
    end else begin
      frame_ready <= 1'b0;
      if (s_tvalid) begin
        if (pp_wr_addr == '0) frame_tuser <= s_tuser;
        if (s_tlast) begin
          pp_wr_sel   <= ~pp_wr_sel;
          pp_wr_addr  <= '0;
          frame_ready <= 1'b1;
        end else begin
          pp_wr_addr <= pp_wr_addr + 1'b1;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // Four-bank compute SRAM  (4 x 512 x 32-bit  -> 4 BRAM18 / 2 BRAM36)
  // -------------------------------------------------------------------------
  (* ram_style = "block" *)
  logic [DATA_W-1:0] bank [0:3][0:BANK_DEPTH-1];

  logic [1:0]         bk_ra_sel;
  logic [BANK_AW-1:0] bk_ra_addr;
  logic [DATA_W-1:0]  bk_ra_data;

  logic [1:0]         bk_rb_sel;
  logic [BANK_AW-1:0] bk_rb_addr;
  logic [DATA_W-1:0]  bk_rb_data;

  always_ff @(posedge clk) begin
    bk_ra_data <= bank[bk_ra_sel][bk_ra_addr];
    bk_rb_data <= bank[bk_rb_sel][bk_rb_addr];
  end

  logic [1:0]         bk_w_sel_a,  bk_w_sel_b;
  logic [BANK_AW-1:0] bk_w_addr_a, bk_w_addr_b;
  logic [DATA_W-1:0]  bk_w_data_a, bk_w_data_b;
  logic               bk_w_en_a,   bk_w_en_b;

  always_ff @(posedge clk) begin
    if (bk_w_en_a) bank[bk_w_sel_a][bk_w_addr_a] <= bk_w_data_a;
    if (bk_w_en_b) bank[bk_w_sel_b][bk_w_addr_b] <= bk_w_data_b;
  end

  // -------------------------------------------------------------------------
  // Twiddle ROM  (512 x 36-bit  -> 1 BRAM36)
  // -------------------------------------------------------------------------
  (* rom_style = "block" *)
  logic [35:0] twiddle_rom [0:BANK_DEPTH-1];

  initial $readmemh("twiddle_rom.hex", twiddle_rom);

  logic [BANK_AW-1:0]   tw_addr;
  logic [35:0]           tw_rdata;
  logic [TWIDDLE_W-1:0]  tw_cos_r, tw_sin_r;

  always_ff @(posedge clk)
    tw_rdata <= twiddle_rom[tw_addr];

  assign tw_cos_r = tw_rdata[35:18];
  assign tw_sin_r = tw_rdata[17:0];

  // -------------------------------------------------------------------------
  // FSM state and counters
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    S_IDLE    = 2'd0,
    S_LOAD    = 2'd1,
    S_COMPUTE = 2'd2,
    S_OUTPUT  = 2'd3
  } fsm_t;

  fsm_t              state;
  logic [ADDR_W-1:0] ld_cnt;
  logic [ADDR_W-2:0] bf_cnt;
  logic [3:0]        pass_cnt;
  logic [ADDR_W-1:0] out_cnt;

  // -------------------------------------------------------------------------
  // Butterfly address and twiddle address (combinational)
  // -------------------------------------------------------------------------
  logic [ADDR_W-1:0]  bf_top_idx, bf_bot_idx;
  logic [ADDR_W-1:0]  bf_stride;
  logic [BANK_AW-1:0] tw_k;

  always_comb begin
    bf_top_idx = {1'b0, bf_cnt};
    bf_bot_idx = fft_n_half | {1'b0, bf_cnt};

    bf_stride = fft_n >> (2*pass_cnt + 2);
    if (bf_stride == '0) bf_stride = 1;

    tw_k = BANK_AW'( ({1'b0, bf_cnt} & (bf_stride - 1'b1)) << (2*pass_cnt) );

    case (fft_size)
      2'b00  : tw_k = BANK_AW'(tw_k << 2);
      2'b01  : tw_k = BANK_AW'(tw_k << 1);
      default: tw_k = tw_k;
    endcase
  end

  logic [ADDR_W-1:0] out_brev;
  assign out_brev = bit_rev_f(out_cnt, log2_n);

  // -------------------------------------------------------------------------
  // Bank read address mux  (single always_comb)
  // -------------------------------------------------------------------------
  always_comb begin
    bk_ra_sel  = 2'd0;
    bk_ra_addr = '0;
    bk_rb_sel  = 2'd0;
    bk_rb_addr = '0;
    tw_addr    = '0;

    case (state)
      S_COMPUTE : begin
        bk_ra_sel  = bank_sel_f(bf_top_idx);
        bk_ra_addr = bank_addr_f(bf_top_idx);
        bk_rb_sel  = bank_sel_f(bf_bot_idx);
        bk_rb_addr = bank_addr_f(bf_bot_idx);
        tw_addr    = tw_k;
      end
      S_OUTPUT : begin
        bk_ra_sel  = bank_sel_f(out_brev);
        bk_ra_addr = bank_addr_f(out_brev);
      end
      default : ;
    endcase
  end

  // -------------------------------------------------------------------------
  // PE write-back address pipeline  (depth=3: BRAM-rd + s1 FF + s2 FF)
  // -------------------------------------------------------------------------
  logic [2:0]        pe_vld_pipe;
  logic [ADDR_W-1:0] pe_top_pipe [0:2];
  logic [ADDR_W-1:0] pe_bot_pipe [0:2];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pe_vld_pipe <= '0;
      for (int i = 0; i < 3; i++) begin
        pe_top_pipe[i] <= '0;
        pe_bot_pipe[i] <= '0;
      end
    end else begin
      pe_vld_pipe[0] <= (state == S_COMPUTE);
      pe_top_pipe[0] <= bf_top_idx;
      pe_bot_pipe[0] <= bf_bot_idx;
      for (int i = 1; i < 3; i++) begin
        pe_vld_pipe[i] <= pe_vld_pipe[i-1];
        pe_top_pipe[i] <= pe_top_pipe[i-1];
        pe_bot_pipe[i] <= pe_bot_pipe[i-1];
      end
    end
  end

  // -------------------------------------------------------------------------
  // Butterfly PE stage 0  (combinational from BRAM output)
  // -------------------------------------------------------------------------
  logic signed [HALF_W-1:0] pe_ar, pe_ai, pe_br, pe_bi;
  assign pe_ar = signed'(bk_ra_data[HALF_W-1:0]);
  assign pe_ai = signed'(bk_ra_data[DATA_W-1:HALF_W]);
  assign pe_br = signed'(bk_rb_data[HALF_W-1:0]);
  assign pe_bi = signed'(bk_rb_data[DATA_W-1:HALF_W]);

  logic signed [HALF_W:0] bf_sum_r, bf_sum_i, bf_dif_r, bf_dif_i;
  assign bf_sum_r = {pe_ar[HALF_W-1], pe_ar} + {pe_br[HALF_W-1], pe_br};
  assign bf_sum_i = {pe_ai[HALF_W-1], pe_ai} + {pe_bi[HALF_W-1], pe_bi};
  assign bf_dif_r = {pe_ar[HALF_W-1], pe_ar} - {pe_br[HALF_W-1], pe_br};
  assign bf_dif_i = {pe_ai[HALF_W-1], pe_ai} - {pe_bi[HALF_W-1], pe_bi};

  logic signed [HALF_W-1:0] sum_r, sum_i, dif_r, dif_i;
  assign sum_r = bf_sum_r[HALF_W:1];
  assign sum_i = bf_sum_i[HALF_W:1];
  assign dif_r = bf_dif_r[HALF_W:1];
  assign dif_i = bf_dif_i[HALF_W:1];

  // -------------------------------------------------------------------------
  // Butterfly PE stage 1  (register butterfly + twiddle)
  // -------------------------------------------------------------------------
  logic signed [HALF_W-1:0]    s1_sum_r, s1_sum_i;
  logic signed [HALF_W-1:0]    s1_dif_r, s1_dif_i;
  logic signed [TWIDDLE_W-1:0] s1_tw_c,  s1_tw_d;

  always_ff @(posedge clk) begin
    s1_sum_r <= sum_r;
    s1_sum_i <= sum_i;
    s1_dif_r <= dif_r;
    s1_dif_i <= dif_i;
    s1_tw_c  <= signed'(tw_cos_r);
    s1_tw_d  <= signed'(tw_sin_r);
  end

  // -------------------------------------------------------------------------
  // Butterfly PE stage 2  (4 x DSP48E2 multiply)
  // -------------------------------------------------------------------------
  (* use_dsp = "yes" *) logic signed [PROD_W-1:0] dsp_ac;
  (* use_dsp = "yes" *) logic signed [PROD_W-1:0] dsp_bd;
  (* use_dsp = "yes" *) logic signed [PROD_W-1:0] dsp_bc;
  (* use_dsp = "yes" *) logic signed [PROD_W-1:0] dsp_ad;
  logic signed [HALF_W-1:0] s2_sum_r, s2_sum_i;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dsp_ac <= '0; dsp_bd <= '0; dsp_bc <= '0; dsp_ad <= '0;
    end else begin
      dsp_ac   <= PROD_W'(s1_dif_r) * PROD_W'(s1_tw_c);
      dsp_bd   <= PROD_W'(s1_dif_i) * PROD_W'(s1_tw_d);
      dsp_bc   <= PROD_W'(s1_dif_i) * PROD_W'(s1_tw_c);
      dsp_ad   <= PROD_W'(s1_dif_r) * PROD_W'(s1_tw_d);
      s2_sum_r <= s1_sum_r;
      s2_sum_i <= s1_sum_i;
    end
  end

  // (a+jb)(c-jd) = (ac+bd) + j(bc-ad)
  logic signed [HALF_W-1:0] pe_out_r, pe_out_i;
  assign pe_out_r = (dsp_ac + dsp_bd) >>> TW_SHIFT;
  assign pe_out_i = (dsp_bc - dsp_ad) >>> TW_SHIFT;

  // -------------------------------------------------------------------------
  // LOAD path 1-cycle delayed write  (ping-pong -> compute bank)
  // -------------------------------------------------------------------------
  logic [ADDR_W-1:0] ld_dly;
  logic              ld_dly_vld;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ld_dly     <= '0;
      ld_dly_vld <= 1'b0;
    end else begin
      ld_dly     <= ld_cnt;
      ld_dly_vld <= (state == S_LOAD);
    end
  end

  // -------------------------------------------------------------------------
  // Bank write mux  (single always_comb)
  // Priority: LOAD (mutually exclusive with COMPUTE in practice)
  // -------------------------------------------------------------------------
  always_comb begin
    bk_w_en_a   = 1'b0;
    bk_w_sel_a  = 2'd0;
    bk_w_addr_a = '0;
    bk_w_data_a = '0;
    bk_w_en_b   = 1'b0;
    bk_w_sel_b  = 2'd0;
    bk_w_addr_b = '0;
    bk_w_data_b = '0;

    if (ld_dly_vld) begin
      bk_w_en_a   = 1'b1;
      bk_w_sel_a  = bank_sel_f(ld_dly);
      bk_w_addr_a = bank_addr_f(ld_dly);
      bk_w_data_a = pp_rd_data;
    end else if (pe_vld_pipe[2]) begin
      bk_w_en_a   = 1'b1;
      bk_w_sel_a  = bank_sel_f(pe_top_pipe[2]);
      bk_w_addr_a = bank_addr_f(pe_top_pipe[2]);
      bk_w_data_a = {s2_sum_i, s2_sum_r};
      bk_w_en_b   = 1'b1;
      bk_w_sel_b  = bank_sel_f(pe_bot_pipe[2]);
      bk_w_addr_b = bank_addr_f(pe_bot_pipe[2]);
      bk_w_data_b = {pe_out_i, pe_out_r};
    end
  end

  // -------------------------------------------------------------------------
  // FSM  (single always_ff for state, counters, and AXI-S master outputs)
  // -------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= S_IDLE;
      ld_cnt     <= '0;
      bf_cnt     <= '0;
      pass_cnt   <= '0;
      out_cnt    <= '0;
      pp_rd_addr <= '0;
      m_tvalid   <= 1'b0;
      m_tlast    <= 1'b0;
      m_tdata    <= '0;
      m_tuser    <= '0;
    end else begin

      case (state)

        // -- IDLE -----------------------------------------------------------
        S_IDLE : begin
          m_tvalid <= 1'b0;
          m_tlast  <= 1'b0;
          ld_cnt   <= '0;
          bf_cnt   <= '0;
          pass_cnt <= '0;
          out_cnt  <= '0;
          if (frame_ready) begin
            pp_rd_addr <= '0;
            state      <= S_LOAD;
          end
        end

        // -- LOAD : ping-pong buffer -> compute banks -----------------------
        // Issue pp_rd_addr = ld_cnt; ld_dly logic handles delayed bank write.
        S_LOAD : begin
          pp_rd_addr <= ld_cnt;
          if (ld_cnt == fft_n - 1) begin
            ld_cnt <= '0;
            state  <= S_COMPUTE;
          end else begin
            ld_cnt <= ld_cnt + 1'b1;
          end
        end

        // -- COMPUTE : radix-2 butterfly passes ----------------------------
        S_COMPUTE : begin
          if (bf_cnt == fft_n_half[ADDR_W-2:0] - 1) begin
            bf_cnt <= '0;
            if (pass_cnt == num_passes - 1) begin
              pass_cnt <= '0;
              state    <= S_OUTPUT;
              out_cnt  <= '0;
            end else begin
              pass_cnt <= pass_cnt + 1'b1;
            end
          end else begin
            bf_cnt <= bf_cnt + 1'b1;
          end
        end

        // -- OUTPUT : stream N results in natural order --------------------
        // Read address (bit-reversed out_cnt) driven combinationally.
        // BRAM latency = 1 cycle: issue at out_cnt, present data at out_cnt+1.
        S_OUTPUT : begin
          if (!m_tvalid || m_tready) begin
            if (out_cnt >= 1) begin
              m_tdata  <= bk_ra_data;
              m_tvalid <= 1'b1;
              m_tuser  <= frame_tuser;
              m_tlast  <= (out_cnt == fft_n);
            end
            if (out_cnt == fft_n) begin
              state   <= S_IDLE;
              out_cnt <= '0;
            end else begin
              out_cnt <= out_cnt + 1'b1;
            end
          end
        end

        default : state <= S_IDLE;

      endcase
    end
  end

endmodule : oct_fft
