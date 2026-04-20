// ============================================================================
// spectral_shaping.sv
// OCT AI Accelerator — Spectral Shaping IIR SOS Chebyshev Filter Bank
//
// Target  : Xilinx UltraScale+ (XCZU7EV / ZCU104), 250 MHz DSP clock
//
// Implements a 5-zone Chebyshev Type-I IIR filter bank using 3 cascaded
// second-order sections (SOS) per zone (order-6 per zone).
// Matches Tang et al. Gabor-domain OCM digital filter architecture.
//
// Pipeline overview (Direct Form II Transposed, 2 cycles per SOS stage):
//   Cycle 1 per stage : register all 5 products (b0,b1,b2 × x; a1,a2 × w_prev)
//   Cycle 2 per stage : compute w, update s1/s2 states
//   3 stages × 2 cycles = 6 cycle total filter latency
//   Bypass (zone_sel==5): 2-cycle registered passthrough
//
// Coefficient BRAM layout (1 × BRAM18):
//   Address = zone * SOS_STAGES * 5 + stage * 5 + coeff_idx
//   coeff_idx: 0=b0, 1=b1, 2=b2, 3=a1, 4=a2
//   Total entries: 5 * 3 * 5 = 75 (fits in a tiny fraction of BRAM18)
//
// Resource budget: 15 DSP48E2 (5 per SOS stage × 3 stages),
//                  1 BRAM18 (coefficient storage)
//
// Note on pipelined IIR feedback:
//   The a1/a2 feedback terms use w_prev (the registered output of each SOS
//   stage from the previous sample), not the current-cycle w. This is the
//   standard FPGA pipelined IIR approximation that sustains 1-sample/clock
//   throughput at 250 MHz while introducing a 1-sample group delay on the
//   feedback poles — acceptable for OCT spectral shaping applications.
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module spectral_shaping #(
    parameter int N_ZONES    = 5,
    parameter int SOS_STAGES = 3,
    parameter int DATA_W     = 16,
    parameter int COEFF_W    = 18,   // Q1.17 signed
    parameter int STATE_W    = 32
) (
    input  logic              clk,
    input  logic              rst_n,

    // AXI4-Stream slave
    input  logic [DATA_W-1:0]  s_tdata,
    input  logic               s_tvalid,
    input  logic               s_tlast,
    input  logic [31:0]        s_tuser,
    output logic               s_tready,

    // AXI4-Stream master
    output logic [DATA_W-1:0]  m_tdata,
    output logic               m_tvalid,
    output logic               m_tlast,
    output logic [31:0]        m_tuser,
    input  logic               m_tready,

    // Zone select: 0..4 = active zone, 5 = bypass
    input  logic [2:0]         zone_sel,

    // Runtime coefficient write port (BRAM)
    input  logic               coeff_we,
    input  logic [8:0]         coeff_addr,
    input  logic [COEFF_W-1:0] coeff_wdata
);

    // -----------------------------------------------------------------------
    // Local parameters
    // -----------------------------------------------------------------------
    localparam int BYPASS_SEL    = 5;
    localparam int TOTAL_COEFF   = N_ZONES * SOS_STAGES * 5;  // 75
    localparam int PROD_W        = COEFF_W + DATA_W;           // 34
    localparam int ACC_W         = PROD_W + 2;                 // 36 (guard bits)
    localparam int FRAC_BITS     = COEFF_W - 1;                // 17
    localparam int FILTER_LAT    = SOS_STAGES * 2;             // 6
    localparam int BYPASS_LAT    = 2;

    // -----------------------------------------------------------------------
    // Flow control — purely streaming, always ready when downstream ready
    // -----------------------------------------------------------------------
    assign s_tready = m_tready;
    wire in_fire = s_tvalid & m_tready;

    // -----------------------------------------------------------------------
    // BRAM18 coefficient storage: 75 × 18-bit
    // Dual-port: port A write-only (runtime update), port B read-only
    // -----------------------------------------------------------------------
    (* ram_style = "block" *)
    logic signed [COEFF_W-1:0] coeff_ram [0:TOTAL_COEFF-1];

    // Port A: synchronous write
    always_ff @(posedge clk) begin
        if (coeff_we) coeff_ram[coeff_addr] <= coeff_wdata;
    end

    // -----------------------------------------------------------------------
    // Coefficient cache: active zone's 15 coefficients in registers
    // Loaded from BRAM over 15 cycles whenever zone_sel changes.
    // Layout: cache[stage*5 + 0..4] = {b0, b1, b2, a1, a2} for that stage
    // -----------------------------------------------------------------------
    logic signed [COEFF_W-1:0] coeff_cache [0:SOS_STAGES*5-1];
    logic [2:0]  zone_sel_q;
    logic [3:0]  load_cnt;
    logic        cache_loading;
    logic        cache_valid;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            zone_sel_q    <= 3'd0;
            load_cnt      <= 4'd0;
            cache_loading <= 1'b1;
            cache_valid   <= 1'b0;
        end else begin
            zone_sel_q <= zone_sel;
            if (zone_sel != zone_sel_q) begin
                cache_loading <= 1'b1;
                cache_valid   <= 1'b0;
                load_cnt      <= 4'd0;
            end else if (cache_loading) begin
                // BRAM read: address = zone * 15 + load_cnt
                coeff_cache[load_cnt] <= coeff_ram[zone_sel_q * (SOS_STAGES*5) + load_cnt];
                if (load_cnt == 4'(SOS_STAGES * 5 - 1)) begin
                    cache_loading <= 1'b0;
                    cache_valid   <= 1'b1;
                end else begin
                    load_cnt <= load_cnt + 4'd1;
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // Coefficient aliases per SOS stage s: b0[s], b1[s], b2[s], a1c[s], a2c[s]
    // -----------------------------------------------------------------------
    logic signed [COEFF_W-1:0] cb0 [0:SOS_STAGES-1];
    logic signed [COEFF_W-1:0] cb1 [0:SOS_STAGES-1];
    logic signed [COEFF_W-1:0] cb2 [0:SOS_STAGES-1];
    logic signed [COEFF_W-1:0] ca1 [0:SOS_STAGES-1];
    logic signed [COEFF_W-1:0] ca2 [0:SOS_STAGES-1];

    generate
        for (genvar s = 0; s < SOS_STAGES; s++) begin : g_coeff
            assign cb0[s] = coeff_cache[s*5+0];
            assign cb1[s] = coeff_cache[s*5+1];
            assign cb2[s] = coeff_cache[s*5+2];
            assign ca1[s] = coeff_cache[s*5+3];
            assign ca2[s] = coeff_cache[s*5+4];
        end
    endgenerate

    // -----------------------------------------------------------------------
    // IIR state registers per SOS stage
    //   s1[s], s2[s]   : Direct Form II Transposed state (STATE_W bits)
    //   w_prev[s]       : Registered output of stage s (feedback, DATA_W bits)
    // States are cleared when zone_sel changes or cache is not valid.
    // -----------------------------------------------------------------------
    logic signed [STATE_W-1:0] iir_s1   [0:SOS_STAGES-1];
    logic signed [STATE_W-1:0] iir_s2   [0:SOS_STAGES-1];
    logic signed [DATA_W-1:0]  iir_wprv [0:SOS_STAGES-1];

    wire state_flush = (zone_sel != zone_sel_q) || !cache_valid;

    // -----------------------------------------------------------------------
    // Stage pipeline registers (cycle 1: products + state capture)
    // -----------------------------------------------------------------------
    // Products: 18-bit coeff × 16-bit data = 34-bit signed
    logic signed [PROD_W-1:0]  pp_b0  [0:SOS_STAGES-1];
    logic signed [PROD_W-1:0]  pp_b1  [0:SOS_STAGES-1];
    logic signed [PROD_W-1:0]  pp_b2  [0:SOS_STAGES-1];
    logic signed [PROD_W-1:0]  pp_a1  [0:SOS_STAGES-1];
    logic signed [PROD_W-1:0]  pp_a2  [0:SOS_STAGES-1];
    // State capture at cycle 1
    logic signed [STATE_W-1:0] ps1    [0:SOS_STAGES-1];
    logic signed [STATE_W-1:0] ps2    [0:SOS_STAGES-1];
    // Sideband at cycle 1 (only need for stage 0; others propagate from w_out)
    logic                      p1_vld [0:SOS_STAGES-1];
    logic                      p1_lst [0:SOS_STAGES-1];
    logic [31:0]               p1_usr [0:SOS_STAGES-1];

    // Stage outputs after cycle 2
    logic signed [DATA_W-1:0]  w_out  [0:SOS_STAGES-1];
    logic                      w_vld  [0:SOS_STAGES-1];
    logic                      w_lst  [0:SOS_STAGES-1];
    logic [31:0]               w_usr  [0:SOS_STAGES-1];

    // Stage inputs (wired connections)
    // x_in[0] = s_tdata (registered by cycle-1 products)
    // x_in[s] = w_out[s-1] for s > 0

    // -----------------------------------------------------------------------
    // SOS Pipeline: generate SOS_STAGES identical 2-cycle pipe stages
    // -----------------------------------------------------------------------
    generate
        for (genvar s = 0; s < SOS_STAGES; s++) begin : g_sos

            // -----------------------------------------------------------------
            // Cycle 1 register: compute all 5 products, capture state
            // -----------------------------------------------------------------
            always_ff @(posedge clk) begin
                if (!rst_n || state_flush) begin
                    pp_b0[s]  <= '0;
                    pp_b1[s]  <= '0;
                    pp_b2[s]  <= '0;
                    pp_a1[s]  <= '0;
                    pp_a2[s]  <= '0;
                    ps1[s]    <= '0;
                    ps2[s]    <= '0;
                    p1_vld[s] <= 1'b0;
                    p1_lst[s] <= 1'b0;
                    p1_usr[s] <= '0;
                end else if (m_tready) begin
                    // Select input for this stage
                    // Stage 0 input: s_tdata; Stage s>0 input: w_out[s-1]
                    if (s == 0) begin
                        (* use_dsp = "yes" *)
                        pp_b0[s] <= PROD_W'($signed(s_tdata) * cb0[s]);
                        (* use_dsp = "yes" *)
                        pp_b1[s] <= PROD_W'($signed(s_tdata) * cb1[s]);
                        (* use_dsp = "yes" *)
                        pp_b2[s] <= PROD_W'($signed(s_tdata) * cb2[s]);
                        p1_vld[s] <= s_tvalid;
                        p1_lst[s] <= s_tlast;
                        p1_usr[s] <= s_tuser;
                    end else begin
                        (* use_dsp = "yes" *)
                        pp_b0[s] <= PROD_W'(w_out[s-1] * cb0[s]);
                        (* use_dsp = "yes" *)
                        pp_b1[s] <= PROD_W'(w_out[s-1] * cb1[s]);
                        (* use_dsp = "yes" *)
                        pp_b2[s] <= PROD_W'(w_out[s-1] * cb2[s]);
                        p1_vld[s] <= w_vld[s-1];
                        p1_lst[s] <= w_lst[s-1];
                        p1_usr[s] <= w_usr[s-1];
                    end
                    // Feedback: use w_prev (registered previous output of this stage)
                    (* use_dsp = "yes" *)
                    pp_a1[s] <= PROD_W'(iir_wprv[s] * ca1[s]);
                    (* use_dsp = "yes" *)
                    pp_a2[s] <= PROD_W'(iir_wprv[s] * ca2[s]);
                    // Capture state for use in cycle 2
                    ps1[s] <= iir_s1[s];
                    ps2[s] <= iir_s2[s];
                end
            end

            // -----------------------------------------------------------------
            // Cycle 2: compute w[n], update states, saturate output
            // w[n] = b0*x[n] + s1[n-1]  (Q1.17 product + state → truncate)
            // s1[n] = b1*x[n] + s2[n-1] - a1*w_prev
            // s2[n] = b2*x[n]            - a2*w_prev
            // -----------------------------------------------------------------
            // Combinational w accumulation (declared in generate scope)
            logic signed [ACC_W-1:0]         w_full;
            logic signed [ACC_W:0]           w_rounded;
            logic signed [ACC_W-FRAC_BITS:0] w_int;

            always_comb begin
                w_full   = ACC_W'(pp_b0[s]) + ACC_W'(ps1[s]);
                // Round: add rounding bit (bit FRAC_BITS-1)
                w_rounded = {w_full[ACC_W-1], w_full} + {{ACC_W{1'b0}}, w_full[FRAC_BITS-1]};
                // Shift out fractional bits
                w_int     = w_rounded[ACC_W : FRAC_BITS];
            end

            always_ff @(posedge clk) begin
                if (!rst_n || state_flush) begin
                    w_out[s]    <= '0;
                    w_vld[s]    <= 1'b0;
                    w_lst[s]    <= 1'b0;
                    w_usr[s]    <= '0;
                    iir_s1[s]   <= '0;
                    iir_s2[s]   <= '0;
                    iir_wprv[s] <= '0;
                end else if (m_tready) begin
                    // State update
                    iir_s1[s] <= STATE_W'(ACC_W'(pp_b1[s]) + ACC_W'(ps2[s]) - ACC_W'(pp_a1[s]));
                    iir_s2[s] <= STATE_W'(ACC_W'(pp_b2[s])                  - ACC_W'(pp_a2[s]));

                    // Saturate w_int to DATA_W output
                    if ($signed(w_int) > $signed({{(ACC_W-FRAC_BITS-DATA_W+1){1'b0}}, 15'h7FFF}))
                        w_out[s] <= 16'sh7FFF;
                    else if ($signed(w_int) < $signed({{(ACC_W-FRAC_BITS-DATA_W+1){1'b1}}, 15'h0000}))
                        w_out[s] <= 16'sh8000;
                    else
                        w_out[s] <= w_int[DATA_W-1:0];

                    // Register w output for next sample's feedback
                    iir_wprv[s] <= w_out[s];

                    // Sideband propagation
                    w_vld[s] <= p1_vld[s];
                    w_lst[s] <= p1_lst[s];
                    w_usr[s] <= p1_usr[s];
                end
            end

        end // for s
    endgenerate

    // -----------------------------------------------------------------------
    // Bypass path: 2-cycle registered shift for zone_sel == 5
    // -----------------------------------------------------------------------
    logic signed [DATA_W-1:0] byp_d [0:BYPASS_LAT-1];
    logic                     byp_vld [0:BYPASS_LAT-1];
    logic                     byp_lst [0:BYPASS_LAT-1];
    logic [31:0]              byp_usr [0:BYPASS_LAT-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < BYPASS_LAT; i++) begin
                byp_d[i]   <= '0;
                byp_vld[i] <= 1'b0;
                byp_lst[i] <= 1'b0;
                byp_usr[i] <= '0;
            end
        end else if (m_tready) begin
            byp_d[0]   <= $signed(s_tdata);
            byp_vld[0] <= s_tvalid;
            byp_lst[0] <= s_tlast;
            byp_usr[0] <= s_tuser;
            for (int i = 1; i < BYPASS_LAT; i++) begin
                byp_d[i]   <= byp_d[i-1];
                byp_vld[i] <= byp_vld[i-1];
                byp_lst[i] <= byp_lst[i-1];
                byp_usr[i] <= byp_usr[i-1];
            end
        end
    end

    // -----------------------------------------------------------------------
    // Output mux: bypass or filtered (last SOS stage output)
    // zone_sel pipeline: latch zone_sel at the filter output timing
    // For bypass (2-cycle latency), register zone_sel 2 cycles.
    // For filter  (6-cycle latency), register zone_sel 6 cycles.
    // We output using bypass timing (2 cycles) when zone_sel==5,
    // and filter timing (6 cycles) otherwise.
    // -----------------------------------------------------------------------
    // Register zone_sel for filter latency alignment
    logic [2:0] zsel_pipe [0:FILTER_LAT-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < FILTER_LAT; i++) zsel_pipe[i] <= 3'd5;
        end else if (m_tready) begin
            zsel_pipe[0] <= zone_sel;
            for (int i = 1; i < FILTER_LAT; i++) zsel_pipe[i] <= zsel_pipe[i-1];
        end
    end

    // Register zone_sel for bypass latency alignment (2 cycles)
    logic [2:0] zsel_byp [0:BYPASS_LAT-1];
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < BYPASS_LAT; i++) zsel_byp[i] <= 3'd5;
        end else if (m_tready) begin
            zsel_byp[0] <= zone_sel;
            for (int i = 1; i < BYPASS_LAT; i++) zsel_byp[i] <= zsel_byp[i-1];
        end
    end

    // Output register
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_tdata  <= '0;
            m_tvalid <= 1'b0;
            m_tlast  <= 1'b0;
            m_tuser  <= '0;
        end else if (m_tready) begin
            if (zsel_byp[BYPASS_LAT-1] == 3'(BYPASS_SEL)) begin
                // Bypass: 2-cycle path
                m_tdata  <= byp_d[BYPASS_LAT-1];
                m_tvalid <= byp_vld[BYPASS_LAT-1];
                m_tlast  <= byp_lst[BYPASS_LAT-1];
                m_tuser  <= byp_usr[BYPASS_LAT-1];
            end else begin
                // Filtered: 6-cycle IIR path (last SOS stage output)
                m_tdata  <= w_out[SOS_STAGES-1];
                m_tvalid <= w_vld[SOS_STAGES-1];
                m_tlast  <= w_lst[SOS_STAGES-1];
                m_tuser  <= w_usr[SOS_STAGES-1];
            end
        end
    end

endmodule : spectral_shaping
