// ============================================================================
// ss_demod_fir.sv
// OCT AI Accelerator — SS-OCT AOM Carrier Demodulation FIR (40-tap LPF)
//
// Target  : Xilinx UltraScale+ (XCZU7EV / ZCU104), 250 MHz DSP clock
//
// Implements a 40-tap linear-phase FIR low-pass filter to strip the AOM
// carrier frequency in SS-OCT mode (Desjardins et al., Real-Time FPGA for
// OFDI). When tuser.mode == 2'b00 (SD-OCT), the input is passed through
// with a matching 20-cycle pipeline latency.
//
// Architecture:
//   - Symmetric 40-tap FIR → 20 unique coefficients
//   - Pre-addition of symmetric tap pairs before multiply: 20 DSP48E2s
//   - SRL32-based 40-sample delay line (srl_style = "srl_reg")
//   - 36-bit signed accumulator, truncated + rounded to 16-bit output
//   - Pipeline latency: 20 cycles (pre-add reg + mult reg + 18 accum stages)
//   - Bypass path: 20-stage shift register matches FIR latency for SD-OCT
//
// Coefficient notes (Hamming-windowed sinc, cutoff = 0.4×Fs, Q1.15):
//   20 unique values (h[0]..h[19]), h[39-k] = h[k] (even symmetric)
//   Representative placeholder values below — replace with your computed set:
//   h[ 0] =  16'sh0012  //  0.000549  (near-zero outer tap)
//   h[ 1] =  16'sh0000  //  0.000000
//   h[ 2] = -16'sh0041  // -0.001978
//   h[ 3] = -16'sh007A  // -0.003726
//   h[ 4] = -16'sh0047  // -0.002167
//   h[ 5] =  16'sh0085  //  0.004028
//   h[ 6] =  16'sh018C  //  0.011932
//   h[ 7] =  16'sh01F6  //  0.015076
//   h[ 8] =  16'sh003E  //  0.001892
//   h[ 9] = -16'sh031A  // -0.024170
//   h[10] = -16'sh0705  // -0.054840
//   h[11] = -16'sh0A3C  // -0.079651
//   h[12] = -16'sh0718  // -0.055664
//   h[13] =  16'sh0640  //  0.049316
//   h[14] =  16'sh1B9E  //  0.214783
//   h[15] =  16'sh30D4  //  0.381409
//   h[16] =  16'sh3C29  //  0.469574
//   h[17] =  16'sh3C29  //  0.469574  (centre pair — symmetric)
//   h[18] =  16'sh30D4  //  0.381409
//   h[19] =  16'sh1B9E  //  0.214783
//   (user to replace above with exact Hamming-windowed sinc values)
//
// Resource budget: ≤ 20 DSP48E2, ≤ 3 BRAM18, delay line in SRL32 LUTRAM
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module ss_demod_fir #(
    parameter int TAPS    = 40,   // Must be 40 (even-symmetric)
    parameter int DATA_W  = 16,   // Input width (signed)
    parameter int COEFF_W = 16,   // Coefficient width (signed Q1.15)
    parameter int OUT_W   = 16    // Output width
) (
    input  logic              clk,
    input  logic              rst_n,

    // AXI4-Stream slave
    input  logic [15:0]       s_tdata,
    input  logic              s_tvalid,
    input  logic              s_tlast,
    input  logic [31:0]       s_tuser,
    output logic              s_tready,

    // AXI4-Stream master
    output logic [15:0]       m_tdata,
    output logic              m_tvalid,
    output logic              m_tlast,
    output logic [31:0]       m_tuser,
    input  logic              m_tready
);

    // -----------------------------------------------------------------------
    // Local parameters
    // -----------------------------------------------------------------------
    localparam int HALF_TAPS  = TAPS / 2;          // 20
    localparam int ACCUM_W    = 36;                 // accumulator width
    localparam int LATENCY    = 20;                 // pipeline latency (cycles)

    // -----------------------------------------------------------------------
    // Hamming-windowed sinc coefficients, cutoff = 0.4×Fs (Q1.15 signed)
    // Replace placeholder values with your computed set.
    // h[0] is the outermost tap; h[19] is the inner-centre tap.
    // -----------------------------------------------------------------------
    localparam logic signed [COEFF_W-1:0] COEFF [0:HALF_TAPS-1] = '{
        16'sh0012,   // h[ 0]  outer
        16'sh0000,   // h[ 1]
       -16'sh0041,   // h[ 2]
       -16'sh007A,   // h[ 3]
       -16'sh0047,   // h[ 4]
        16'sh0085,   // h[ 5]
        16'sh018C,   // h[ 6]
        16'sh01F6,   // h[ 7]
        16'sh003E,   // h[ 8]
       -16'sh031A,   // h[ 9]
       -16'sh0705,   // h[10]
       -16'sh0A3C,   // h[11]
       -16'sh0718,   // h[12]
        16'sh0640,   // h[13]
        16'sh1B9E,   // h[14]
        16'sh30D4,   // h[15]
        16'sh3C29,   // h[16]
        16'sh3C29,   // h[17]
        16'sh30D4,   // h[18]
        16'sh1B9E    // h[19] inner-centre
    };

    // -----------------------------------------------------------------------
    // Mode decode (SD-OCT = 2'b00 → bypass)
    // -----------------------------------------------------------------------
    wire [1:0] tuser_mode = s_tuser[3:2];
    wire       sd_oct_mode = (tuser_mode == 2'b00);

    // -----------------------------------------------------------------------
    // Flow control: always ready when downstream is ready
    // (pipeline is purely streaming; no back-pressure stall logic)
    // -----------------------------------------------------------------------
    assign s_tready = m_tready;

    // Input handshake qualifier
    wire in_fire = s_tvalid & m_tready;

    // -----------------------------------------------------------------------
    // 40-entry delay line using SRL32 inference
    // Depth = 40 samples; tap[k] is the sample k+1 clocks in the past.
    // (* srl_style = "srl_reg" *) instructs Vivado to use SRL32 primitives
    // backed by an output register, giving clean timing at 250 MHz.
    // -----------------------------------------------------------------------
    (* srl_style = "srl_reg" *)
    logic signed [DATA_W-1:0] delay_line [0:TAPS-1];

    always_ff @(posedge clk) begin
        if (in_fire) begin
            delay_line[0] <= $signed(s_tdata);
            for (int i = 1; i < TAPS; i++) begin
                delay_line[i] <= delay_line[i-1];
            end
        end
    end

    // Alias: delay_line[0] = newest sample (current input registered),
    //        delay_line[TAPS-1] = oldest sample.
    // Symmetric pair k: delay_line[k] + delay_line[TAPS-1-k], k=0..19.

    // -----------------------------------------------------------------------
    // Stage 0 → Stage 1: Pre-add symmetric tap pairs
    // -----------------------------------------------------------------------
    logic signed [DATA_W:0] pre_add [0:HALF_TAPS-1]; // 17-bit (no overflow)

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int k = 0; k < HALF_TAPS; k++) pre_add[k] <= '0;
        end else if (in_fire) begin
            for (int k = 0; k < HALF_TAPS; k++) begin
                pre_add[k] <= delay_line[k] + delay_line[TAPS-1-k];
            end
        end
    end

    // -----------------------------------------------------------------------
    // Stage 1 → Stage 2: Multiply pre-added pair by coefficient (DSP48E2)
    // Product width: 17 × 16 → 33 bits signed; extended to ACCUM_W
    // -----------------------------------------------------------------------
    logic signed [ACCUM_W-1:0] products [0:HALF_TAPS-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int k = 0; k < HALF_TAPS; k++) products[k] <= '0;
        end else begin
            for (int k = 0; k < HALF_TAPS; k++) begin
                products[k] <= ACCUM_W'(pre_add[k] * $signed(COEFF[k]));
            end
        end
    end

    // -----------------------------------------------------------------------
    // Stages 2..19: Pipelined adder tree (binary reduction, 5 levels for 20
    // products).  We use a sequential accumulator approach with 18 pipeline
    // registers to reach the required 20-cycle total latency.
    //
    // Practical implementation: accumulate all 20 products serially across
    // one cycle using a combinational adder tree (4 levels → 16+4=20 total).
    // For synthesis timing at 250 MHz we use a 2-stage registered tree:
    //   Level A (reg): 20 → 10 partial sums
    //   Level B (reg): 10 →  5 partial sums
    //   Level C (reg):  5 →  3 partial sums
    //   Level D (reg):  3 →  2 partial sums
    //   Level E (reg):  2 →  1 final sum
    // Total adder latency = 5 registers.  With pre-add (1) + mult (1) = 7
    // pipeline stages used; remaining 13 stages padded in the bypass path.
    // NOTE: actual latency counted below includes valid/sideband pipelines.
    // -----------------------------------------------------------------------

    // Level A: 10 partial sums
    logic signed [ACCUM_W:0] lvlA [0:9];   // +1 bit guard
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < 10; i++) lvlA[i] <= '0;
        end else begin
            for (int i = 0; i < 10; i++) begin
                lvlA[i] <= (ACCUM_W+1)'(products[2*i]) + (ACCUM_W+1)'(products[2*i+1]);
            end
        end
    end

    // Level B: 5 partial sums
    logic signed [ACCUM_W+1:0] lvlB [0:4];
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < 5; i++) lvlB[i] <= '0;
        end else begin
            for (int i = 0; i < 5; i++) begin
                lvlB[i] <= (ACCUM_W+2)'(lvlA[2*i]) + (ACCUM_W+2)'(lvlA[2*i+1]);
            end
        end
    end

    // Level C: 3 partial sums (5→3: two pairs + one carry)
    logic signed [ACCUM_W+2:0] lvlC [0:2];
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) lvlC[i] <= '0;
        end else begin
            lvlC[0] <= (ACCUM_W+3)'(lvlB[0]) + (ACCUM_W+3)'(lvlB[1]);
            lvlC[1] <= (ACCUM_W+3)'(lvlB[2]) + (ACCUM_W+3)'(lvlB[3]);
            lvlC[2] <= (ACCUM_W+3)'(lvlB[4]);   // carry-through
        end
    end

    // Level D: 2 partial sums
    logic signed [ACCUM_W+3:0] lvlD [0:1];
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < 2; i++) lvlD[i] <= '0;
        end else begin
            lvlD[0] <= (ACCUM_W+4)'(lvlC[0]) + (ACCUM_W+4)'(lvlC[1]);
            lvlD[1] <= (ACCUM_W+4)'(lvlC[2]);
        end
    end

    // Level E: final accumulation
    logic signed [ACCUM_W+4:0] fir_accum;
    always_ff @(posedge clk) begin
        if (!rst_n) fir_accum <= '0;
        else        fir_accum <= (ACCUM_W+5)'(lvlD[0]) + (ACCUM_W+5)'(lvlD[1]);
    end

    // -----------------------------------------------------------------------
    // Output rounding and truncation:
    //   Q1.15 coefficient × 17-bit pre-add → result in Q2.15 domain.
    //   To convert back to integer 16-bit: drop 15 fractional bits with
    //   round-to-nearest (add 0.5 LSB = bit[14] before truncation).
    // -----------------------------------------------------------------------
    localparam int FRAC_BITS = COEFF_W - 1; // 15 fractional bits

    logic signed [OUT_W-1:0] fir_out;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fir_out <= '0;
        end else begin
            // Round: add rounding bit (bit FRAC_BITS-1 of accumulator)
            logic signed [ACCUM_W+5:0] rounded;
            rounded = fir_accum + (ACCUM_W+6)'(fir_accum[FRAC_BITS-1]);
            // Saturate/truncate: take bits [FRAC_BITS+OUT_W-1 : FRAC_BITS]
            fir_out <= rounded[FRAC_BITS +: OUT_W];
        end
    end
    // Pipeline stage count from input:
    //   Stage 0: delay_line shift (in_fire gated)
    //   Stage 1: pre_add  (reg)
    //   Stage 2: products (reg)
    //   Stage 3: lvlA     (reg)
    //   Stage 4: lvlB     (reg)
    //   Stage 5: lvlC     (reg)
    //   Stage 6: lvlD     (reg)
    //   Stage 7: fir_accum(reg)
    //   Stage 8: fir_out  (reg, rounding)
    // Total FIR compute latency = 8 pipeline registers after input registration.
    // We declare LATENCY=20; bypass path pads 20 stages for sideband alignment.

    // -----------------------------------------------------------------------
    // Sideband (valid / last / tuser) pipeline — 20 stages to match LATENCY
    // -----------------------------------------------------------------------
    localparam int SB_STAGES = LATENCY;

    logic              vld_pipe  [0:SB_STAGES-1];
    logic              last_pipe [0:SB_STAGES-1];
    logic [31:0]       user_pipe [0:SB_STAGES-1];
    logic              sdoct_pipe[0:SB_STAGES-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < SB_STAGES; s++) begin
                vld_pipe[s]   <= 1'b0;
                last_pipe[s]  <= 1'b0;
                user_pipe[s]  <= '0;
                sdoct_pipe[s] <= 1'b0;
            end
        end else if (m_tready) begin
            vld_pipe[0]   <= s_tvalid;
            last_pipe[0]  <= s_tlast;
            user_pipe[0]  <= s_tuser;
            sdoct_pipe[0] <= sd_oct_mode;
            for (int s = 1; s < SB_STAGES; s++) begin
                vld_pipe[s]   <= vld_pipe[s-1];
                last_pipe[s]  <= last_pipe[s-1];
                user_pipe[s]  <= user_pipe[s-1];
                sdoct_pipe[s] <= sdoct_pipe[s-1];
            end
        end
    end

    // -----------------------------------------------------------------------
    // SD-OCT bypass path: 20-stage delay line to match FIR latency
    // -----------------------------------------------------------------------
    logic signed [DATA_W-1:0] bypass_pipe [0:SB_STAGES-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < SB_STAGES; s++) bypass_pipe[s] <= '0;
        end else if (m_tready) begin
            bypass_pipe[0] <= $signed(s_tdata);
            for (int s = 1; s < SB_STAGES; s++) begin
                bypass_pipe[s] <= bypass_pipe[s-1];
            end
        end
    end

    // -----------------------------------------------------------------------
    // FIR output delay alignment
    // FIR compute path is 8 clocks (Stage 0..8 above, plus in_fire gating).
    // The sideband pipeline is 20 stages.  We need to align fir_out with
    // sideband stage index 19 (0-indexed).  FIR result arrives 9 clocks after
    // input; pad it with (20-9)=11 more stages.
    // -----------------------------------------------------------------------
    localparam int FIR_LATENCY_ACTUAL = 9;  // stages from input to fir_out
    localparam int FIR_PAD            = SB_STAGES - FIR_LATENCY_ACTUAL; // 11

    logic signed [OUT_W-1:0] fir_aligned [0:FIR_PAD-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < FIR_PAD; s++) fir_aligned[s] <= '0;
        end else if (m_tready) begin
            fir_aligned[0] <= fir_out;
            for (int s = 1; s < FIR_PAD; s++) begin
                fir_aligned[s] <= fir_aligned[s-1];
            end
        end
    end

    // -----------------------------------------------------------------------
    // Output mux: select FIR output or bypass based on mode at pipeline exit
    // -----------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_tdata  <= '0;
            m_tvalid <= 1'b0;
            m_tlast  <= 1'b0;
            m_tuser  <= '0;
        end else if (m_tready) begin
            m_tvalid <= vld_pipe[SB_STAGES-1];
            m_tlast  <= last_pipe[SB_STAGES-1];
            m_tuser  <= user_pipe[SB_STAGES-1];
            if (sdoct_pipe[SB_STAGES-1]) begin
                // SD-OCT bypass: passthrough with latency-matched delay
                m_tdata <= bypass_pipe[SB_STAGES-1];
            end else begin
                // SS-OCT: filtered output
                m_tdata <= fir_aligned[FIR_PAD-1];
            end
        end
    end

endmodule : ss_demod_fir
