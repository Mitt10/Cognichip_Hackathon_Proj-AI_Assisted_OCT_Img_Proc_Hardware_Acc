// ============================================================================
// dc_subtract.sv
// OCT AI Accelerator — DC / Background Subtraction (Module 7)
//
// Maintains a 1024-entry per-sample background estimate in a BRAM and
// subtracts it from each incoming A-scan sample (Tang et al. 2014/2018/2019).
//
// Pipeline (2-cycle latency):
//   Cycle N   : present BRAM read address (sample index); latch input sideband
//               into stage-0 delay registers.
//   Cycle N+1 : BRAM registered output (bg) available; compute:
//                 diff    = sample - bg          (DC-removed output)
//                 new_bg  = bg + (diff >>> avg_shift)  (IIR update)
//               Write new_bg back via Port B; register result to output.
//
// Flow control:
//   pipe_en = m_tready || !m_tvalid   — entire pipeline freezes when stalled.
//   s_tready is driven by pipe_en.
//   BRAM read address is held when pipe_en=0, keeping rdata stable.
//   Port B write is gated by pipe_en to avoid duplicate writes on stall.
//
// Resource target: 1 × BRAM36, ~150 LUTs.
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module dc_subtract (
    input  logic        clk,
    input  logic        rst_n,

    // -----------------------------------------------------------------------
    // AXI4-Stream slave
    // -----------------------------------------------------------------------
    input  logic [15:0] s_tdata,   // signed 16-bit raw sample
    input  logic        s_tvalid,
    input  logic        s_tlast,
    input  logic [31:0] s_tuser,
    output logic        s_tready,

    // -----------------------------------------------------------------------
    // AXI4-Stream master (signed, DC-removed)
    // -----------------------------------------------------------------------
    output logic [15:0] m_tdata,
    output logic        m_tvalid,
    output logic        m_tlast,
    output logic [31:0] m_tuser,
    input  logic        m_tready,

    // -----------------------------------------------------------------------
    // Config
    // -----------------------------------------------------------------------
    input  logic [7:0]  avg_shift  // IIR shift: bg += (sample - bg) >> avg_shift
);

    // -----------------------------------------------------------------------
    // Pipeline enable — advance the entire pipeline this cycle
    // -----------------------------------------------------------------------
    logic pipe_en;
    assign pipe_en  = m_tready || !m_tvalid;
    assign s_tready = pipe_en;

    // -----------------------------------------------------------------------
    // Sample index counter (0 … ASCAN_SAMPLES-1, resets on tlast)
    // -----------------------------------------------------------------------
    localparam int unsigned IDX_W = $clog2(ASCAN_SAMPLES); // 10 bits for 1024

    logic [IDX_W-1:0] idx;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            idx <= '0;
        end else if (pipe_en && s_tvalid) begin
            if (s_tlast) idx <= '0;
            else         idx <= idx + 1'b1;
        end
    end

    // -----------------------------------------------------------------------
    // Background BRAM — 1024 × 16-bit
    //   Port A : synchronous read  (registered output, 1-cycle latency)
    //   Port B : synchronous write (write-enable gated)
    // -----------------------------------------------------------------------
    (* ram_style = "block" *)
    logic [15:0] bg_mem [0:ASCAN_SAMPLES-1];

    // --- Port A: read ---
    logic [IDX_W-1:0] raddr_a;   // held when pipe_en=0 to freeze output
    logic [15:0]      rdata_a;   // registered BRAM output, available cycle N+1

    always_ff @(posedge clk) begin
        if (pipe_en) begin
            raddr_a <= idx;
            rdata_a <= bg_mem[idx];
        end
        // When !pipe_en: raddr_a/rdata_a hold their values (pipeline frozen)
    end

    // --- Port B: write ---
    logic             wen_b;
    logic [IDX_W-1:0] waddr_b;
    logic [15:0]      wdata_b;

    always_ff @(posedge clk) begin
        if (wen_b) bg_mem[waddr_b] <= wdata_b;
    end

    // -----------------------------------------------------------------------
    // Stage-0 delay registers
    //   Latch input sideband on cycle N so it aligns with BRAM output on N+1
    // -----------------------------------------------------------------------
    logic [15:0]      st0_data;
    logic             st0_valid;
    logic             st0_last;
    logic [31:0]      st0_user;
    logic [IDX_W-1:0] st0_idx;   // index for Port B write-back

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st0_valid <= 1'b0;
            st0_data  <= '0;
            st0_last  <= 1'b0;
            st0_user  <= '0;
            st0_idx   <= '0;
        end else if (pipe_en) begin
            st0_valid <= s_tvalid;
            st0_data  <= s_tdata;
            st0_last  <= s_tlast;
            st0_user  <= s_tuser;
            st0_idx   <= idx;      // capture the address sent to BRAM Port A
        end
    end

    // -----------------------------------------------------------------------
    // Stage-1 combinational: subtraction + IIR background update
    //   Inputs  : st0_* (1-cycle delayed) and rdata_a (BRAM registered output)
    //   Both are stable on cycle N+1 because pipe_en was 1 on cycle N.
    // -----------------------------------------------------------------------
    logic signed [15:0] bg_s;      // background estimate
    logic signed [15:0] sample_s;  // raw sample (delayed 1 cycle)
    logic signed [16:0] diff_wide; // extra bit to avoid overflow in subtract
    logic signed [15:0] diff_s;    // DC-removed (saturated to 16-bit)
    logic signed [16:0] new_bg_wide;
    logic signed [15:0] new_bg_s;  // updated background

    assign bg_s       = signed'(rdata_a);
    assign sample_s   = signed'(st0_data);
    assign diff_wide  = 17'(sample_s) - 17'(bg_s);

    // Saturate diff to 16-bit signed
    assign diff_s = (diff_wide > 17'sh7FFF) ? 16'sh7FFF :
                    (diff_wide < -17'sh8000) ? 16'sh8000 :
                    diff_wide[15:0];

    // IIR: new_bg = bg + (diff >>> avg_shift)
    assign new_bg_wide = 17'(bg_s) + 17'($signed(diff_s) >>> avg_shift);
    assign new_bg_s    = (new_bg_wide > 17'sh7FFF) ? 16'sh7FFF :
                         (new_bg_wide < -17'sh8000) ? 16'sh8000 :
                         new_bg_wide[15:0];

    // Port B write-back: write new_bg on the same cycle result is computed,
    // only when pipeline is advancing and the stage-0 beat is valid.
    assign wen_b   = st0_valid && pipe_en;
    assign waddr_b = st0_idx;
    assign wdata_b = new_bg_s;

    // -----------------------------------------------------------------------
    // Output register stage (skid buffer)
    //   Loaded from Stage-1 when pipe_en=1, held when downstream stalls.
    // -----------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_tvalid <= 1'b0;
            m_tdata  <= '0;
            m_tlast  <= 1'b0;
            m_tuser  <= '0;
        end else if (pipe_en) begin
            m_tvalid <= st0_valid;
            m_tdata  <= diff_s;    // signed 16-bit DC-removed sample
            m_tlast  <= st0_last;
            m_tuser  <= st0_user;
        end
    end

endmodule : dc_subtract
