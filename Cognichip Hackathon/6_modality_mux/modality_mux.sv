// ============================================================================
// modality_mux.sv
// OCT AI Accelerator — Modality Selector MUX (Module 6)
//
// Selects between SD-OCT and SS-OCT AXI4-Stream sources.
//
// Key behaviour:
//   - mode_sel is latched only at packet boundaries (after tlast) to avoid
//     mid-frame switching glitches.
//   - Combinational pass-through of the selected source; the other source's
//     tready is driven low.
//   - One register stage on the output path for timing closure.
//
// Reset: active-low synchronous (rst_n), consistent with the rest of the OCT
//        pipeline (sd_oct_frontend / ss_oct_frontend use rst_n).
// ============================================================================

`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module modality_mux (
    input  logic        clk,
    input  logic        rst_n,

    // -----------------------------------------------------------------------
    // Mode select (from ctrl_reg)
    //   0 = SD-OCT, 1 = SS-OCT  (upper bits reserved / ignored)
    // -----------------------------------------------------------------------
    input  logic [1:0]  mode_sel,

    // -----------------------------------------------------------------------
    // SD-OCT source
    // -----------------------------------------------------------------------
    input  logic [15:0] sd_tdata,
    input  logic        sd_tvalid,
    input  logic        sd_tlast,
    input  logic [31:0] sd_tuser,
    output logic        sd_tready,

    // -----------------------------------------------------------------------
    // SS-OCT source
    // -----------------------------------------------------------------------
    input  logic [15:0] ss_tdata,
    input  logic        ss_tvalid,
    input  logic        ss_tlast,
    input  logic [31:0] ss_tuser,
    output logic        ss_tready,

    // -----------------------------------------------------------------------
    // Downstream (registered output)
    // -----------------------------------------------------------------------
    output logic [15:0] m_tdata,
    output logic        m_tvalid,
    output logic        m_tlast,
    output logic [31:0] m_tuser,
    input  logic        m_tready
);

    // -----------------------------------------------------------------------
    // Mode latch — only update mode at packet boundaries
    //
    // mode_active: the mode currently being forwarded downstream.
    // pkt_idle   : asserted when no packet is in flight (between tlast and
    //              the start of the next packet).  The mode is allowed to
    //              change only while pkt_idle is true.
    // -----------------------------------------------------------------------
    logic        mode_active; // 0 = SD-OCT, 1 = SS-OCT
    logic        pkt_idle;    // Between-packet gate

    // Detect the end-of-packet on the selected (pre-register) path.
    // A transfer completes when tvalid & tready are both asserted.
    // tlast marks the final beat of a packet.
    logic        sel_tvalid_pre;
    logic        sel_tlast_pre;
    logic        sel_tready_pre; // tready flowing back from the output stage

    // Packet idle flag:
    //   - Set   on reset (idle at startup).
    //   - Cleared when the first beat of a new packet is accepted (tvalid &
    //     tready with !tlast, or even a single-beat packet).
    //   - Set again when the last beat is accepted (tvalid & tready & tlast).
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pkt_idle <= 1'b1;
        end else begin
            if (sel_tvalid_pre && sel_tready_pre && sel_tlast_pre) begin
                // End of packet — return to idle, allow mode switch next cycle
                pkt_idle <= 1'b1;
            end else if (sel_tvalid_pre && sel_tready_pre) begin
                // Mid-packet beat accepted — no longer idle
                pkt_idle <= 1'b0;
            end
        end
    end

    // Latch mode_sel only when between packets
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            mode_active <= 1'b0; // Default: SD-OCT
        end else if (pkt_idle) begin
            mode_active <= mode_sel[0]; // Bit 0 carries SD(0)/SS(1) selection
        end
    end

    // -----------------------------------------------------------------------
    // Combinational MUX — select source signals
    // -----------------------------------------------------------------------
    logic [15:0] sel_tdata_pre;
    logic [31:0] sel_tuser_pre;

    always_comb begin
        if (!mode_active) begin
            // SD-OCT selected
            sel_tdata_pre  = sd_tdata;
            sel_tvalid_pre = sd_tvalid;
            sel_tlast_pre  = sd_tlast;
            sel_tuser_pre  = sd_tuser;
            sd_tready      = sel_tready_pre;
            ss_tready      = 1'b0;
        end else begin
            // SS-OCT selected
            sel_tdata_pre  = ss_tdata;
            sel_tvalid_pre = ss_tvalid;
            sel_tlast_pre  = ss_tlast;
            sel_tuser_pre  = ss_tuser;
            ss_tready      = sel_tready_pre;
            sd_tready      = 1'b0;
        end
    end

    // -----------------------------------------------------------------------
    // Output register stage (one pipeline register for timing closure)
    //
    // Uses a simple elastic buffer / skid register:
    //   - When the output is not stalled (m_tready or !m_tvalid), accept
    //     a new beat from upstream directly.
    //   - sel_tready_pre is asserted whenever the register can accept data.
    // -----------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_tdata  <= '0;
            m_tvalid <= 1'b0;
            m_tlast  <= 1'b0;
            m_tuser  <= '0;
        end else begin
            if (m_tready || !m_tvalid) begin
                // Output register is free — load next beat (if any)
                m_tdata  <= sel_tdata_pre;
                m_tvalid <= sel_tvalid_pre;
                m_tlast  <= sel_tlast_pre;
                m_tuser  <= sel_tuser_pre;
            end
            // When !m_tready && m_tvalid: hold current output — no update
        end
    end

    // Back-pressure: upstream can push when the register is empty or being
    // consumed this cycle.
    assign sel_tready_pre = m_tready || !m_tvalid;

endmodule : modality_mux
