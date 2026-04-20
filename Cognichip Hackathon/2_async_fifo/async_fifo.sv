// ============================================================================
// async_fifo.sv
// Parameterized Asynchronous FIFO — Clock-Domain Crossing
//
// Target  : Xilinx Zynq UltraScale+ XCZU7EV (ZCU104)
// Used at : ADC/camera ↔ DSP (250 MHz) and DSP ↔ NPU (300 MHz) boundaries
//
// Design notes:
//   • Gray-code pointer synchronization with 2-FF synchronizers on each side.
//   • DEPTH must be a power of 2.
//   • Distributed RAM inferred for DEPTH < 512.
//   • Block RAM inferred for DEPTH >= 512 via (* ram_style = "block" *).
//   • Full/empty flags are conservative (may assert one cycle early).
//   • Active-low synchronous reset on each clock domain independently.
//   • No latches.
//   • SVA overflow check in translate_off region.
// ============================================================================

module async_fifo #(
  parameter int unsigned DATA_WIDTH = 16,
  parameter int unsigned DEPTH      = 16    // Must be power-of-2
) (
  // ------------------------------------------------------------------
  // Write side
  // ------------------------------------------------------------------
  input  logic                    wr_clk,
  input  logic                    wr_rst_n,
  input  logic                    wr_en,
  input  logic [DATA_WIDTH-1:0]   wr_data,
  output logic                    wr_full,

  // ------------------------------------------------------------------
  // Read side
  // ------------------------------------------------------------------
  input  logic                    rd_clk,
  input  logic                    rd_rst_n,
  input  logic                    rd_en,
  output logic [DATA_WIDTH-1:0]   rd_data,
  output logic                    rd_empty
);

  // --------------------------------------------------------------------------
  // Local parameters
  // --------------------------------------------------------------------------
  localparam int unsigned ADDR_W = $clog2(DEPTH);   // Address / pointer width
  localparam int unsigned PTR_W  = ADDR_W + 1;      // +1 extra bit for full/empty disambiguation

  // --------------------------------------------------------------------------
  // Dual-port storage array
  //   DEPTH < 512  → distributed RAM (LUT-RAM); no attribute needed
  //   DEPTH >= 512 → block RAM via Xilinx ram_style attribute
  // --------------------------------------------------------------------------
  generate
    if (DEPTH >= 512) begin : gen_bram
      (* ram_style = "block" *)
      logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

      // BRAM write port (synchronous)
      always_ff @(posedge wr_clk) begin
        if (wr_en && !wr_full)
          mem[wr_addr_bin] <= wr_data;
      end

      // BRAM read port (synchronous registered output)
      always_ff @(posedge rd_clk) begin
        rd_data <= mem[rd_addr_bin];
      end

    end else begin : gen_lutram
      (* ram_style = "distributed" *)
      logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

      // LUT-RAM write port (synchronous)
      always_ff @(posedge wr_clk) begin
        if (wr_en && !wr_full)
          mem[wr_addr_bin] <= wr_data;
      end

      // LUT-RAM read port (asynchronous → registered at output)
      always_ff @(posedge rd_clk) begin
        rd_data <= mem[rd_addr_bin];
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Binary pointers (internal)
  // --------------------------------------------------------------------------
  logic [PTR_W-1:0] wr_ptr_bin, wr_ptr_bin_next;
  logic [PTR_W-1:0] rd_ptr_bin, rd_ptr_bin_next;

  // Address portion (lower ADDR_W bits)
  logic [ADDR_W-1:0] wr_addr_bin, rd_addr_bin;
  assign wr_addr_bin = wr_ptr_bin[ADDR_W-1:0];
  assign rd_addr_bin = rd_ptr_bin[ADDR_W-1:0];

  // --------------------------------------------------------------------------
  // Binary → Gray conversion function
  // --------------------------------------------------------------------------
  function automatic logic [PTR_W-1:0] bin2gray (input logic [PTR_W-1:0] b);
    bin2gray = b ^ (b >> 1);
  endfunction

  // --------------------------------------------------------------------------
  // Gray-code pointers (registered)
  // --------------------------------------------------------------------------
  logic [PTR_W-1:0] wr_ptr_gray, rd_ptr_gray;

  // --------------------------------------------------------------------------
  // Write-domain: pointer & full flag
  // --------------------------------------------------------------------------
  // rd_ptr_gray synchronized into write clock domain (2-FF)
  logic [PTR_W-1:0] rd_ptr_gray_sync1_wclk, rd_ptr_gray_sync2_wclk;

  (* ASYNC_REG = "TRUE" *)
  always_ff @(posedge wr_clk) begin
    if (!wr_rst_n) begin
      rd_ptr_gray_sync1_wclk <= '0;
      rd_ptr_gray_sync2_wclk <= '0;
    end else begin
      rd_ptr_gray_sync1_wclk <= rd_ptr_gray;
      rd_ptr_gray_sync2_wclk <= rd_ptr_gray_sync1_wclk;
    end
  end

  assign wr_ptr_bin_next = wr_ptr_bin + {{(PTR_W-1){1'b0}}, (wr_en & ~wr_full)};

  always_ff @(posedge wr_clk) begin
    if (!wr_rst_n) begin
      wr_ptr_bin  <= '0;
      wr_ptr_gray <= '0;
    end else begin
      wr_ptr_bin  <= wr_ptr_bin_next;
      wr_ptr_gray <= bin2gray(wr_ptr_bin_next);
    end
  end

  // Full: MSB and second-MSB differ, remaining bits equal
  // (Gray-code full condition for power-of-2 FIFO)
  always_ff @(posedge wr_clk) begin
    if (!wr_rst_n)
      wr_full <= 1'b0;
    else
      wr_full <= (bin2gray(wr_ptr_bin_next) ==
                  {~rd_ptr_gray_sync2_wclk[PTR_W-1],
                   ~rd_ptr_gray_sync2_wclk[PTR_W-2],
                    rd_ptr_gray_sync2_wclk[PTR_W-3:0]});
  end

  // --------------------------------------------------------------------------
  // Read-domain: pointer & empty flag
  // --------------------------------------------------------------------------
  // wr_ptr_gray synchronized into read clock domain (2-FF)
  logic [PTR_W-1:0] wr_ptr_gray_sync1_rclk, wr_ptr_gray_sync2_rclk;

  (* ASYNC_REG = "TRUE" *)
  always_ff @(posedge rd_clk) begin
    if (!rd_rst_n) begin
      wr_ptr_gray_sync1_rclk <= '0;
      wr_ptr_gray_sync2_rclk <= '0;
    end else begin
      wr_ptr_gray_sync1_rclk <= wr_ptr_gray;
      wr_ptr_gray_sync2_rclk <= wr_ptr_gray_sync1_rclk;
    end
  end

  assign rd_ptr_bin_next = rd_ptr_bin + {{(PTR_W-1){1'b0}}, (rd_en & ~rd_empty)};

  always_ff @(posedge rd_clk) begin
    if (!rd_rst_n) begin
      rd_ptr_bin  <= '0;
      rd_ptr_gray <= '0;
    end else begin
      rd_ptr_bin  <= rd_ptr_bin_next;
      rd_ptr_gray <= bin2gray(rd_ptr_bin_next);
    end
  end

  // Empty: Gray pointers match exactly
  always_ff @(posedge rd_clk) begin
    if (!rd_rst_n)
      rd_empty <= 1'b1;
    else
      rd_empty <= (bin2gray(rd_ptr_bin_next) == wr_ptr_gray_sync2_rclk);
  end

  // --------------------------------------------------------------------------
  // Parameter legality check (elaboration-time)
  // --------------------------------------------------------------------------
  initial begin
    if ((DEPTH & (DEPTH - 1)) != 0) begin
      $fatal(1, "async_fifo: DEPTH=%0d is not a power of 2.", DEPTH);
    end
    if (DEPTH < 2) begin
      $fatal(1, "async_fifo: DEPTH must be >= 2.");
    end
  end

  // --------------------------------------------------------------------------
  // SVA — overflow check (simulation only)
  // --------------------------------------------------------------------------
  // synthesis translate_off
  // Ensure wr_en is never asserted while FIFO is full.
  property p_no_overflow;
    @(posedge wr_clk) disable iff (!wr_rst_n)
    wr_full |-> !wr_en;
  endproperty

  assert property (p_no_overflow)
    else $error("[async_fifo] OVERFLOW: wr_en asserted when wr_full=1 at time %0t", $time);
  // synthesis translate_on

endmodule : async_fifo
