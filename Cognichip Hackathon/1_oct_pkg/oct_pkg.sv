`ifndef OCT_PKG_SV
`define OCT_PKG_SV

// ============================================================================
// oct_pkg.sv
// OCT AI Accelerator — Global Types & Parameters
//
// Target  : Xilinx Zynq UltraScale+ XCZU7EV (ZCU104)
// Imported by every module in the OCT AI Accelerator design hierarchy.
// Synthesis-clean: no simulation-only constructs.
// ============================================================================

package oct_pkg;

  // --------------------------------------------------------------------------
  // A-scan / B-scan geometry
  // --------------------------------------------------------------------------
  parameter int unsigned ASCAN_SAMPLES      = 1024;   // Default FFT size (1024-point)
  parameter int unsigned ASCAN_SAMPLES_2K   = 2048;   // Optional 2048-point mode
  parameter int unsigned BSCAN_LINES        = 1024;   // A-scans per B-scan frame

  // --------------------------------------------------------------------------
  // Data path widths
  // --------------------------------------------------------------------------
  parameter int unsigned DATA_WIDTH         = 16;     // Raw ADC / camera sample (signed)
  parameter int unsigned LOG_WIDTH          = 8;      // Output grayscale pixel width
  parameter int unsigned FFT_WIDTH          = 32;     // Packed complex FFT word
                                                      //   [31:16] = imag (16-bit signed)
                                                      //   [15: 0] = real (16-bit signed)
  parameter int unsigned WEIGHT_WIDTH       = 8;      // NPU INT8 weight
  parameter int unsigned ACCUM_WIDTH        = 32;     // NPU accumulator width

  // --------------------------------------------------------------------------
  // AXI4-Stream TUSER sideband (packed struct, exactly 32 bits)
  //
  //  Bit layout (MSB → LSB):
  //  [31:16]  ascan_idx  (16) — A-scan index within the current B-scan
  //  [15: 8]  galvo_x    ( 8) — Galvanometer X position (normalized)
  //  [ 7: 2]  galvo_y    ( 6) — Galvanometer Y position (normalized)
  //  [ 3: 2]  mode       ( 2) — 2'b00=SD-OCT, 2'b01=SS-OCT
  //  [ 1: 0]  valid      ( 2) — Error / overflow flags propagated downstream
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [15:0] ascan_idx;   // [31:16]
    logic [ 7:0] galvo_x;     // [15: 8]
    logic [ 5:0] galvo_y;     // [ 7: 2]
    logic [ 1:0] mode;        // [ 3: 2]
    logic [ 1:0] valid;       // [ 1: 0]
  } oct_tuser_t;              // Total: 16+8+6+2+2 = 34 bits — see note below

  // Note: The five fields above sum to 34 bits.  The spec calls for a 32-bit
  // TUSER word.  To honour both constraints the two overlapping bit positions
  // are resolved as follows:
  //   • galvo_y is kept at 6 bits (positions [7:2])
  //   • mode    is kept at 2 bits (positions [3:2])  ← overlapping sub-range
  //   • valid   is kept at 2 bits (positions [1:0])
  // This means galvo_y, mode and valid share [7:0] with galvo_x, which would
  // make the struct 34 bits wide and NOT match a 32-bit TUSER word.
  //
  // Resolution: galvo_y is reduced to 4 bits so the total equals exactly 32:
  //   16 (ascan_idx) + 8 (galvo_x) + 4 (galvo_y) + 2 (mode) + 2 (valid) = 32
  //
  // The canonical 32-bit TUSER type is defined below.  oct_tuser_t is kept
  // above for reference but should be cast to oct_tuser32_t at port boundaries.

  typedef struct packed {
    logic [15:0] ascan_idx;   // [31:16] A-scan index within B-scan
    logic [ 7:0] galvo_x;     // [15: 8] Galvanometer X (normalized)
    logic [ 3:0] galvo_y;     // [ 7: 4] Galvanometer Y (normalized, 4-bit)
    logic [ 1:0] mode;        // [ 3: 2] 00=SD-OCT, 01=SS-OCT
    logic [ 1:0] valid;       // [ 1: 0] Error / overflow flags
  } oct_tuser32_t;            // Total: 16+8+4+2+2 = 32 bits ✓

  // --------------------------------------------------------------------------
  // Control register struct
  //
  //  Bit layout (MSB → LSB):
  //  [ 9: 8]  mode_sel    (2) — SD / SS selection
  //  [ 7: 6]  fft_size    (2) — 0=512, 1=1024, 2=2048
  //  [ 5: 3]  ai_task     (3) — 0=off,1=denoise,2=segment,3=superres,4=classify
  //  [    2]  bypass_filt (1) — Bypass spectral shaping filter
  //  [    1]  bypass_disp (1) — Bypass dispersion compensation
  //  [    0]  bypass_npu  (1) — Bypass NPU
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [1:0] mode_sel;     // [ 9: 8] SD-OCT / SS-OCT
    logic [1:0] fft_size;     // [ 7: 6] FFT size select
    logic [2:0] ai_task;      // [ 5: 3] AI task selector
    logic       bypass_filt;  // [    2] Bypass spectral shaping
    logic       bypass_disp;  // [    1] Bypass dispersion compensation
    logic       bypass_npu;   // [    0] Bypass NPU
  } oct_ctrl_t;               // Total: 2+2+3+1+1+1 = 10 bits

  // --------------------------------------------------------------------------
  // Convenience enumerations (synthesizable)
  // --------------------------------------------------------------------------

  // OCT acquisition mode (maps to oct_ctrl_t.mode_sel / oct_tuser32_t.mode)
  typedef enum logic [1:0] {
    MODE_SD_OCT = 2'b00,
    MODE_SS_OCT = 2'b01
  } oct_mode_e;

  // FFT size select (maps to oct_ctrl_t.fft_size)
  typedef enum logic [1:0] {
    FFT_SIZE_512  = 2'b00,
    FFT_SIZE_1024 = 2'b01,
    FFT_SIZE_2048 = 2'b10
  } fft_size_e;

  // AI task select (maps to oct_ctrl_t.ai_task)
  typedef enum logic [2:0] {
    AI_TASK_OFF      = 3'd0,
    AI_TASK_DENOISE  = 3'd1,
    AI_TASK_SEGMENT  = 3'd2,
    AI_TASK_SUPERRES = 3'd3,
    AI_TASK_CLASSIFY = 3'd4
  } ai_task_e;

endpackage : oct_pkg

`endif // OCT_PKG_SV
