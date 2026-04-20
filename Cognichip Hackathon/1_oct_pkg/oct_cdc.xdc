# ==============================================================================
# oct_cdc.xdc
# Timing Constraints — Clock-Domain Crossing (CDC) for OCT AI Accelerator
#
# Target  : Xilinx Zynq UltraScale+ XCZU7EV (ZCU104)
# Scope   : All async_fifo instances at the three CDC boundaries:
#             1. adc_clk   ↔ dsp_clk  (DSP clock  : 250 MHz)
#             2. dsp_clk   ↔ npu_clk  (NPU clock  : 300 MHz)
#             3. cam_clk   ↔ dsp_clk  (camera variant, if used)
#
# Constraint strategy:
#   • create_clock        — define all primary clocks
#   • set_clock_groups    — declare async relationships (no common period)
#   • set_false_path -to  — suppress timing on ASYNC_REG synchronizer FFs
#   • set_max_delay       — bound synchronizer input setup (data stability)
# ==============================================================================


# ------------------------------------------------------------------------------
# 1. Primary Clock Definitions
#    Adjust pin names to match your actual ZCU104 board connections.
# ------------------------------------------------------------------------------

# ADC/camera input clock — representative 100 MHz (adjust to actual ADC clock)
create_clock -period 10.000 -name adc_clk  [get_ports adc_clk]

# Camera pixel clock — representative 74.25 MHz (adjust to actual camera clock)
create_clock -period 13.468 -name cam_clk  [get_ports cam_clk]

# DSP processing clock — 250 MHz (4 ns period)
create_clock -period  4.000 -name dsp_clk  [get_ports dsp_clk]

# NPU inference clock — 300 MHz (3.333 ns period)
create_clock -period  3.333 -name npu_clk  [get_ports npu_clk]

# PL system clock from PS (used for AXI control path) — 100 MHz typical
# Usually auto-created by Vivado from the PS block; shown here for completeness.
# create_clock -period 10.000 -name pl_clk0 [get_pins u_zynq_ps/inst/PS8_i/PLCLK[0]]


# ------------------------------------------------------------------------------
# 2. Declare All Clock Groups as Asynchronous
#    This prevents Vivado from analysing timing between unrelated domains
#    and generating false critical warnings.
# ------------------------------------------------------------------------------

set_clock_groups -asynchronous \
  -group [get_clocks adc_clk] \
  -group [get_clocks cam_clk] \
  -group [get_clocks dsp_clk] \
  -group [get_clocks npu_clk]


# ------------------------------------------------------------------------------
# 3. False Path on ASYNC_REG Synchronizer Flip-Flops
#    The (* ASYNC_REG = "TRUE" *) attribute is applied inside async_fifo.sv
#    to both 2-FF synchronizer chains (wr→rd and rd→wr).
#    Vivado must NOT perform setup/hold analysis across these FFs — metastability
#    resolution is handled by the synchronizer design itself.
#
#    This single constraint catches ALL instances of async_fifo in the design.
# ------------------------------------------------------------------------------

set_false_path -to [get_cells -hierarchical -filter {ASYNC_REG == TRUE}]


# ------------------------------------------------------------------------------
# 4. set_max_delay on Synchronizer Inputs (Data Stability Window)
#    Constrain the combinational path INTO the first synchronizer FF to be
#    no longer than the destination clock period.  This ensures the Gray-code
#    pointer is stable before it can be sampled.
#
#    Use -datapath_only to exclude clock skew from the analysis (appropriate
#    because these are intentional CDC paths).
# ------------------------------------------------------------------------------

# --- ADC → DSP FIFO (all instances of u_fifo_adc_dsp*) ---
set_max_delay -datapath_only -from [get_clocks adc_clk] \
  -to [get_cells -hierarchical \
       -filter {NAME =~ *fifo_adc_dsp* && ASYNC_REG == TRUE}] \
  4.000  ;# destination dsp_clk period

set_max_delay -datapath_only -from [get_clocks dsp_clk] \
  -to [get_cells -hierarchical \
       -filter {NAME =~ *fifo_adc_dsp* && ASYNC_REG == TRUE}] \
  10.000  ;# destination adc_clk period (100 MHz example)

# --- Camera → DSP FIFO (all instances of u_fifo_cam_dsp*) ---
set_max_delay -datapath_only -from [get_clocks cam_clk] \
  -to [get_cells -hierarchical \
       -filter {NAME =~ *fifo_cam_dsp* && ASYNC_REG == TRUE}] \
  4.000  ;# destination dsp_clk period

set_max_delay -datapath_only -from [get_clocks dsp_clk] \
  -to [get_cells -hierarchical \
       -filter {NAME =~ *fifo_cam_dsp* && ASYNC_REG == TRUE}] \
  13.468  ;# destination cam_clk period

# --- DSP → NPU FIFO (all instances of u_fifo_dsp_npu*) ---
set_max_delay -datapath_only -from [get_clocks dsp_clk] \
  -to [get_cells -hierarchical \
       -filter {NAME =~ *fifo_dsp_npu* && ASYNC_REG == TRUE}] \
  3.333  ;# destination npu_clk period

set_max_delay -datapath_only -from [get_clocks npu_clk] \
  -to [get_cells -hierarchical \
       -filter {NAME =~ *fifo_dsp_npu* && ASYNC_REG == TRUE}] \
  4.000  ;# destination dsp_clk period


# ------------------------------------------------------------------------------
# 5. ASYNC_REG Placement — Keep Synchronizer Pairs in the Same Slice
#    The (* ASYNC_REG = "TRUE" *) attribute already instructs Vivado to
#    co-locate each synchronizer chain.  The set_property below reinforces
#    this for any case where the attribute alone is insufficient.
# ------------------------------------------------------------------------------

set_property ASYNC_REG TRUE \
  [get_cells -hierarchical -filter {ASYNC_REG == TRUE}]


# ------------------------------------------------------------------------------
# 6. Output / Input Delay Placeholders (adjust for your I/O interfaces)
#    These are stubs — populate with actual PCB trace delays from the
#    ADC and camera datasheets + board layout.
# ------------------------------------------------------------------------------

# Example: ADC data input (LVDS source-synchronous, adjust as needed)
# set_input_delay -clock adc_clk -max 1.500 [get_ports {adc_data[*]}]
# set_input_delay -clock adc_clk -min 0.500 [get_ports {adc_data[*]}]

# Example: Camera parallel pixel bus
# set_input_delay -clock cam_clk -max 2.000 [get_ports {cam_data[*]}]
# set_input_delay -clock cam_clk -min 0.800 [get_ports {cam_data[*]}]


# ==============================================================================
# End of oct_cdc.xdc
# ==============================================================================
