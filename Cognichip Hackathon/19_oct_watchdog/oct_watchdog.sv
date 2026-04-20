`timescale 1ns/1ps
import oct_pkg::*;

//------------------------------------------------------------------------------
// oct_watchdog
//------------------------------------------------------------------------------
// Deterministic frame-rate monitor for the OCT DSP pipeline.
//
// Function:
// - Measures DSP clock cycles between consecutive ascan_tlast pulses
// - Computes expected_period = CLK_FREQ_HZ / expected_ascan_rate
// - Asserts locked when measured period is within +/-10% of expected_period
// - Latches overflow_flag when any tuser_errors bit is set
// - Asserts galvo_drift when 3 consecutive A-scan periods are out of window
// - Generates a 1-cycle irq pulse when galvo_drift newly asserts
//
// Notes:
// 1) expected_ascan_rate is assumed to be in A-scans/second, integer-scaled
//    such that CLK_FREQ_HZ / expected_ascan_rate is meaningful in integer math.
// 2) overflow_flag is sticky. The prompt references clearing it via AXI write
//    in a STATUS register, but no clear input is provided in this port list.
//    Therefore it clears only on reset in this interface.
// 3) bscan_frame_done is accepted as an observability input but not required
//    for the core A-scan period monitor. It is reserved for future frame-level
//    supervision or software correlation.
// 4) No DSPs are used; arithmetic is logic-only.
//------------------------------------------------------------------------------
module oct_watchdog #(
  parameter int unsigned CLK_FREQ_HZ = 250_000_000
)(
  input  logic              clk,
  input  logic              rst_n,

  // Observability inputs
  input  logic              ascan_tlast,
  input  logic              bscan_frame_done,
  input  logic [31:0]       expected_ascan_rate,
  input  logic [31:0]       tuser_errors,

  // Status outputs
  output logic              overflow_flag,
  output logic              locked,
  output logic              galvo_drift,
  output logic              irq
);

  //----------------------------------------------------------------------------
  // Edge detect for ascan_tlast
  //----------------------------------------------------------------------------
  logic ascan_tlast_d;
  logic ascan_pulse;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      ascan_tlast_d <= 1'b0;
    else
      ascan_tlast_d <= ascan_tlast;
  end

  assign ascan_pulse = ascan_tlast & ~ascan_tlast_d;

  //----------------------------------------------------------------------------
  // Period measurement
  //----------------------------------------------------------------------------
  logic [31:0] period_counter;
  logic [31:0] measured_period;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      period_counter   <= 32'd0;
      measured_period  <= 32'd0;
    end
    else begin
      if (ascan_pulse) begin
        measured_period <= period_counter;
        period_counter  <= 32'd0;
      end
      else begin
        period_counter <= period_counter + 32'd1;
      end
    end
  end

  //----------------------------------------------------------------------------
  // Expected period and +/-10% window
  //----------------------------------------------------------------------------
  logic [31:0] expected_period;
  logic [31:0] tol_period;
  logic [31:0] lower_bound;
  logic [31:0] upper_bound;
  logic        period_in_window;
  logic        period_out_window;

  always_comb begin
    if (expected_ascan_rate != 32'd0)
      expected_period = CLK_FREQ_HZ / expected_ascan_rate;
    else
      expected_period = 32'hFFFF_FFFF;

    tol_period = expected_period / 32'd10;

    if (expected_period > tol_period)
      lower_bound = expected_period - tol_period;
    else
      lower_bound = 32'd0;

    upper_bound = expected_period + tol_period;

    period_in_window  = (measured_period >= lower_bound) &&
                        (measured_period <= upper_bound) &&
                        (expected_ascan_rate != 32'd0) &&
                        (measured_period != 32'd0);

    period_out_window = (measured_period < lower_bound) ||
                        (measured_period > upper_bound);
  end

  //----------------------------------------------------------------------------
  // locked status updates on each completed A-scan period
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      locked <= 1'b0;
    end
    else if (ascan_pulse) begin
      locked <= period_in_window;
    end
  end

  //----------------------------------------------------------------------------
  // Sticky overflow flag
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      overflow_flag <= 1'b0;
    else if (|tuser_errors)
      overflow_flag <= 1'b1;
  end

  //----------------------------------------------------------------------------
  // Galvo drift detection:
  // assert after 3 consecutive out-of-window A-scan periods
  //----------------------------------------------------------------------------
  logic [1:0] drift_count;
  logic       galvo_drift_d;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      drift_count  <= 2'd0;
      galvo_drift  <= 1'b0;
      galvo_drift_d <= 1'b0;
    end
    else begin
      galvo_drift_d <= galvo_drift;

      if (ascan_pulse) begin
        if ((expected_ascan_rate == 32'd0) || (measured_period == 32'd0)) begin
          drift_count <= 2'd0;
        end
        else if (period_out_window) begin
          if (drift_count < 2'd3)
            drift_count <= drift_count + 2'd1;
          else
            drift_count <= drift_count;
        end
        else begin
          drift_count <= 2'd0;
        end

        if (((period_out_window) && (drift_count >= 2'd2)) &&
            (expected_ascan_rate != 32'd0) &&
            (measured_period != 32'd0))
          galvo_drift <= 1'b1;
        else if (!period_out_window)
          galvo_drift <= 1'b0;
      end
    end
  end

  //----------------------------------------------------------------------------
  // IRQ pulse on new galvo_drift assertion
  // "open-drain style, active-high pulse" interpreted here as a 1-cycle pulse.
  // Actual open-drain behavior belongs at the I/O buffer/top level.
  //----------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      irq <= 1'b0;
    else
      irq <= (ascan_pulse &&
              (period_out_window) &&
              (drift_count >= 2'd2) &&
              !galvo_drift_d &&
              (expected_ascan_rate != 32'd0) &&
              (measured_period != 32'd0));
  end

endmodule