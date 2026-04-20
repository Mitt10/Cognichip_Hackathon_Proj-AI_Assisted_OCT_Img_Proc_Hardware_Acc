`timescale 1ns/1ps

module tb_oct_top;

  // ---------------------------------------------------------------------------
  // Clocks / reset
  // ---------------------------------------------------------------------------
  logic pl_clk0;
  logic pl_clk1;
  logic adc_clk;
  logic pl_resetn;

  localparam time PL_CLK0_PER = 4ns;       // 250 MHz
  localparam time PL_CLK1_PER = 3.333ns;   // ~300 MHz
  localparam time ADC_CLK_PER = 5ns;       // arbitrary TB ADC clock

  // ---------------------------------------------------------------------------
  // AXI4-Lite
  // ---------------------------------------------------------------------------
  logic [5:0]  s_axi_awaddr;
  logic        s_axi_awvalid;
  logic        s_axi_awready;
  logic [31:0] s_axi_wdata;
  logic [3:0]  s_axi_wstrb;
  logic        s_axi_wvalid;
  logic        s_axi_wready;
  logic [1:0]  s_axi_bresp;
  logic        s_axi_bvalid;
  logic        s_axi_bready;
  logic [5:0]  s_axi_araddr;
  logic        s_axi_arvalid;
  logic        s_axi_arready;
  logic [31:0] s_axi_rdata;
  logic [1:0]  s_axi_rresp;
  logic        s_axi_rvalid;
  logic        s_axi_rready;

  // ---------------------------------------------------------------------------
  // SD-OCT camera interface
  // ---------------------------------------------------------------------------
  logic [15:0] cam_pixel;
  logic        cam_valid;
  logic        cam_lval;
  logic        cam_fval;

  // ---------------------------------------------------------------------------
  // SS-OCT ADC
  // ---------------------------------------------------------------------------
  logic [13:0] adc_data;
  logic        adc_valid;
  logic        mzi_fringe;

  // ---------------------------------------------------------------------------
  // Galvo / laser
  // ---------------------------------------------------------------------------
  logic galvo_sclk;
  logic galvo_cs_n;
  logic galvo_mosi;
  logic laser_trig;

  // ---------------------------------------------------------------------------
  // AXI4-Stream output to PS DMA
  // ---------------------------------------------------------------------------
  logic [7:0]  m_axis_out_tdata;
  logic        m_axis_out_tvalid;
  logic        m_axis_out_tlast;
  logic [31:0] m_axis_out_tuser;
  logic        m_axis_out_tready;

  // ---------------------------------------------------------------------------
  // IRQ
  // ---------------------------------------------------------------------------
  logic watchdog_irq;

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------
  oct_top dut (
    .pl_clk0          (pl_clk0),
    .pl_clk1          (pl_clk1),
    .pl_resetn        (pl_resetn),
    .adc_clk          (adc_clk),

    .s_axi_awaddr     (s_axi_awaddr),
    .s_axi_awvalid    (s_axi_awvalid),
    .s_axi_awready    (s_axi_awready),
    .s_axi_wdata      (s_axi_wdata),
    .s_axi_wstrb      (s_axi_wstrb),
    .s_axi_wvalid     (s_axi_wvalid),
    .s_axi_wready     (s_axi_wready),
    .s_axi_bresp      (s_axi_bresp),
    .s_axi_bvalid     (s_axi_bvalid),
    .s_axi_bready     (s_axi_bready),
    .s_axi_araddr     (s_axi_araddr),
    .s_axi_arvalid    (s_axi_arvalid),
    .s_axi_arready    (s_axi_arready),
    .s_axi_rdata      (s_axi_rdata),
    .s_axi_rresp      (s_axi_rresp),
    .s_axi_rvalid     (s_axi_rvalid),
    .s_axi_rready     (s_axi_rready),

    .cam_pixel        (cam_pixel),
    .cam_valid        (cam_valid),
    .cam_lval         (cam_lval),
    .cam_fval         (cam_fval),

    .adc_data         (adc_data),
    .adc_valid        (adc_valid),
    .mzi_fringe       (mzi_fringe),

    .galvo_sclk       (galvo_sclk),
    .galvo_cs_n       (galvo_cs_n),
    .galvo_mosi       (galvo_mosi),
    .laser_trig       (laser_trig),

    .m_axis_out_tdata (m_axis_out_tdata),
    .m_axis_out_tvalid(m_axis_out_tvalid),
    .m_axis_out_tlast (m_axis_out_tlast),
    .m_axis_out_tuser (m_axis_out_tuser),
    .m_axis_out_tready(m_axis_out_tready),

    .watchdog_irq     (watchdog_irq)
  );

  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------
  localparam logic [5:0] ADDR_CTRL       = 6'h00;
  localparam logic [5:0] ADDR_STATUS     = 6'h04;
  localparam logic [5:0] ADDR_ASCAN_RATE = 6'h08;
  localparam logic [5:0] ADDR_AI_LATENCY = 6'h0C;

  // NOTE:
  // Uploaded oct_ctrl_regs.sv resets CTRL to 0x41, not 0x00.
  localparam logic [31:0] CTRL_RESET_EXPECTED = 32'h0000_0041;

  // ctrl[9:8]=mode_sel, [7:6]=fft_size, [5:3]=ai_task, [2]=bypass_filt,
  // [1]=bypass_disp, [0]=bypass_npu
  localparam logic [31:0] CTRL_SD_BYPASS_NPU = 32'h0000_0041; // mode=SD, fft=1024, ai=0, bypass_npu=1
  localparam logic [31:0] CTRL_SD_DENOISE    = 32'h0000_0048; // mode=SD, fft=1024, ai=1, bypass_npu=0

  localparam int FRAME_LINES  = 1024;
  localparam int LINE_PIXELS  = 2048;
  localparam int DEADLOCK_MAX = 10000;

  // Requested spec says:
  //   - 2 complete frames of 1024x2048 pixels
  //   - overall timeout 2ms
  // Those are mutually inconsistent at 250 MHz.
  // One full frame at 1 pixel / pl_clk0 takes ~8.39 ms.
  localparam time TB_TIMEOUT = 30ms;

  // Requested throughput check says 1M bytes within 500us.
  // At 1 byte/cycle and 250 MHz, 1M bytes needs at least ~4 ms.
  localparam time DATAFLOW_WINDOW = 6ms;
  localparam int  DATAFLOW_MIN_BYTES = 1024*1024;

  // ---------------------------------------------------------------------------
  // Scoreboarding / monitors
  // ---------------------------------------------------------------------------
  integer out_byte_count_total;
  integer out_byte_count_window;
  integer stuck_valid_cycles;
  integer phase4_out_bytes;
  integer phase5_out_bytes;
  bit     saw_output_phase2;
  bit     saw_output_phase4;
  bit     saw_output_phase5;

  string current_phase;
  int    cycle_counter;

  // ---------------------------------------------------------------------------
  // Clock generation
  // ---------------------------------------------------------------------------
  initial begin
    pl_clk0 = 1'b0;
    forever #(PL_CLK0_PER/2) pl_clk0 = ~pl_clk0;
  end

  initial begin
    pl_clk1 = 1'b0;
    forever #(PL_CLK1_PER/2) pl_clk1 = ~pl_clk1;
  end

  initial begin
    adc_clk = 1'b0;
    forever #(ADC_CLK_PER/2) adc_clk = ~adc_clk;
  end

  // ---------------------------------------------------------------------------
  // Timeout
  // ---------------------------------------------------------------------------
  initial begin
    #(TB_TIMEOUT);
    fail("GLOBAL", $sformatf("timeout reached before PASS (TB_TIMEOUT=%0t)", TB_TIMEOUT));
  end

  // ---------------------------------------------------------------------------
  // AXI4-Stream sink and deadlock monitor
  // ---------------------------------------------------------------------------
  always_ff @(posedge pl_clk0) begin
    if (!pl_resetn) begin
      out_byte_count_total  <= 0;
      out_byte_count_window <= 0;
      stuck_valid_cycles    <= 0;
      phase4_out_bytes      <= 0;
      phase5_out_bytes      <= 0;
      saw_output_phase2     <= 0;
      saw_output_phase4     <= 0;
      saw_output_phase5     <= 0;
    end else begin
      if (m_axis_out_tvalid && !m_axis_out_tready) begin
        stuck_valid_cycles <= stuck_valid_cycles + 1;
        if (stuck_valid_cycles > DEADLOCK_MAX) begin
          fail(current_phase,
               $sformatf("AXI4-Stream tvalid stuck high > %0d cycles without tready", DEADLOCK_MAX));
        end
      end else begin
        stuck_valid_cycles <= 0;
      end

      if (m_axis_out_tvalid && m_axis_out_tready) begin
        out_byte_count_total  <= out_byte_count_total + 1;
        out_byte_count_window <= out_byte_count_window + 1;

        if (current_phase == "PHASE2") begin
          saw_output_phase2 <= 1'b1;
        end
        if (current_phase == "PHASE4") begin
          saw_output_phase4 <= 1'b1;
          phase4_out_bytes  <= phase4_out_bytes + 1;
        end
        if (current_phase == "PHASE5") begin
          saw_output_phase5 <= 1'b1;
          phase5_out_bytes  <= phase5_out_bytes + 1;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Simple ADC idle pattern
  // ---------------------------------------------------------------------------
  always_ff @(posedge adc_clk) begin
    if (!pl_resetn) begin
      adc_data   <= '0;
      adc_valid  <= 1'b0;
      mzi_fringe <= 1'b0;
    end else begin
      adc_valid  <= 1'b0;
      adc_data   <= adc_data + 14'd1;
      mzi_fringe <= ~mzi_fringe;
    end
  end

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  task automatic fail(input string phase, input string detail);
    begin
      $display("TB_OCT_TOP: FAIL — %s %s", phase, detail);
      $fatal(1);
    end
  endtask

  task automatic expect_eq32(
    input string phase,
    input logic [31:0] got,
    input logic [31:0] exp,
    input string label
  );
    begin
      if (got !== exp) begin
        fail(phase, $sformatf("%s mismatch: got=0x%08x exp=0x%08x", label, got, exp));
      end
    end
  endtask

  task automatic axi_write32(
    input logic [5:0]  addr,
    input logic [31:0] data
  );
    begin
      @(posedge pl_clk0);
      s_axi_awaddr  <= addr;
      s_axi_awvalid <= 1'b1;
      s_axi_wdata   <= data;
      s_axi_wstrb   <= 4'hF;
      s_axi_wvalid  <= 1'b1;
      s_axi_bready  <= 1'b1;

      while (!(s_axi_awready && s_axi_wready)) begin
        @(posedge pl_clk0);
      end

      @(posedge pl_clk0);
      s_axi_awvalid <= 1'b0;
      s_axi_wvalid  <= 1'b0;

      while (!s_axi_bvalid) begin
        @(posedge pl_clk0);
      end

      if (s_axi_bresp !== 2'b00) begin
        fail(current_phase, $sformatf("AXI write BRESP error at addr 0x%02x: %0b", addr, s_axi_bresp));
      end

      @(posedge pl_clk0);
      s_axi_bready <= 1'b0;
    end
  endtask

  task automatic axi_read32(
    input  logic [5:0]  addr,
    output logic [31:0] data
  );
    begin
      @(posedge pl_clk0);
      s_axi_araddr  <= addr;
      s_axi_arvalid <= 1'b1;
      s_axi_rready  <= 1'b1;

      while (!s_axi_arready) begin
        @(posedge pl_clk0);
      end

      @(posedge pl_clk0);
      s_axi_arvalid <= 1'b0;

      while (!s_axi_rvalid) begin
        @(posedge pl_clk0);
      end

      if (s_axi_rresp !== 2'b00) begin
        fail(current_phase, $sformatf("AXI read RRESP error at addr 0x%02x: %0b", addr, s_axi_rresp));
      end

      data = s_axi_rdata;
      @(posedge pl_clk0);
      s_axi_rready <= 1'b0;
    end
  endtask

  task automatic apply_reset(input int cycles_low);
    begin
      pl_resetn <= 1'b0;
      repeat (cycles_low) @(posedge pl_clk0);
      pl_resetn <= 1'b1;
      repeat (5) @(posedge pl_clk0);
    end
  endtask

  task automatic drive_sd_frames(input int num_frames);
    int f, line, pix;
    begin
      cycle_counter = 0;

      for (f = 0; f < num_frames; f++) begin
        @(posedge pl_clk0);
        cam_fval <= 1'b1;

        for (line = 0; line < FRAME_LINES; line++) begin
          for (pix = 0; pix < LINE_PIXELS; pix++) begin
            @(posedge pl_clk0);
            cam_lval  <= 1'b1;
            cam_valid <= 1'b1;
            cam_pixel <= cycle_counter[15:0];
            cycle_counter++;
          end

          @(posedge pl_clk0);
          cam_lval  <= 1'b0;
          cam_valid <= 1'b0;
          cam_pixel <= '0;
        end

        @(posedge pl_clk0);
        cam_fval  <= 1'b0;
        cam_lval  <= 1'b0;
        cam_valid <= 1'b0;
      end
    end
  endtask

  task automatic dataflow_window_check(
    input string phase,
    input time   window_time,
    input int    min_bytes
  );
    int bytes_before;
    int bytes_after;
    begin
      bytes_before = out_byte_count_total;
      #(window_time);
      bytes_after = out_byte_count_total;

      if ((bytes_after - bytes_before) < min_bytes) begin
        fail(phase,
             $sformatf("insufficient output data in %0t: saw %0d bytes, expected at least %0d",
                       window_time, bytes_after - bytes_before, min_bytes));
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Main sequence
  // ---------------------------------------------------------------------------
  logic [31:0] rd_val;
  logic [31:0] status_val;
  logic [31:0] latency_val;

  initial begin
    current_phase = "INIT";

    // defaults
    s_axi_awaddr   = '0;
    s_axi_awvalid  = 1'b0;
    s_axi_wdata    = '0;
    s_axi_wstrb    = 4'h0;
    s_axi_wvalid   = 1'b0;
    s_axi_bready   = 1'b0;
    s_axi_araddr   = '0;
    s_axi_arvalid  = 1'b0;
    s_axi_rready   = 1'b0;

    cam_pixel      = '0;
    cam_valid      = 1'b0;
    cam_lval       = 1'b0;
    cam_fval       = 1'b0;

    adc_data       = '0;
    adc_valid      = 1'b0;
    mzi_fringe     = 1'b0;

    m_axis_out_tready = 1'b1;
    pl_resetn      = 1'b0;
    cycle_counter  = 0;

    // ---------------------------------------------------------
    // Reset
    // ---------------------------------------------------------
    apply_reset(20);

    // ---------------------------------------------------------
    // PHASE 1 — Bring-up check
    // ---------------------------------------------------------
    current_phase = "PHASE1";

    axi_read32(ADDR_CTRL, rd_val);
    expect_eq32(current_phase, rd_val, CTRL_RESET_EXPECTED, "CTRL after reset");

    axi_write32(ADDR_CTRL, CTRL_SD_BYPASS_NPU);
    axi_read32 (ADDR_CTRL, rd_val);
    expect_eq32(current_phase, rd_val, CTRL_SD_BYPASS_NPU, "CTRL readback");

    // Optional expected A-scan rate for watchdog
    axi_write32(ADDR_ASCAN_RATE, 32'd1000);

    // ---------------------------------------------------------
    // PHASE 2 — SD-OCT pipeline smoke test
    // ---------------------------------------------------------
    current_phase = "PHASE2";
    out_byte_count_window = 0;
    saw_output_phase2     = 0;

    fork
      drive_sd_frames(2);
      dataflow_window_check(current_phase, DATAFLOW_WINDOW, DATAFLOW_MIN_BYTES);
    join

    if (!saw_output_phase2) begin
      fail(current_phase, "no AXI4-Stream output activity observed");
    end

    // ---------------------------------------------------------
    // PHASE 3 — Watchdog responds
    // ---------------------------------------------------------
    current_phase = "PHASE3";

    axi_read32(ADDR_STATUS, status_val);

    // STATUS = {30'b0, overflow, locked}
    if (status_val[0] !== 1'b1) begin
      fail(current_phase, $sformatf("locked bit not set, STATUS=0x%08x", status_val));
    end

    if (status_val[1] !== 1'b0) begin
      fail(current_phase, $sformatf("overflow bit set unexpectedly, STATUS=0x%08x", status_val));
    end

    // ---------------------------------------------------------
    // PHASE 4 — NPU enable (denoising)
    // ---------------------------------------------------------
    current_phase = "PHASE4";
    phase4_out_bytes = 0;
    saw_output_phase4 = 0;

    axi_write32(ADDR_CTRL, CTRL_SD_DENOISE);
    axi_read32 (ADDR_CTRL, rd_val);
    expect_eq32(current_phase, rd_val, CTRL_SD_DENOISE, "CTRL readback after NPU enable");

    drive_sd_frames(1);

    if (!saw_output_phase4 || (phase4_out_bytes == 0)) begin
      fail(current_phase, "no output data observed after enabling NPU");
    end

    axi_read32(ADDR_AI_LATENCY, latency_val);
    if (latency_val == 32'd0) begin
      fail(current_phase, "AI_LATENCY register is zero after NPU run");
    end

    // ---------------------------------------------------------
    // PHASE 5 — Reset recovery
    // ---------------------------------------------------------
    current_phase = "PHASE5";
    phase5_out_bytes = 0;
    saw_output_phase5 = 0;

    fork
      begin
        repeat (1000) @(posedge pl_clk0);
        pl_resetn <= 1'b0;
        repeat (10) @(posedge pl_clk0);
        pl_resetn <= 1'b1;
      end
      begin
        drive_sd_frames(1);
      end
    join

    // Re-program control after reset, since ctrl regs reset too.
    axi_write32(ADDR_CTRL, CTRL_SD_DENOISE);

    drive_sd_frames(1);

    if (!saw_output_phase5 || (phase5_out_bytes == 0)) begin
      fail(current_phase, "output did not resume after mid-stream reset");
    end

    $display("TB_OCT_TOP: PASS");
    $finish;
  end

endmodule