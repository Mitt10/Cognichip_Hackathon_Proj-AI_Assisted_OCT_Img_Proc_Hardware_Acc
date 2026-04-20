`ifndef OCT_PKG_SV
  `include "oct_pkg.sv"
`endif

import oct_pkg::*;

module oct_top (
  // ---------------------------------------------------------------------------
  // Global
  // ---------------------------------------------------------------------------
  input  logic              pl_clk0,        // 250 MHz from PS PLL
  input  logic              pl_clk1,        // 300 MHz from PS PLL
  input  logic              pl_resetn,      // PS-generated active-low reset

  // NOTE:
  // The uploaded ss_oct_frontend and requested u_adc_cdc both require adc_clk,
  // but adc_clk was not listed in the requested I/O list. It is exposed here
  // so the wrapper is synthesizable and the CDC instance is legal.
  input  logic              adc_clk,

  // ---------------------------------------------------------------------------
  // AXI4-Lite slave (PS -> PL control regs), 32-bit data, 6-bit address
  // ---------------------------------------------------------------------------
  input  logic [5:0]        s_axi_awaddr,
  input  logic              s_axi_awvalid,
  output logic              s_axi_awready,
  input  logic [31:0]       s_axi_wdata,
  input  logic [3:0]        s_axi_wstrb,
  input  logic              s_axi_wvalid,
  output logic              s_axi_wready,
  output logic [1:0]        s_axi_bresp,
  output logic              s_axi_bvalid,
  input  logic              s_axi_bready,
  input  logic [5:0]        s_axi_araddr,
  input  logic              s_axi_arvalid,
  output logic              s_axi_arready,
  output logic [31:0]       s_axi_rdata,
  output logic [1:0]        s_axi_rresp,
  output logic              s_axi_rvalid,
  input  logic              s_axi_rready,

  // ---------------------------------------------------------------------------
  // SD-OCT camera interface
  // ---------------------------------------------------------------------------
  input  logic [15:0]       cam_pixel,
  input  logic              cam_valid,
  input  logic              cam_lval,
  input  logic              cam_fval,

  // ---------------------------------------------------------------------------
  // SS-OCT ADC
  // ---------------------------------------------------------------------------
  input  logic [13:0]       adc_data,
  input  logic              adc_valid,
  input  logic              mzi_fringe,

  // ---------------------------------------------------------------------------
  // Galvo / laser
  // ---------------------------------------------------------------------------
  output logic              galvo_sclk,
  output logic              galvo_cs_n,
  output logic              galvo_mosi,
  output logic              laser_trig,

  // ---------------------------------------------------------------------------
  // AXI4-Stream output to PS DMA (250 MHz, 8-bit master)
  // ---------------------------------------------------------------------------
  output logic [7:0]        m_axis_out_tdata,
  output logic              m_axis_out_tvalid,
  output logic              m_axis_out_tlast,
  output logic [31:0]       m_axis_out_tuser,
  input  logic              m_axis_out_tready,

  // ---------------------------------------------------------------------------
  // IRQ
  // ---------------------------------------------------------------------------
  output logic              watchdog_irq
);

  // ===========================================================================
  // Resets
  // ===========================================================================
  logic rst_n_clk0, rst_n_clk1, rst_n_adc;
  assign rst_n_clk0 = pl_resetn;
  assign rst_n_clk1 = pl_resetn;
  assign rst_n_adc  = pl_resetn;

  // ===========================================================================
  // Control / status
  // ===========================================================================
  oct_ctrl_t         ctrl_out;
  logic [1:0]        ctrl_status_in;
  logic [31:0]       ai_latency_cycles;

  logic              wd_overflow_flag;
  logic              wd_locked;
  logic              wd_galvo_drift;

  // ===========================================================================
  // Control-derived wires
  // ===========================================================================
  logic [1:0]        mode_sel;
  logic [1:0]        fft_size_sel;
  logic [2:0]        ai_task_sel;
  logic              bypass_filt;
  logic              bypass_disp;
  logic              bypass_npu;

  assign mode_sel     = ctrl_out.mode_sel;
  assign fft_size_sel = ctrl_out.fft_size;
  assign ai_task_sel  = ctrl_out.ai_task;
  assign bypass_filt  = ctrl_out.bypass_filt;
  assign bypass_disp  = ctrl_out.bypass_disp;
  assign bypass_npu   = ctrl_out.bypass_npu;

  assign ctrl_status_in = {wd_overflow_flag, wd_locked};

  // ===========================================================================
  // Static / calibration / bring-up ties
  // ===========================================================================
  logic [10:0]       sd_roi_start, sd_roi_end;
  logic [3:0]        sd_gain_shift;
  logic [7:0]        sd_galvo_x, sd_galvo_y;

  logic [10:0]       ss_ascan_depth;
  logic [15:0]       ss_sweep_period;
  logic [15:0]       ss_galvo_x_step, ss_galvo_y_step;

  logic [7:0]        dc_avg_shift;
  logic [2:0]        shaping_zone_sel;
  logic              shaping_coeff_we;
  logic [8:0]        shaping_coeff_addr;
  logic [17:0]       shaping_coeff_wdata;

  logic              resamp_lut_we;
  logic [9:0]        resamp_lut_addr;
  logic [19:0]       resamp_lut_wdata;

  logic              disp_phase_we;
  logic [9:0]        disp_phase_addr;
  logic [35:0]       disp_phase_wdata;

  logic [31:0]       drm_min_cfg, drm_max_cfg;
  logic              cordic_en_cfg;

  logic              sched_mc_we;
  logic [4:0]        sched_mc_addr;
  logic [127:0]      sched_mc_wdata;

  logic [19:0]       host_raddr;
  logic [7:0]        host_rdata;
  logic              host_re;

  assign sd_roi_start        = 11'd0;
  assign sd_roi_end          = 11'd1023;
  assign sd_gain_shift       = 4'd0;
  assign sd_galvo_x          = 8'd0;
  assign sd_galvo_y          = 8'd0;

  assign ss_ascan_depth      = 11'd1024;
  assign ss_sweep_period     = 16'd0;
  assign ss_galvo_x_step     = 16'd0;
  assign ss_galvo_y_step     = 16'd0;

  assign dc_avg_shift        = 8'd4;
  assign shaping_zone_sel    = bypass_filt ? 3'd5 : 3'd0;
  assign shaping_coeff_we    = 1'b0;
  assign shaping_coeff_addr  = '0;
  assign shaping_coeff_wdata = '0;

  assign resamp_lut_we       = 1'b0;
  assign resamp_lut_addr     = '0;
  assign resamp_lut_wdata    = '0;

  assign disp_phase_we       = 1'b0;
  assign disp_phase_addr     = '0;
  assign disp_phase_wdata    = '0;

  assign drm_min_cfg         = 32'd0;
  assign drm_max_cfg         = 32'h0000_FFFF;
  assign cordic_en_cfg       = 1'b0;

  assign sched_mc_we         = 1'b0;
  assign sched_mc_addr       = '0;
  assign sched_mc_wdata      = '0;

  // ===========================================================================
  // AXI4-Stream pipeline interconnect
  // ===========================================================================
  logic [15:0]       sd_tdata,       ss_tdata,       mux_tdata,      dc_tdata;
  logic [15:0]       shaping_tdata,  demod_tdata,    resamp_tdata;
  logic [31:0]       disp_tdata,     fft_tdata;
  logic [7:0]        maglog_tdata;

  logic              sd_tvalid,      ss_tvalid,      mux_tvalid,     dc_tvalid;
  logic              shaping_tvalid, demod_tvalid,   resamp_tvalid;
  logic              disp_tvalid,    fft_tvalid,     maglog_tvalid;

  logic              sd_tlast,       ss_tlast,       mux_tlast,      dc_tlast;
  logic              shaping_tlast,  demod_tlast,    resamp_tlast;
  logic              disp_tlast,     fft_tlast,      maglog_tlast;

  logic [31:0]       sd_tuser,       ss_tuser,       mux_tuser,      dc_tuser;
  logic [31:0]       shaping_tuser,  demod_tuser,    resamp_tuser;
  logic [31:0]       disp_tuser,     fft_tuser,      maglog_tuser;

  logic              sd_tready,      ss_tready,      mux_tready,     dc_tready;
  logic              shaping_tready, demod_tready,   resamp_tready;
  logic              disp_tready,    fft_tready,     maglog_tready;

  // ===========================================================================
  // B-scan buffer / NPU side
  // ===========================================================================
  logic              bscan_dsp_tready;
  logic              bscan_dsp_frame_done;
  logic [7:0]        bscan_npu_rdata;
  logic [19:0]       bscan_npu_raddr;
  logic              bscan_npu_re;
  logic [7:0]        bscan_npu_wdata;
  logic [19:0]       bscan_npu_waddr;
  logic              bscan_npu_we;
  logic              bscan_npu_frame_ready;

  logic              sched_start;
  logic              sched_busy;
  logic              sched_done;
  logic              sched_weight_load;
  logic [17:0]       sched_weight_rd_addr;
  logic              sched_act_clear;
  logic              sched_act_valid_out;
  logic [1:0]        sched_act_fn_out;
  logic [7:0]        sched_requant_shift_out;
  logic              sched_pool_en_out;
  logic              sched_act_bank_sel;
  logic [19:0]       sched_act_rd_addr;
  logic [19:0]       sched_act_wr_addr;

  logic [63:0]       wmem_rd_data;
  logic              wmem_rd_valid;
  logic              wmem_load_done;

  logic [7:0]        pe_weight_in [0:15][0:15];
  logic [7:0]        pe_act_in    [0:15];
  logic [31:0]       pe_acc_out   [0:15];
  logic              pe_out_valid;

  logic [31:0]       postop_bias      [0:15];
  logic [15:0]       postop_bn_scale  [0:15];
  logic [15:0]       postop_bn_shift  [0:15];
  logic [7:0]        postop_out_data  [0:15];
  logic              postop_out_valid;

  logic [31:0]       npu_cdc_wr_data, npu_cdc_rd_data;
  logic              npu_cdc_wr_en,   npu_cdc_wr_full;
  logic              npu_cdc_rd_en,   npu_cdc_rd_empty;

  logic [7:0]        out_cdc_wr_data, out_cdc_rd_data;
  logic              out_cdc_wr_en,   out_cdc_wr_full;
  logic              out_cdc_rd_en,   out_cdc_rd_empty;

  // ===========================================================================
  // SS ADC CDC FIFO (requested explicit instance)
  // Note: ss_oct_frontend already contains its own adc_clk->sys_clk FIFO in the
  // uploaded RTL. This explicit CDC instance is therefore left functionally idle
  // to preserve the requested top-level instance list.
  // ===========================================================================
  logic [15:0]       adc_cdc_rd_data;
  logic              adc_cdc_wr_full, adc_cdc_rd_empty;
  logic              adc_cdc_wr_en,   adc_cdc_rd_en;
  logic [15:0]       adc_cdc_wr_data;

  assign adc_cdc_wr_en   = 1'b0;
  assign adc_cdc_wr_data = '0;
  assign adc_cdc_rd_en   = 1'b0;

  // ===========================================================================
  // Error aggregation for watchdog
  // ===========================================================================
  logic [31:0]       tuser_errors_all;

  assign tuser_errors_all =
      ({30'd0, sd_tuser[1:0]})      |
      ({30'd0, ss_tuser[1:0]})      |
      ({30'd0, mux_tuser[1:0]})     |
      ({30'd0, dc_tuser[1:0]})      |
      ({30'd0, shaping_tuser[1:0]}) |
      ({30'd0, demod_tuser[1:0]})   |
      ({30'd0, resamp_tuser[1:0]})  |
      ({30'd0, disp_tuser[1:0]})    |
      ({30'd0, fft_tuser[1:0]})     |
      ({30'd0, maglog_tuser[1:0]});

  // ===========================================================================
  // Lightweight top-level glue
  // ===========================================================================
  assign sched_start     = bscan_npu_frame_ready & ~bypass_npu;

  assign bscan_npu_raddr = sched_act_rd_addr;
  assign bscan_npu_re    = sched_act_valid_out;
  assign bscan_npu_waddr = sched_act_wr_addr;
  assign bscan_npu_we    = postop_out_valid;
  assign bscan_npu_wdata = postop_out_data[0];

  assign host_raddr      = 20'd0;
  assign host_re         = 1'b0;

  assign npu_cdc_wr_en   = 1'b0;
  assign npu_cdc_wr_data = 32'd0;
  assign npu_cdc_rd_en   = 1'b0;

  assign out_cdc_wr_en   = postop_out_valid & ~out_cdc_wr_full;
  assign out_cdc_wr_data = postop_out_data[0];
  assign out_cdc_rd_en   = m_axis_out_tready & ~out_cdc_rd_empty;

  assign m_axis_out_tdata  = bypass_npu ? host_rdata      : out_cdc_rd_data;
  assign m_axis_out_tvalid = bypass_npu ? 1'b0            : ~out_cdc_rd_empty;
  assign m_axis_out_tlast  = 1'b0;
  assign m_axis_out_tuser  = 32'd0;

  genvar gi, gj;
  generate
    for (gi = 0; gi < 16; gi++) begin : G_ACTS
      assign pe_act_in[gi]       = bscan_npu_rdata;
      assign postop_bias[gi]     = 32'd0;
      assign postop_bn_scale[gi] = 16'h7FFF;
      assign postop_bn_shift[gi] = 16'd0;
    end
    for (gi = 0; gi < 16; gi++) begin : G_WROWS
      for (gj = 0; gj < 16; gj++) begin : G_WCOLS
        assign pe_weight_in[gi][gj] = wmem_rd_data[7:0];
      end
    end
  endgenerate

  // ===========================================================================
  // Instantiations
  // ===========================================================================
  oct_ctrl_regs u_ctrl (
    .S_AXI_ACLK      (pl_clk0),
    .S_AXI_ARESETN   (rst_n_clk0),
    .S_AXI_AWADDR    (s_axi_awaddr),
    .S_AXI_AWVALID   (s_axi_awvalid),
    .S_AXI_AWREADY   (s_axi_awready),
    .S_AXI_WDATA     (s_axi_wdata),
    .S_AXI_WSTRB     (s_axi_wstrb),
    .S_AXI_WVALID    (s_axi_wvalid),
    .S_AXI_WREADY    (s_axi_wready),
    .S_AXI_BRESP     (s_axi_bresp),
    .S_AXI_BVALID    (s_axi_bvalid),
    .S_AXI_BREADY    (s_axi_bready),
    .S_AXI_ARADDR    (s_axi_araddr),
    .S_AXI_ARVALID   (s_axi_arvalid),
    .S_AXI_ARREADY   (s_axi_arready),
    .S_AXI_RDATA     (s_axi_rdata),
    .S_AXI_RRESP     (s_axi_rresp),
    .S_AXI_RVALID    (s_axi_rvalid),
    .S_AXI_RREADY    (s_axi_rready),
    .ctrl_out        (ctrl_out),
    .status_in       (ctrl_status_in),
    .ai_latency_in   (ai_latency_cycles)
  );

  sd_oct_frontend u_sd_fe (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .cam_pixel       (cam_pixel),
    .cam_valid       (cam_valid),
    .cam_lval        (cam_lval),
    .cam_fval        (cam_fval),
    .roi_start       (sd_roi_start),
    .roi_end         (sd_roi_end),
    .gain_shift      (sd_gain_shift),
    .m_tdata         (sd_tdata),
    .m_tvalid        (sd_tvalid),
    .m_tlast         (sd_tlast),
    .m_tuser         (sd_tuser),
    .m_tready        (sd_tready),
    .galvo_x         (sd_galvo_x),
    .galvo_y         (sd_galvo_y)
  );

  ss_oct_frontend u_ss_fe (
    .adc_clk         (adc_clk),
    .sys_clk         (pl_clk0),
    .rst_n           (rst_n_adc),
    .adc_data        (adc_data),
    .adc_valid       (adc_valid),
    .mzi_fringe      (mzi_fringe),
    .galvo_sclk      (galvo_sclk),
    .galvo_cs_n      (galvo_cs_n),
    .galvo_mosi      (galvo_mosi),
    .laser_trig      (laser_trig),
    .ascan_depth     (ss_ascan_depth),
    .sweep_period    (ss_sweep_period),
    .galvo_x_step    (ss_galvo_x_step),
    .galvo_y_step    (ss_galvo_y_step),
    .m_tdata         (ss_tdata),
    .m_tvalid        (ss_tvalid),
    .m_tlast         (ss_tlast),
    .m_tuser         (ss_tuser),
    .m_tready        (ss_tready)
  );

  modality_mux u_mux (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .mode_sel        (mode_sel),
    .sd_tdata        (sd_tdata),
    .sd_tvalid       (sd_tvalid),
    .sd_tlast        (sd_tlast),
    .sd_tuser        (sd_tuser),
    .sd_tready       (sd_tready),
    .ss_tdata        (ss_tdata),
    .ss_tvalid       (ss_tvalid),
    .ss_tlast        (ss_tlast),
    .ss_tuser        (ss_tuser),
    .ss_tready       (ss_tready),
    .m_tdata         (mux_tdata),
    .m_tvalid        (mux_tvalid),
    .m_tlast         (mux_tlast),
    .m_tuser         (mux_tuser),
    .m_tready        (mux_tready)
  );

  dc_subtract u_dc (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .s_tdata         (mux_tdata),
    .s_tvalid        (mux_tvalid),
    .s_tlast         (mux_tlast),
    .s_tuser         (mux_tuser),
    .s_tready        (mux_tready),
    .m_tdata         (dc_tdata),
    .m_tvalid        (dc_tvalid),
    .m_tlast         (dc_tlast),
    .m_tuser         (dc_tuser),
    .m_tready        (dc_tready),
    .avg_shift       (dc_avg_shift)
  );

  spectral_shaping u_shaping (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .s_tdata         (dc_tdata),
    .s_tvalid        (dc_tvalid),
    .s_tlast         (dc_tlast),
    .s_tuser         (dc_tuser),
    .s_tready        (dc_tready),
    .m_tdata         (shaping_tdata),
    .m_tvalid        (shaping_tvalid),
    .m_tlast         (shaping_tlast),
    .m_tuser         (shaping_tuser),
    .m_tready        (shaping_tready),
    .zone_sel        (shaping_zone_sel),
    .coeff_we        (shaping_coeff_we),
    .coeff_addr      (shaping_coeff_addr),
    .coeff_wdata     (shaping_coeff_wdata)
  );

  ss_demod_fir u_demod (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .s_tdata         (shaping_tdata),
    .s_tvalid        (shaping_tvalid),
    .s_tlast         (shaping_tlast),
    .s_tuser         (shaping_tuser),
    .s_tready        (shaping_tready),
    .m_tdata         (demod_tdata),
    .m_tvalid        (demod_tvalid),
    .m_tlast         (demod_tlast),
    .m_tuser         (demod_tuser),
    .m_tready        (demod_tready)
  );

  klinear_resampler u_resampler (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .s_tdata         (demod_tdata),
    .s_tvalid        (demod_tvalid),
    .s_tlast         (demod_tlast),
    .s_tuser         (demod_tuser),
    .s_tready        (demod_tready),
    .m_tdata         (resamp_tdata),
    .m_tvalid        (resamp_tvalid),
    .m_tlast         (resamp_tlast),
    .m_tuser         (resamp_tuser),
    .m_tready        (resamp_tready),
    .lut_we          (resamp_lut_we),
    .lut_addr        (resamp_lut_addr),
    .lut_wdata       (resamp_lut_wdata)
  );

  dispersion_comp u_disp (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .s_tdata         (resamp_tdata),
    .s_tvalid        (resamp_tvalid),
    .s_tlast         (resamp_tlast),
    .s_tuser         (resamp_tuser),
    .s_tready        (resamp_tready),
    .m_tdata         (disp_tdata),
    .m_tvalid        (disp_tvalid),
    .m_tlast         (disp_tlast),
    .m_tuser         (disp_tuser),
    .m_tready        (disp_tready),
    .phase_we        (disp_phase_we),
    .phase_addr      (disp_phase_addr),
    .phase_wdata     (disp_phase_wdata),
    .bypass          (bypass_disp)
  );

  oct_fft u_fft (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .s_tdata         (disp_tdata),
    .s_tvalid        (disp_tvalid),
    .s_tlast         (disp_tlast),
    .s_tuser         (disp_tuser),
    .s_tready        (disp_tready),
    .m_tdata         (fft_tdata),
    .m_tvalid        (fft_tvalid),
    .m_tlast         (fft_tlast),
    .m_tuser         (fft_tuser),
    .m_tready        (fft_tready),
    .fft_size        (fft_size_sel)
  );

  mag_log_drm u_maglog (
    .clk             (pl_clk0),
    .rst_n           (rst_n_clk0),
    .s_tdata         (fft_tdata),
    .s_tvalid        (fft_tvalid),
    .s_tlast         (fft_tlast),
    .s_tuser         (fft_tuser),
    .s_tready        (fft_tready),
    .m_tdata         (maglog_tdata),
    .m_tvalid        (maglog_tvalid),
    .m_tlast         (maglog_tlast),
    .m_tuser         (maglog_tuser),
    .m_tready        (bscan_dsp_tready),
    .drm_min         (drm_min_cfg),
    .drm_max         (drm_max_cfg),
    .cordic_en       (cordic_en_cfg)
  );

  bscan_buffer u_bscan_buf (
    .dsp_clk         (pl_clk0),
    .dsp_rst_n       (rst_n_clk0),
    .dsp_tdata       (maglog_tdata),
    .dsp_tvalid      (maglog_tvalid),
    .dsp_tlast       (maglog_tlast),
    .dsp_tuser       (maglog_tuser),
    .dsp_tready      (bscan_dsp_tready),
    .dsp_frame_done  (bscan_dsp_frame_done),
    .npu_clk         (pl_clk1),
    .npu_rst_n       (rst_n_clk1),
    .npu_rdata       (bscan_npu_rdata),
    .npu_raddr       (bscan_npu_raddr),
    .npu_re          (bscan_npu_re),
    .npu_wdata       (bscan_npu_wdata),
    .npu_waddr       (bscan_npu_waddr),
    .npu_we          (bscan_npu_we),
    .npu_frame_ready (bscan_npu_frame_ready),
    .host_raddr      (host_raddr),
    .host_rdata      (host_rdata),
    .host_re         (host_re)
  );

  npu_weight_buf u_wmem (
    .clk             (pl_clk1),
    .rst_n           (rst_n_clk1),
    .dma_waddr       (18'd0),
    .dma_wdata       (64'd0),
    .dma_we          (1'b0),
    .rd_addr         (sched_weight_rd_addr),
    .rd_data         (wmem_rd_data),
    .rd_en           (sched_weight_load),
    .rd_valid        (wmem_rd_valid),
    .load_done       (wmem_load_done)
  );

  npu_pe_array u_pe (
    .clk             (pl_clk1),
    .rst_n           (rst_n_clk1),
    .weight_in       (pe_weight_in),
    .weight_load     (sched_weight_load),
    .act_in          (pe_act_in),
    .act_valid       (sched_act_valid_out),
    .act_last        (1'b0),
    .acc_out         (pe_acc_out),
    .out_valid       (pe_out_valid),
    .acc_clear       (sched_act_clear)
  );

  npu_postop u_postop (
    .clk             (pl_clk1),
    .rst_n           (rst_n_clk1),
    .acc_in          (pe_acc_out),
    .acc_valid       (pe_out_valid),
    .bias            (postop_bias),
    .bn_scale        (postop_bn_scale),
    .bn_shift        (postop_bn_shift),
    .requant_shift   (sched_requant_shift_out),
    .act_fn          (sched_act_fn_out),
    .pool_en         (sched_pool_en_out),
    .out_data        (postop_out_data),
    .out_valid       (postop_out_valid)
  );

  npu_scheduler u_sched (
    .clk               (pl_clk1),
    .rst_n             (rst_n_clk1),
    .start             (sched_start),
    .task_sel          (ai_task_sel),
    .busy              (sched_busy),
    .done              (sched_done),
    .mc_we             (sched_mc_we),
    .mc_addr           (sched_mc_addr),
    .mc_wdata          (sched_mc_wdata),
    .weight_load       (sched_weight_load),
    .weight_rd_addr    (sched_weight_rd_addr),
    .act_clear         (sched_act_clear),
    .act_valid_out     (sched_act_valid_out),
    .act_fn_out        (sched_act_fn_out),
    .requant_shift_out (sched_requant_shift_out),
    .pool_en_out       (sched_pool_en_out),
    .act_bank_sel      (sched_act_bank_sel),
    .act_rd_addr       (sched_act_rd_addr),
    .act_wr_addr       (sched_act_wr_addr),
    .inference_cycles  (ai_latency_cycles)
  );

  oct_watchdog u_watchdog (
    .clk               (pl_clk0),
    .rst_n             (rst_n_clk0),
    .ascan_tlast       (maglog_tlast & maglog_tvalid & bscan_dsp_tready),
    .bscan_frame_done  (bscan_dsp_frame_done),
    .expected_ascan_rate (32'd0),
    .tuser_errors      (tuser_errors_all),
    .overflow_flag     (wd_overflow_flag),
    .locked            (wd_locked),
    .galvo_drift       (wd_galvo_drift),
    .irq               (watchdog_irq)
  );

  async_fifo #(
    .DATA_WIDTH      (16),
    .DEPTH           (32)
  ) u_adc_cdc (
    .wr_clk          (adc_clk),
    .wr_rst_n        (rst_n_adc),
    .wr_en           (adc_cdc_wr_en),
    .wr_data         (adc_cdc_wr_data),
    .wr_full         (adc_cdc_wr_full),
    .rd_clk          (pl_clk0),
    .rd_rst_n        (rst_n_clk0),
    .rd_en           (adc_cdc_rd_en),
    .rd_data         (adc_cdc_rd_data),
    .rd_empty        (adc_cdc_rd_empty)
  );

  async_fifo #(
    .DATA_WIDTH      (32),
    .DEPTH           (16)
  ) u_npu_cdc (
    .wr_clk          (pl_clk0),
    .wr_rst_n        (rst_n_clk0),
    .wr_en           (npu_cdc_wr_en),
    .wr_data         (npu_cdc_wr_data),
    .wr_full         (npu_cdc_wr_full),
    .rd_clk          (pl_clk1),
    .rd_rst_n        (rst_n_clk1),
    .rd_en           (npu_cdc_rd_en),
    .rd_data         (npu_cdc_rd_data),
    .rd_empty        (npu_cdc_rd_empty)
  );

  async_fifo #(
    .DATA_WIDTH      (8),
    .DEPTH           (512)
  ) u_out_cdc (
    .wr_clk          (pl_clk1),
    .wr_rst_n        (rst_n_clk1),
    .wr_en           (out_cdc_wr_en),
    .wr_data         (out_cdc_wr_data),
    .wr_full         (out_cdc_wr_full),
    .rd_clk          (pl_clk0),
    .rd_rst_n        (rst_n_clk0),
    .rd_en           (out_cdc_rd_en),
    .rd_data         (out_cdc_rd_data),
    .rd_empty        (out_cdc_rd_empty)
  );

  // ===========================================================================
  // Vivado constraints hint
  // ===========================================================================
  // set_clock_groups -asynchronous \
  //   -group [get_clocks pl_clk0] \
  //   -group [get_clocks pl_clk1] \
  //   -group [get_clocks adc_clk]
  //
  // set_max_delay -datapath_only <N> \
  //   -from [get_pins u_adc_cdc/*]  -to [get_pins u_adc_cdc/*]
  // set_max_delay -datapath_only <N> \
  //   -from [get_pins u_npu_cdc/*]  -to [get_pins u_npu_cdc/*]
  // set_max_delay -datapath_only <N> \
  //   -from [get_pins u_out_cdc/*]  -to [get_pins u_out_cdc/*]

endmodule