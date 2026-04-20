`timescale 1ns/1ps

module tb_ss_oct_frontend();

    // -----------------------------------------------------------------------
    // Clock Periods & Parameters
    // -----------------------------------------------------------------------
    localparam ADC_CLK_PERIOD = 2.0; // 500 MHz
    localparam SYS_CLK_PERIOD = 4.0; // 250 MHz
    
    // -----------------------------------------------------------------------
    // Signals
    // -----------------------------------------------------------------------
    logic        adc_clk;
    logic        sys_clk;
    logic        rst_n;

    logic [13:0] adc_data;
    logic        adc_valid;
    logic        mzi_fringe;

    logic        galvo_sclk;
    logic        galvo_cs_n;
    logic        galvo_mosi;
    logic        laser_trig;

    logic [10:0] ascan_depth;
    logic [15:0] sweep_period;
    logic [15:0] galvo_x_step;
    logic [15:0] galvo_y_step;

    logic [15:0] m_tdata;
    logic        m_tvalid;
    logic        m_tlast;
    logic [31:0] m_tuser;
    logic        m_tready;

    // -----------------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------------
    ss_oct_frontend dut (
        .adc_clk      (adc_clk),
        .sys_clk      (sys_clk),
        .rst_n        (rst_n),
        .adc_data     (adc_data),
        .adc_valid    (adc_valid),
        .mzi_fringe   (mzi_fringe),
        .galvo_sclk   (galvo_sclk),
        .galvo_cs_n   (galvo_cs_n),
        .galvo_mosi   (galvo_mosi),
        .laser_trig   (laser_trig),
        .ascan_depth  (ascan_depth),
        .sweep_period (sweep_period),
        .galvo_x_step (galvo_x_step),
        .galvo_y_step (galvo_y_step),
        .m_tdata      (m_tdata),
        .m_tvalid     (m_tvalid),
        .m_tlast      (m_tlast),
        .m_tuser      (m_tuser),
        .m_tready     (m_tready)
    );

    // -----------------------------------------------------------------------
    // Clock Generation
    // -----------------------------------------------------------------------
    initial begin
        adc_clk = 0;
        forever #(ADC_CLK_PERIOD / 2.0) adc_clk = ~adc_clk;
    end

    initial begin
        sys_clk = 0;
        forever #(SYS_CLK_PERIOD / 2.0) sys_clk = ~sys_clk;
    end

    // -----------------------------------------------------------------------
    // Stimulus Tasks
    // -----------------------------------------------------------------------
    task reset_dut();
        rst_n        = 0;
        adc_data     = '0;
        adc_valid    = 0;
        mzi_fringe   = 0;
        m_tready     = 1;
        
        // Configuration setup
        ascan_depth  = 11'd10;     // Short depth for fast simulation
        sweep_period = 16'd1000;
        galvo_x_step = 16'h0010;   // 16 in 16.0 fp
        galvo_y_step = 16'h0020;   // 32 in 16.0 fp

        #(SYS_CLK_PERIOD * 10);
        rst_n = 1;
        #(SYS_CLK_PERIOD * 5);
    endtask

    // Simulates an MZI fringe clock and corresponding ADC sample
    task drive_k_clock_sample(logic [13:0] sample_val);
        // Assert MZI Fringe (Rising Edge)
        @(posedge adc_clk);
        mzi_fringe = 1;
        adc_data   = sample_val;
        adc_valid  = 1;
        
        // Deassert MZI Fringe (Falling Edge)
        @(posedge adc_clk);
        mzi_fringe = 0;
        adc_valid  = 0;
        
        // Wait a few cycles between fringes to mimic real optical behavior
        #(ADC_CLK_PERIOD * 3);
    endtask

    task drive_full_ascan();
        $display("[%0t] Starting A-Scan...", $time);
        for (int i = 0; i < ascan_depth; i++) begin
            drive_k_clock_sample(14'(i + 100)); // Arbitrary ADC data sequence
        end
    endtask

    // -----------------------------------------------------------------------
    // Main Test Sequence
    // -----------------------------------------------------------------------
    initial begin
        $display("==================================================");
        $display(" Starting SS-OCT Frontend Simulation");
        $display("==================================================");

        reset_dut();

        // 1. Drive first A-Scan
        // This should trigger AXI stream data, m_tlast on the 10th sample, 
        // and kick off the SPI transaction for Galvo X.
        drive_full_ascan();

        // Wait enough time for the slow SPI transaction to complete
        // 16 bits * 8 sys_clk cycles = 128 sys_clks = 512 ns
        #(SYS_CLK_PERIOD * 150);

        // 2. Drive second A-Scan
        drive_full_ascan();

        // Wait for SPI transaction again
        #(SYS_CLK_PERIOD * 150);

        $display("==================================================");
        $display(" Test Complete. Please inspect waveforms.");
        $display("==================================================");
        $finish;
    end

    // -----------------------------------------------------------------------
    // Simple AXI-Stream & SPI Monitors
    // -----------------------------------------------------------------------
    always @(posedge sys_clk) begin
        if (m_tvalid && m_tready) begin
            $display("[%0t] AXI-S Read  -> Data: %0h | TLAST: %b | TUSER: %0h", 
                     $time, m_tdata, m_tlast, m_tuser);
        end
    end

    always @(posedge sys_clk) begin
        if (m_tlast && m_tvalid && m_tready) begin
            $display("[%0t] A-Scan Complete (TLAST fired)!", $time);
        end
    end

endmodule