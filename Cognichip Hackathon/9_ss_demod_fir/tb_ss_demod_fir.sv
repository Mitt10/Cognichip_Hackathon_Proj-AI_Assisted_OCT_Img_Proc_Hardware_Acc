`timescale 1ns/1ps

module tb_ss_demod_fir();

    // -----------------------------------------------------------------------
    // Clocks & Resets
    // -----------------------------------------------------------------------
    logic clk = 0;
    logic rst_n = 0;
    
    always #2.0 clk = ~clk; // 250 MHz DSP clock [cite: 220]

    // -----------------------------------------------------------------------
    // DUT Parameters & Signals
    // -----------------------------------------------------------------------
    localparam int TAPS    = 40;
    localparam int DATA_W  = 16;
    localparam int COEFF_W = 16;
    localparam int OUT_W   = 16;
    
    logic [15:0] s_tdata;
    logic        s_tvalid;
    logic        s_tlast;
    logic [31:0] s_tuser;
    logic        s_tready;

    logic [15:0] m_tdata;
    logic        m_tvalid;
    logic        m_tlast;
    logic [31:0] m_tuser;
    logic        m_tready;

    // -----------------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------------
    ss_demod_fir #(
        .TAPS    (TAPS),
        .DATA_W  (DATA_W),
        .COEFF_W (COEFF_W),
        .OUT_W   (OUT_W)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .s_tdata  (s_tdata),
        .s_tvalid (s_tvalid),
        .s_tlast  (s_tlast),
        .s_tuser  (s_tuser),
        .s_tready (s_tready),
        .m_tdata  (m_tdata),
        .m_tvalid (m_tvalid),
        .m_tlast  (m_tlast),
        .m_tuser  (m_tuser),
        .m_tready (m_tready)
    );

    // -----------------------------------------------------------------------
    // Tasks
    // -----------------------------------------------------------------------
    task reset_dut();
        rst_n    = 0;
        s_tdata  = '0;
        s_tvalid = 0;
        s_tlast  = 0;
        s_tuser  = '0;
        m_tready = 1;
        
        #20 rst_n = 1;
        #10;
    endtask

    // Injects a single spike (impulse) followed by zeros to observe the FIR
    // coefficients outputting one by one.
    task send_impulse(logic [1:0] mode_sel);
        int flush_cycles = TAPS + 30; // Enough zeros to flush the 20-cycle pipeline + 40 taps
        
        $display("[%0t] Sending Impulse in Mode: %b", $time, mode_sel);
        
        // s_tuser bits [3:2] determine the mode [cite: 237]
        s_tuser  = {28'd0, mode_sel, 2'b00};
        
        // 1. Send the impulse peak
        s_tvalid = 1;
        s_tdata  = 16'sh7FFF; // Max positive signed 16-bit
        s_tlast  = 0;
        
        do begin @(posedge clk); end while (!s_tready);
        
        // 2. Send zeros to flush the pipeline and observe the tap cascade
        s_tdata = 16'sh0000;
        for (int i = 0; i < flush_cycles; i++) begin
            s_tlast = (i == flush_cycles - 1);
            do begin @(posedge clk); end while (!s_tready);
        end
        
        s_tvalid = 0;
        s_tlast  = 0;
    endtask

    // -----------------------------------------------------------------------
    // Main Test Sequence
    // -----------------------------------------------------------------------
    initial begin
        $display("Starting SS Demod FIR Simulation...");
        reset_dut();

        // Test 1: SS-OCT Mode (FIR Filtering) [cite: 220]
        // Mode = 2'b01. You should see the Hamming-windowed sinc coefficients 
        // appear on m_tdata as the impulse propagates through the delay line.
        $display("\n--- Test 1: SS-OCT Mode (FIR Active) ---");
        send_impulse(2'b01);
        #100;

        // Test 2: SD-OCT Mode (Bypass) 
        // Mode = 2'b00. The impulse (0x7FFF) should appear exactly 20 valid 
        // clock cycles later, unchanged.
        $display("\n--- Test 2: SD-OCT Mode (Bypass) ---");
        send_impulse(2'b00);
        #100;

        // Test 3: Backpressure handling
        // Assert m_tready = 0 to stall the pipeline midway. Since in_fire
        // gates the delay line[cite: 244], no data should be lost.
        $display("\n--- Test 3: Downstream Backpressure (SS-OCT Mode) ---");
        fork
            send_impulse(2'b01);
            begin
                #50;
                $display("[%0t] >> Stalling downstream (m_tready = 0)", $time);
                m_tready = 0;
                #40; // Stall for 20 clock cycles
                $display("[%0t] >> Releasing stall (m_tready = 1)", $time);
                m_tready = 1;
            end
        join

        #100;
        $display("\nSimulation Complete.");
        $finish;
    end

    // -----------------------------------------------------------------------
    // Output Monitor
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (m_tvalid && m_tready) begin
            if (m_tdata != 16'h0000) begin
                $display("[%0t] AXI-S Out -> Data: %0d (0x%h) | TLAST: %b", 
                         $time, $signed(m_tdata), m_tdata, m_tlast);
            end
        end
    end

endmodule