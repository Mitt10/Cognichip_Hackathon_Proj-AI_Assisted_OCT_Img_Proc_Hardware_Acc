`timescale 1ns/1ps

module tb_spectral_shaping();

    // -----------------------------------------------------------------------
    // Clocks & Resets
    // -----------------------------------------------------------------------
    logic clk = 0;
    logic rst_n = 0;
    
    always #2.0 clk = ~clk; // 250 MHz clock

    // -----------------------------------------------------------------------
    // Parameters & Signals
    // -----------------------------------------------------------------------
    localparam int N_ZONES    = 5;
    localparam int SOS_STAGES = 3;
    localparam int DATA_W     = 16;
    localparam int COEFF_W    = 18;
    localparam int STATE_W    = 32;

    // AXI-Stream Slave (Input)
    logic [DATA_W-1:0] s_tdata;
    logic              s_tvalid;
    logic              s_tlast;
    logic [31:0]       s_tuser;
    logic              s_tready;

    // AXI-Stream Master (Output)
    logic [DATA_W-1:0] m_tdata;
    logic              m_tvalid;
    logic              m_tlast;
    logic [31:0]       m_tuser;
    logic              m_tready;

    // Control & BRAM
    logic [2:0]        zone_sel;
    logic              coeff_we;
    logic [8:0]        coeff_addr;
    logic [COEFF_W-1:0] coeff_wdata;

    // -----------------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------------
    spectral_shaping #(
        .N_ZONES    (N_ZONES),
        .SOS_STAGES (SOS_STAGES),
        .DATA_W     (DATA_W),
        .COEFF_W    (COEFF_W),
        .STATE_W    (STATE_W)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .s_tdata     (s_tdata),
        .s_tvalid    (s_tvalid),
        .s_tlast     (s_tlast),
        .s_tuser     (s_tuser),
        .s_tready    (s_tready),
        .m_tdata     (m_tdata),
        .m_tvalid    (m_tvalid),
        .m_tlast     (m_tlast),
        .m_tuser     (m_tuser),
        .m_tready    (m_tready),
        .zone_sel    (zone_sel),
        .coeff_we    (coeff_we),
        .coeff_addr  (coeff_addr),
        .coeff_wdata (coeff_wdata)
    );

    // -----------------------------------------------------------------------
    // Tasks
    // -----------------------------------------------------------------------
    task reset_dut();
        rst_n       = 0;
        s_tdata     = '0;
        s_tvalid    = 0;
        s_tlast     = 0;
        s_tuser     = '0;
        m_tready    = 1;
        zone_sel    = 3'd5; // Default to bypass
        coeff_we    = 0;
        coeff_addr  = '0;
        coeff_wdata = '0;
        
        #20 rst_n = 1;
        #10;
    endtask

    // Writes an identity filter (gain = 1.0, all other coeffs = 0) to a specific zone.
    // Q1.17 format: 1.0 = 18'h20000. 
    task load_identity_coeffs(int target_zone);
        int base_addr = target_zone * (SOS_STAGES * 5);
        $display("[%0t] Loading Identity Coefficients for Zone %0d", $time, target_zone);
        
        @(posedge clk);
        for (int stage = 0; stage < SOS_STAGES; stage++) begin
            for (int c = 0; c < 5; c++) begin
                coeff_we    = 1;
                coeff_addr  = base_addr + (stage * 5) + c;
                
                // Set b0 (index 0) to 1.0, everything else to 0.0
                if (c == 0) coeff_wdata = 18'h20000; 
                else        coeff_wdata = 18'h00000;
                
                @(posedge clk);
            end
        end
        coeff_we = 0;
        @(posedge clk);
    endtask

    // Sends a short packet of data
    task send_data_packet(int length, logic [15:0] base_val);
        for (int i = 0; i < length; i++) begin
            s_tdata  = base_val + i;
            s_tuser  = i;
            s_tvalid = 1;
            s_tlast  = (i == length - 1);
            
            do begin
                @(posedge clk);
            end while (!s_tready);
        end
        s_tvalid = 0;
        s_tlast  = 0;
    endtask

    // -----------------------------------------------------------------------
    // Main Test Sequence
    // -----------------------------------------------------------------------
    initial begin
        $display("Starting Spectral Shaping Simulation...");
        reset_dut();

        // 1. Pre-load BRAM coefficients for Zone 0
        load_identity_coeffs(0);
        #30;

        // 2. Test Bypass Mode (zone_sel == 5)
        $display("\n--- Test 1: Bypass Mode (Zone 5) ---");
        zone_sel = 3'd5; 
        #20; // Let pipeline settle
        send_data_packet(4, 16'h1000);
        
        // Wait for data to flush (Bypass latency is 2 cycles + 1 output reg) 
        #50;

        // 3. Test Filter Mode (zone_sel == 0)
        $display("\n--- Test 2: Filter Mode (Zone 0) ---");
        zone_sel = 3'd0;
        
        // IMPORTANT: Must wait at least 15 clock cycles for the coefficient 
        // cache to load from BRAM before sending data. 
        $display("[%0t] Waiting for coefficient cache to load...", $time);
        #40; 
        
        // Because we loaded an identity filter, the data should pass through 
        // unchanged, but with a 6-cycle SOS latency + 1 output reg. 
        send_data_packet(4, 16'h2000);
        #80;

        // 4. Test Downstream Backpressure
        $display("\n--- Test 3: Downstream Backpressure ---");
        m_tready = 0; // Stall downstream
        
        fork
            send_data_packet(5, 16'h3000);
            begin
                #30;
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
            $display("[%0t] AXI-S Out -> Data: %0h | TLAST: %b | TUSER: %0d", 
                     $time, m_tdata, m_tlast, m_tuser);
        end
    end

endmodule