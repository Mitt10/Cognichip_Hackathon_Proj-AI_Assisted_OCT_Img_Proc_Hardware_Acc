`timescale 1ns/1ps

module tb_modality_mux();

    // -----------------------------------------------------------------------
    // Clocks & Resets
    // -----------------------------------------------------------------------
    logic clk = 0;
    logic rst_n = 0;
    
    always #5 clk = ~clk; // 100 MHz clock

    // -----------------------------------------------------------------------
    // Signals
    // -----------------------------------------------------------------------
    logic [1:0]  mode_sel;

    // SD-OCT Interface
    logic [15:0] sd_tdata;
    logic        sd_tvalid;
    logic        sd_tlast;
    logic [31:0] sd_tuser;
    logic        sd_tready;

    // SS-OCT Interface
    logic [15:0] ss_tdata;
    logic        ss_tvalid;
    logic        ss_tlast;
    logic [31:0] ss_tuser;
    logic        ss_tready;

    // Output Interface
    logic [15:0] m_tdata;
    logic        m_tvalid;
    logic        m_tlast;
    logic [31:0] m_tuser;
    logic        m_tready;

    // -----------------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------------
    modality_mux dut (.*);

    // -----------------------------------------------------------------------
    // Stimulus Tasks
    // -----------------------------------------------------------------------
    task reset_dut();
        rst_n     = 0;
        mode_sel  = 2'b00;
        
        sd_tdata  = 0; sd_tvalid = 0; sd_tlast = 0; sd_tuser = 0;
        ss_tdata  = 0; ss_tvalid = 0; ss_tlast = 0; ss_tuser = 0;
        m_tready  = 1;
        
        #20 rst_n = 1;
        #10;
    endtask

    // Helper task to send an SD-OCT packet
    task send_sd_packet(int length);
        for (int i = 0; i < length; i++) begin
            sd_tdata  = 16'h5D00 + i; // 5D prefix to easily spot SD data
            sd_tuser  = i;
            sd_tvalid = 1;
            sd_tlast  = (i == length - 1);
            
            // Wait until the DUT accepts the data
            do begin
                @(posedge clk);
            end while (!sd_tready);
        end
        sd_tvalid = 0;
        sd_tlast  = 0;
    endtask

    // Helper task to send an SS-OCT packet
    task send_ss_packet(int length);
        for (int i = 0; i < length; i++) begin
            ss_tdata  = 16'h5500 + i; // 55 prefix to easily spot SS data
            ss_tuser  = i;
            ss_tvalid = 1;
            ss_tlast  = (i == length - 1);
            
            // Wait until the DUT accepts the data
            do begin
                @(posedge clk);
            end while (!ss_tready);
        end
        ss_tvalid = 0;
        ss_tlast  = 0;
    endtask

    // -----------------------------------------------------------------------
    // Main Test Sequence
    // -----------------------------------------------------------------------
    initial begin
        $display("Starting Modality MUX Simulation...");
        reset_dut();

        // Test 1: SD-OCT routing (Default mode_sel = 00)
        $display("\n--- Test 1: SD-OCT Packet ---");
        send_sd_packet(4);
        #30;

        // Test 2: Mode switch glitch prevention
        // We will start an SD-OCT packet, and halfway through, change mode_sel to SS-OCT.
        // The MUX should ignore the change until the SD-OCT packet finishes.
        $display("\n--- Test 2: Mid-Packet Mode Switch ---");
        fork
            send_sd_packet(6);
            begin
                #25;
                $display("[%0t] >> Requesting Mode switch to SS-OCT (1) mid-packet!", $time);
                mode_sel = 2'b01;
            end
        join
        
        #30;

        // Test 3: SS-OCT routing (Mode is now 01)
        $display("\n--- Test 3: SS-OCT Packet ---");
        send_ss_packet(4);
        #30;

        // Test 4: Backpressure Testing
        // Stall the downstream consumer and ensure no data is lost.
        $display("\n--- Test 4: Downstream Backpressure ---");
        m_tready = 0; // Simulate downstream stall
        fork
            send_ss_packet(5);
            begin
                #45;
                $display("[%0t] >> Releasing downstream backpressure (m_tready = 1)", $time);
                m_tready = 1;
            end
        join

        #50;
        $display("\nSimulation Complete.");
        $finish;
    end

    // -----------------------------------------------------------------------
    // Output Monitor
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (m_tvalid && m_tready) begin
            $display("[%0t] AXI-S Output >> Data: %h | TLAST: %b | TUSER: %0d", 
                     $time, m_tdata, m_tlast, m_tuser);
        end
    end

endmodule