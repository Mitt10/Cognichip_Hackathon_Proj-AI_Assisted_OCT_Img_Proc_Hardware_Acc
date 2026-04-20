`timescale 1ns/1ps

// Mock package for standalone compilation (remove if using your actual oct_pkg)
package oct_pkg;
    localparam int unsigned ASCAN_SAMPLES = 1024;
endpackage

module tb_dc_subtract();
    import oct_pkg::*;

    // -----------------------------------------------------------------------
    // Clocks & Resets
    // -----------------------------------------------------------------------
    logic clk = 0;
    logic rst_n = 0;
    
    always #5 clk = ~clk; // 100 MHz clock

    // -----------------------------------------------------------------------
    // Signals
    // -----------------------------------------------------------------------
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

    logic [7:0]  avg_shift;

    // -----------------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------------
    dc_subtract dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .s_tdata    (s_tdata),
        .s_tvalid   (s_tvalid),
        .s_tlast    (s_tlast),
        .s_tuser    (s_tuser),
        .s_tready   (s_tready),
        .m_tdata    (m_tdata),
        .m_tvalid   (m_tvalid),
        .m_tlast    (m_tlast),
        .m_tuser    (m_tuser),
        .m_tready   (m_tready),
        .avg_shift  (avg_shift)
    );

    // -----------------------------------------------------------------------
    // Tasks
    // -----------------------------------------------------------------------
    task reset_dut();
        rst_n     = 0;
        s_tvalid  = 0;
        s_tdata   = 0;
        s_tlast   = 0;
        s_tuser   = 0;
        m_tready  = 1;
        avg_shift = 8'd0; // Immediate update (bg adapts instantly)
        
        #20 rst_n = 1;
        #10;
    endtask

    // Simulates an entire A-Scan with a constant DC offset and some alternating AC data
    task send_ascan(int base_dc, int ascan_id);
        $display("[%0t] Sending A-Scan ID: %0d with DC offset: %0d", $time, ascan_id, base_dc);
        
        for (int i = 0; i < ASCAN_SAMPLES; i++) begin
            s_tdata  = 16'(base_dc + (i % 10)); // DC offset + small AC pattern
            s_tuser  = {16'(ascan_id), 16'(i)};
            s_tvalid = 1;
            s_tlast  = (i == ASCAN_SAMPLES - 1); // Assert on the last sample
            
            // Wait for handshake
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
        $display("Starting DC Subtract Simulation...");
        reset_dut();

        // Test 1: First A-Scan (BRAM is uninitialized, likely zeroes if simulated purely, 
        // but let's assume it starts near 0). 
        // With avg_shift = 0, new_bg = bg + (sample - bg) = sample.
        avg_shift = 8'd0;
        send_ascan(500, 1);
        #50;

        // Test 2: Second A-Scan (DC should now be fully removed because avg_shift was 0)
        // Background stored is exactly the previous line. Diff should be roughly 0.
        send_ascan(500, 2);
        #50;

        // Test 3: Slower IIR Update
        // Change avg_shift to 2 (diff >>> 2). Background adapts slowly.
        $display("\n--- Changing avg_shift to 2 ---");
        avg_shift = 8'd2;
        send_ascan(1000, 3); // Large DC step
        #50;
        send_ascan(1000, 4); // Should see DC closer to 0 now
        #50;

        // Test 4: Backpressure Handling
        // Assert m_tready = 0 midway through the frame to test pipeline freezing.
        $display("\n--- Testing Downstream Backpressure ---");
        fork
            send_ascan(1500, 5);
            begin
                // Wait a few cycles, then stall downstream
                #100;
                $display("[%0t] >> Stalling downstream (m_tready = 0)", $time);
                m_tready = 0;
                #100; // Hold stall for 10 clock cycles
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
            // Only print first few and last few samples to avoid console spam
            int sample_idx = m_tuser[15:0];
            if (sample_idx < 3 || sample_idx > ASCAN_SAMPLES - 3) begin
                $display("[%0t] AXI-S Out -> DC-Removed Data: %0d | TLAST: %b | IDX: %0d", 
                         $time, $signed(m_tdata), m_tlast, sample_idx);
            end
        end
    end

endmodule