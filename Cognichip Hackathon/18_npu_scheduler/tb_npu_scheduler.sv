`timescale 1ns/1ps

module tb_npu_scheduler;

    // =========================================================================
    // Parameters & Signals
    // =========================================================================
    localparam real CLK_PERIOD = 3.333; // ~300 MHz
    
    logic        clk;
    logic        rst_n;
    logic        start;
    logic [2:0]  task_sel;
    logic        mc_we;
    logic [7:0]  mc_addr;
    logic [63:0] mc_wdata;
    
    logic        busy;
    logic        done;
    logic [15:0] weight_rd_addr;
    logic [1:0]  act_fn_out;
    logic [31:0] inference_cycles;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    npu_scheduler dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .task_sel(task_sel),
        .mc_we(mc_we),
        .mc_addr(mc_addr),
        .mc_wdata(mc_wdata),
        .busy(busy),
        .done(done),
        .weight_rd_addr(weight_rd_addr),
        .act_fn_out(act_fn_out),
        .inference_cycles(inference_cycles)
    );

    // =========================================================================
    // Clock Generation
    // =========================================================================
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2.0) clk = ~clk;
    end

    // =========================================================================
    // Global Timeout (50,000 cycles)
    // =========================================================================
    initial begin
        #(50000 * CLK_PERIOD);
        $display("TB_SCHEDULER: FAIL — Global Timeout");
        $finish;
    end

    // =========================================================================
    // Helper Tasks
    // =========================================================================
    // Packs microcode. Assuming format: {Reserved[30:0], act_fn[1:0], bias_base[14:0], weight_base[15:0]}
    task write_microcode(
        input [7:0]  addr,
        input [15:0] w_base,
        input [14:0] b_base,
        input [1:0]  act
    );
        begin
            @(posedge clk);
            mc_we    <= 1'b1;
            mc_addr  <= addr;
            mc_wdata <= {31'd0, act, b_base, w_base};
            @(posedge clk);
            mc_we    <= 1'b0;
        end
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    int timeout_ctr;
    int first_run_cycles;
    int done_pulse_count;
    logic [15:0] prev_weight_addr;

    initial begin
        // ---------------------------------------------------------------------
        // Reset & Setup
        // ---------------------------------------------------------------------
        rst_n    = 0;
        start    = 0;
        task_sel = 0;
        mc_we    = 0;
        mc_addr  = 0;
        mc_wdata = 0;

        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);

        // Load Task 1 (Denoise) Microcode
        // Offset 0 (Task 1 base = 1 * 0 = 0, assuming base offset scheme, or standard 0 offset)
        write_microcode(8'd0, 16'd0,   15'd0,  2'b01); // Layer 0: W=0,   B=0,  ReLU
        write_microcode(8'd1, 16'd100, 15'd50, 2'b01); // Layer 1: W=100, B=50, ReLU
        write_microcode(8'd2, 16'd200, 15'd75, 2'b00); // Layer 2: W=200, B=75, Linear

        // ---------------------------------------------------------------------
        // TEST 1: FSM completes and done pulses
        // ---------------------------------------------------------------------
        @(posedge clk);
        start    <= 1'b1;
        task_sel <= 3'd1;
        @(posedge clk);
        start    <= 1'b0;

        // Check busy goes high within 2 cycles
        repeat(2) @(posedge clk);
        if (!busy) begin
            $display("TB_SCHEDULER: FAIL — TEST 1 (busy did not assert)");
            $finish;
        end

        // Wait for done to pulse (timeout 10000) while monitoring TEST 2 & 3
        timeout_ctr = 0;
        done_pulse_count = 0;
        prev_weight_addr = 0;

        while (timeout_ctr < 10000) begin
            @(posedge clk);
            
            // TEST 2 & 3: Monitoring logic during active execution
            if (busy) begin
                // Check address monotonicity within layers
                if (weight_rd_addr < prev_weight_addr && 
                   !(weight_rd_addr == 100 && prev_weight_addr < 100) && 
                   !(weight_rd_addr == 200 && prev_weight_addr < 200)) begin
                    $display("TB_SCHEDULER: FAIL — TEST 2 (addresses not increasing within layer)");
                    $finish;
                end
                prev_weight_addr = weight_rd_addr;

                // Check post-op parameters per layer
                if (weight_rd_addr < 100) begin
                    if (act_fn_out !== 2'b01) begin
                        $display("TB_SCHEDULER: FAIL — TEST 3 (Layer 0 expected ReLU)");
                        $finish;
                    end
                end else if (weight_rd_addr >= 100 && weight_rd_addr < 200) begin
                    if (act_fn_out !== 2'b01) begin
                        $display("TB_SCHEDULER: FAIL — TEST 3 (Layer 1 expected ReLU)");
                        $finish;
                    end
                end else if (weight_rd_addr >= 200) begin
                    if (act_fn_out !== 2'b00) begin
                        $display("TB_SCHEDULER: FAIL — TEST 3 (Layer 2 expected Linear)");
                        $finish;
                    end
                end
            end

            // Done pulse detection
            if (done) begin
                done_pulse_count++;
                if (done_pulse_count > 1) begin
                    $display("TB_SCHEDULER: FAIL — TEST 1 (done pulsed more than once)");
                    $finish;
                end
                
                // Assert busy goes low after done
                @(posedge clk);
                if (busy) begin
                    $display("TB_SCHEDULER: FAIL — TEST 1 (busy did not drop after done)");
                    $finish;
                end
                break;
            end
            
            timeout_ctr++;
        end

        if (timeout_ctr >= 10000) begin
            $display("TB_SCHEDULER: FAIL — TEST 1 (timeout waiting for done)");
            $finish;
        end

        // ---------------------------------------------------------------------
        // TEST 4: inference_cycles counter
        // ---------------------------------------------------------------------
        first_run_cycles = inference_cycles;
        if (first_run_cycles <= 0 || first_run_cycles >= 10000) begin
            $display("TB_SCHEDULER: FAIL — TEST 4 (cycles out of bounds: %0d)", first_run_cycles);
            $finish;
        end

        // Run second time for determinism
        repeat(5) @(posedge clk);
        start    <= 1'b1;
        task_sel <= 3'd1;
        @(posedge clk);
        start    <= 1'b0;

        wait(done);
        @(posedge clk);

        if (inference_cycles !== first_run_cycles) begin
            $display("TB_SCHEDULER: FAIL — TEST 4 (non-deterministic cycles: %0d vs %0d)", 
                     first_run_cycles, inference_cycles);
            $finish;
        end

        // ---------------------------------------------------------------------
        // TEST 5: Task switch
        // ---------------------------------------------------------------------
        repeat(5) @(posedge clk);

        // Load Task 4 (Classify) Microcode at offset 32 (4 * 8)
        write_microcode(8'd32, 16'd500, 15'd10, 2'b01); // Layer 0: W=500
        write_microcode(8'd33, 16'd600, 15'd20, 2'b00); // Layer 1: W=600

        @(posedge clk);
        start    <= 1'b1;
        task_sel <= 3'd4;
        @(posedge clk);
        start    <= 1'b0;

        // Wait for start of processing
        wait(busy);
        @(posedge clk);

        // Check if weight base matches Task 4, Layer 0
        if (weight_rd_addr !== 16'd500) begin
            $display("TB_SCHEDULER: FAIL — TEST 5 (wrong initial weight base for task 4: %0d)", weight_rd_addr);
            $finish;
        end

        wait(done);
        @(posedge clk);

        // ---------------------------------------------------------------------
        // Completion
        // ---------------------------------------------------------------------
        $display("TB_SCHEDULER: PASS");
        $finish;
    end

endmodule