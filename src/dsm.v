`timescale 1ns / 1ps

module delta_sigma_modulator (
    input  wire       ASIC_clk,       // 50 MHz clk
    input  wire       rst_n,          // Asynchronous active-low reset
    
    // Inputs from the Datapath / FSM
    input  wire       dsm_enable,     // Controlled by FSM
    input  wire [7:0] target_voltage, // 8-bit target from multiplexed hold_register
    
    // Outputs to the Output Router
    output reg        pdm_out         // 1-bit Pulse Density Modulation stream
);

    // Internal 8-bit accumulator register
    reg [7:0] accumulator;

    // Internal calculation for sigma-delta arithmetic
    // 9-bit for addition and carry-out
    // padded with 1'b0
    wire [8:0] sigma_adder;
    assign sigma_adder = {1'b0, accumulator} + {1'b0, target_voltage};

    always @(posedge ASIC_clk or negedge rst_n) begin
        if (!rst_n) begin
            accumulator <= 8'd0;
            pdm_out     <= 1'b0;
        end else begin
            if (dsm_enable == 1'b1) begin
                // remainder stays in the accumulator
                accumulator <= sigma_adder[7:0];
                
                // carry-out becomes the analog bitstream
                pdm_out     <= sigma_adder[8];
            end else begin
                // discharge NEOS gate voltage when disabled
                accumulator <= 8'd0;
                pdm_out     <= 1'b0;
            end
        end
    end

endmodule
