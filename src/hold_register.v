`timescale 1ns / 1ps

module hold_register (
    input  wire        ASIC_clk,      // 50 MHz clk
    input  wire        rst_n,         // Asynchronous active-low reset
    
    // Inputs from the SPI SIPO block
    input  wire        WE,            // Write Enable
    input  wire [15:0] data_word_16b, // 16 bit data word containing:
                                      // [15] parity
                                      // [14:13] mode
                                      // [12:9] address
                                      // [8:0] payload (multiplexed)
    
    // Decoded Outputs to the FSM, Timers, and Datapath
    output reg         parity_bit,         // Extracted parity bit [15]
    output reg [1:0]   mode,               // Decoded mode [14:13]
    output reg [3:0]   address,            // Decoded Address [12:9]
    output reg [8:0]   pulse_width,        // Decoded Pulse Width [8:0] (Modes 00, 01)
    output reg [7:0]   dsm_target_voltage, // DSM Target Voltage (Mode 11)
    output reg [9:0]   dsp_batch_size,     // DSP Batch Size (Mode 10)
    output reg         parity_error        // High for 1 cycle if parity check fails
);

    // Internal calculation for Even Parity across bits [14:0]
    // ^ bitwise XOR reduction
    wire calculated_parity;
    assign calculated_parity = ^data_word_16b[14:0];

    always @(posedge ASIC_clk or negedge rst_n) begin
        if (!rst_n) begin
            parity_bit         <= 1'b0;
            mode               <= 2'b00;
            address            <= 4'b0000;
            pulse_width        <= 9'd0;
            dsm_target_voltage <= 8'd0;
            dsp_batch_size     <= 10'd0;
            parity_error       <= 1'b0;
        end else begin
            // Default condition: Error flag clear after 1 clock cycle
            parity_error <= 1'b0;

            // Only update latched values when  SIPO has a full data word (WE high)
            if (WE == 1'b1) begin
                
                // Check if incoming parity bit matches calculated parity
                if (data_word_16b[15] == calculated_parity) begin
                    // Valid packet
                    // data word slicing
                    parity_bit <= data_word_16b[15];
                    mode       <= data_word_16b[14:13];
                    address    <= data_word_16b[12:9];
                    
                    // Multiplex the 9-bit payload based on mode
                    // Modes 00 and 01 both require the pulse width payload for their tests
                    if (data_word_16b[14:13] == 2'b00 || data_word_16b[14:13] == 2'b01) begin
                        pulse_width <= data_word_16b[8:0];
                    end 
                    else if (data_word_16b[14:13] == 2'b10) begin
                        // Zero-pad to 10 bits for DSP batch size
                        dsp_batch_size <= {1'b0, data_word_16b[8:0]};
                    end
                    else if (data_word_16b[14:13] == 2'b11) begin
                        // Mode 11: Map lower 8 bits for DSM voltage
                        dsm_target_voltage <= data_word_16b[7:0];
                    end
                end 
                else begin
                    // Corrupted packet
                    // Reg hold previous valid state
                    parity_error <= 1'b1;
                end
            end
        end
    end

endmodule
