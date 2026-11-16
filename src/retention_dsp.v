`timescale 1ns / 1ps

module retention_dsp (
    input  wire        ASIC_clk,       // 50 MHz clk
    input  wire        rst_n,          // Asynchronous active-low reset
    
    // Inputs from the Datapath / FSM
    input  wire        dsp_measuring,  // High during retention decay measuring
    input  wire [9:0]  batch_size,     // Number of tests 
    input  wire        uart_start,     // FSM trigger to send data to PC
    
    // Outputs to FSM and Output Router
    output reg         batch_complete, // High when all tests are completed
    output reg         uart_busy,      // High while UART is transmitting
    output reg         uart_tx_out     // 115200 baud serial output
);

    // =========================================================================
    // Internal Registers & Signals
    // =========================================================================
    
    // Edge detection for measurement window
    reg dsp_measuring_d;
    
    // 16-bit Statistical Registers (Average logic removed for area optimization)
    reg [15:0] cycle_counter;
    reg [15:0] total_acc;
    reg [15:0] max_reg;
    reg [15:0] min_reg;
    
    // test number tracking
    reg [9:0]  test_count; 
    
    // UART TX State Machine Registers (Scaled to 6 bytes / 48-bit payload)
    reg [8:0]  baud_counter;    // Counts to 434 for 115200 baud @ 50MHz
    reg [3:0]  bit_counter;     // Counts 10 bits per byte (start, 8 data, stop)
    reg [2:0]  byte_counter;    // Counts 6 bytes (48 bits total)
    reg [7:0]  current_tx_byte; // Staging byte for transmission

    // ===================================================================================================
    // DSP Logic (measurement and accumulation) max 1.31 milliseconds (65,535 cycles) before overflowing
    // ===================================================================================================
    always @(posedge ASIC_clk or negedge rst_n) begin
        if (!rst_n) begin
            dsp_measuring_d <= 1'b0;
            cycle_counter   <= 16'd0;
            total_acc       <= 16'd0;
            max_reg         <= 16'd0;
            min_reg         <= 16'hFFFF; // Max 16-bit value
            test_count      <= 10'd0;
            batch_complete  <= 1'b0;
        end else begin
            dsp_measuring_d <= dsp_measuring;

            // Merged reset on rising edge (+ve)
            if (dsp_measuring == 1'b1 && dsp_measuring_d == 1'b0) begin
                cycle_counter <= 16'd1;
                
                // Clears accumulators at the start of the first test or start of a new batch
                if (test_count == 10'd0 || batch_complete == 1'b1) begin
                    total_acc      <= 16'd0;
                    max_reg        <= 16'd0;
                    min_reg        <= 16'hFFFF;
                    test_count     <= 10'd0; 
                    batch_complete <= 1'b0;  
                end
            end 
            // cycle counting
            else if (dsp_measuring == 1'b1) begin
                cycle_counter <= cycle_counter + 1'b1;
            end
            
            // processing triggered on falling edge (-ve)
            if (dsp_measuring == 1'b0 && dsp_measuring_d == 1'b1) begin
                total_acc <= total_acc + cycle_counter;
                
                if (cycle_counter > max_reg) begin
                    max_reg <= cycle_counter;
                end
                
                if (cycle_counter < min_reg) begin
                    min_reg <= cycle_counter;
                end
                
                test_count <= test_count + 1'b1;
                
                // completion if test count target is reached
                if ((test_count + 1'b1) >= batch_size) begin
                    batch_complete <= 1'b1;
                end
            end
        end
    end

    // =======================================================================================
    // UART transmitter (115200 Baud, 8N1) - 6-Byte Multiplexer Serialization
    // =======================================================================================
    always @(posedge ASIC_clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_busy       <= 1'b0;
            uart_tx_out     <= 1'b1; // UART idle state high
            baud_counter    <= 9'd0;
            bit_counter     <= 4'd0;
            byte_counter    <= 3'd0;
            current_tx_byte <= 8'd0;
        end else begin
            if (uart_start == 1'b1 && uart_busy == 1'b0) begin
                uart_busy    <= 1'b1;
                baud_counter <= 9'd0;
                bit_counter  <= 4'd0;
                byte_counter <= 3'd0;
            end 
            else if (uart_busy == 1'b1) begin
                // Clock divider for 115200 baud (50MHz / 115200 = 434)
                if (baud_counter < 9'd433) begin
                    baud_counter <= baud_counter + 1'b1;
                end else begin
                    baud_counter <= 9'd0;
                    
                    // State machine for single UART byte
                    case (bit_counter)
                        4'd0: begin 
                            uart_tx_out <= 1'b0; // Start Bit
                            
                            // BYTE MULTIPLEXER: 6 bytes total (Min, Max, Total)
                            case (byte_counter)
                                3'd0: current_tx_byte <= min_reg[7:0];
                                3'd1: current_tx_byte <= min_reg[15:8];
                                3'd2: current_tx_byte <= max_reg[7:0];
                                3'd3: current_tx_byte <= max_reg[15:8];
                                3'd4: current_tx_byte <= total_acc[7:0];
                                3'd5: current_tx_byte <= total_acc[15:8];
                                default: current_tx_byte <= 8'd0;
                            endcase
                        end
                        4'd1, 4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8: begin
                            uart_tx_out <= current_tx_byte[0]; // Data Bits (LSB first)
                            current_tx_byte <= {1'b0, current_tx_byte[7:1]}; // Shift current byte
                        end
                        4'd9: begin
                            uart_tx_out <= 1'b1; // Stop Bit
                        end
                    endcase
                    
                    // Advance bit counter
                    if (bit_counter < 4'd9) begin
                        bit_counter <= bit_counter + 1'b1;
                    end else begin
                        bit_counter <= 4'd0;
                        
                        // Advance byte counter up to 6 bytes (0 to 5)
                        if (byte_counter < 3'd5) begin
                            byte_counter <= byte_counter + 1'b1;
                        end else begin
                            uart_busy <= 1'b0; 
                        end
                    end
                end
            end else begin
                uart_tx_out <= 1'b1; // TX high when idle
            end
        end
    end

endmodule
