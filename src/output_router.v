`timescale 1ns / 1ps

module output_router (

    input  wire       ASIC_clk,         // 50 MHz clk
    input  wire       rst_n,            // Asynchronous active-low reset

    // Inputs from Hold Register
    input  wire [1:0] mode,             // 2-bit decoded opcode
    input  wire [3:0] address,          // 4-bit device target address

    // Inputs from FSM
    input  wire [3:0] current_state,    // 4-bit active FSM state
    input  wire       dac_trig,         // Internal DAC trigger
    input  wire       adc_trig,         // Internal ADC trigger

    // Inputs from DSM and DSP
    input  wire       pdm_out,          // Internal DSM PDM stream
    input  wire       uart_tx_out,      // Internal DSP UART serial out

    // Outputs to External Pins (Laptop, Microcontroller, DAC, ADC, Mux)
    output reg  [5:0] status_out,       //[5:4] masked mode | [3:0] active state
    output reg  [3:0] address_out,      // Masked target address to external Mux
    output reg        dac_trig_out,     // Masked & buffered trigger to external DAC
    output reg        adc_trig_out,     // Masked & buffered trigger to external ADC
    output reg        pdm_pin_out,      // Buffered PDM to external RC filter
    output reg        uart_pin_out      // Buffered UART to PC
);

    // =========================================================================
    // FSM state def (mirrored for gate masking)
    // =========================================================================
    localparam [3:0]
        S_IDLE          = 4'd0,
        S_SETUP         = 4'd1,
        S_LOADING       = 4'd2,
        S_IV_SWEEP      = 4'd3,
        S_IV_HOLD       = 4'd4,
        S_RET_START     = 4'd5,
        S_RET_HOLD      = 4'd6,
        S_RET_READ_ACT  = 4'd7,
        S_RET_READ_HOLD = 4'd8,
        S_UNLOADING     = 4'd9,
        S_DSP_BATCH     = 4'd10, // Added DSP Batch state
        S_DSP_UART      = 4'd11; // Added DSP UART state

    // =========================================================================
    // combinational logic
    // =========================================================================
    
    // Mask 1: parity check window
    // address and mode only allowed after parity check
    wire valid_operation_window;
    assign valid_operation_window = (current_state >= S_LOADING) && (current_state <= S_DSP_UART);

    // Mask 2: DAC trig window
    wire valid_dac_window;
    assign valid_dac_window = (current_state == S_IV_SWEEP)   || 
                              (current_state == S_RET_START)  || 
                              (current_state == S_RET_READ_ACT);

    // Mask 3: ADC trig window
    wire valid_adc_window;
    assign valid_adc_window = (current_state == S_IV_HOLD) || 
                              (current_state == S_RET_READ_HOLD) ||
                              (current_state == S_DSP_BATCH); // Allow ADC to fire in DSP batch loop

    // bitwise AND gate masking
    wire [3:0] masked_address;
    wire [1:0] masked_mode;
    wire       masked_dac_trig;
    wire       masked_adc_trig;

    assign masked_address  = address  & {4{valid_operation_window}}; 
    assign masked_mode     = mode     & {2{valid_operation_window}}; 
    assign masked_dac_trig = dac_trig & valid_dac_window;
    assign masked_adc_trig = adc_trig & valid_adc_window;

    // =========================================================================
    // output stage and buffering
    // =========================================================================
    always @(posedge ASIC_clk or negedge rst_n) begin
        if (!rst_n) begin
            status_out      <= 6'b00_0000;
            address_out     <= 4'b0000;
            dac_trig_out    <= 1'b0;
            adc_trig_out    <= 1'b0;
            pdm_pin_out     <= 1'b0;
            uart_pin_out    <= 1'b1; // UART idle state is high
        end else begin
            // status output
            status_out      <= {masked_mode, current_state};

            // control outputs
            address_out     <= masked_address;
            dac_trig_out    <= masked_dac_trig;
            adc_trig_out    <= masked_adc_trig;
            
            // Continuous stream routing (unmasked)
            pdm_pin_out     <= pdm_out;
            uart_pin_out    <= uart_tx_out;
        end
    end

endmodule
