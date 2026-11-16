`timescale 1ns / 1ps

module fsm (

    input  wire       ASIC_clk,      // 50 MHz clk
    input  wire       rst_n,         // Asynchronous active-low reset

    // Inputs from SIPO & Hold reg
    input  wire       WE,            
    input  wire       parity_error,  
    input  wire [1:0] mode,          

    // Inputs from timers & ext. ADC
    input  wire       fixed_trig,    
    input  wire       fast_trig,     
    input  wire       slow_trig,     
    input  wire       adc_done,      
    
    // Inputs from Retention DSP
    input  wire       batch_complete,
    input  wire       uart_busy,

    // Outputs to timers
    output reg        enable_fixed,  
    output reg        enable_fast,   
    output reg        enable_slow,   

    // Outputs to output router & external
    output reg  [3:0] current_state, 
    output reg        dac_trig,      
    output reg        adc_trig,      
    
    // Outputs to DSM & DSP
    output reg        dsm_enable,
    output reg        dsp_measuring,
    output reg        uart_start
);

    // 13 states encoding fsm
    localparam [3:0]
        S_IDLE          = 4'd0,     // 1)  Default idle state
        S_SETUP         = 4'd1,     // 2)  Parity verification state
        S_LOADING       = 4'd2,     // 3)  5-cycle settling before pulse
        S_IV_SWEEP      = 4'd3,     // 4a) Branch IV: Active IV Sweep
        S_IV_HOLD       = 4'd4,     // 5a) Branch IV: Hold IV & ADC conversion
        S_RET_START     = 4'd5,     // 4b) Branch Retention: Active initial pulse
        S_RET_HOLD      = 4'd6,     // 5b) Branch Retention: Long-term wait
        S_RET_READ_ACT  = 4'd7,     // 6b) Branch Retention: Active read pulse
        S_RET_READ_HOLD = 4'd8,     // 7b) Branch Retention: Hold read & ADC conversion
        S_UNLOADING     = 4'd9,     // 8)  5-cycle settling before returning to IDLE
        S_DSP_BATCH     = 4'd10,    // 4c) Branch DSP: Measure decay loop
        S_DSP_UART_START= 4'd11,    // 5c) Branch DSP: Pulse UART trigger
        S_DSP_UART_WAIT = 4'd12;    // 6c) Branch DSP: Wait for UART to finish

    reg [3:0] next_state;

    //sequential logic
    always @(posedge ASIC_clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= S_IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    //combinational logic
    always @(*) begin
        // Defaults to LOW after each state change (!)
        next_state    = current_state;
        enable_fixed  = 1'b0;
        enable_fast   = 1'b0;
        enable_slow   = 1'b0;
        dac_trig      = 1'b0;
        adc_trig      = 1'b0;
        dsm_enable    = 1'b0;
        dsp_measuring = 1'b0;
        uart_start    = 1'b0;

        case (current_state)
            // -----------------------------------------------------------------
            // 1) IDLE: Wait for SIPO complete 16-bit word
            // -----------------------------------------------------------------
            S_IDLE: begin
                if (WE == 1'b1) begin
                    next_state = S_SETUP;
                end
            end

            // -----------------------------------------------------------------
            // 2) SETUP: Check parity result from hold_register
            // -----------------------------------------------------------------
            S_SETUP: begin
                if (parity_error == 1'b0) begin
                    next_state = S_LOADING;
                end else begin
                    next_state = S_IDLE;
                end
            end

            // -----------------------------------------------------------------
            // 3) LOADING: 5-cycle analogue settling time before branching
            // -----------------------------------------------------------------
            S_LOADING: begin
                enable_fixed = 1'b1;
                
                // Assert DSM enable early so the analogue voltage can settle
                if (mode == 2'd2) begin
                    dsm_enable = 1'b1;
                end
                
                if (fixed_trig == 1'b1) begin
                    if (mode == 2'd0) begin
                        next_state = S_IV_SWEEP;
                    end else if (mode == 2'd1) begin
                        next_state = S_RET_START;
                    end else if (mode == 2'd2) begin
                        // Evaluate batch_complete after DSP drops dsp_measuring
                        if (batch_complete == 1'b1) begin
                            next_state = S_DSP_UART_START; 
                        end else begin
                            next_state = S_DSP_BATCH; 
                        end
                    end else if (mode == 2'd3) begin
                        // Mode 11 (3): Pure configuration packet. Skip execution completely.
                        next_state = S_UNLOADING; 
                    end else begin
                        next_state = S_UNLOADING;
                    end
                end
            end

            // =================================================================
            // BRANCH IV (mode == 0)
            // =================================================================
            
            // 4a) ACTIVE IV SWEEP
            S_IV_SWEEP: begin
                enable_fast = 1'b1;
                dac_trig    = 1'b1;
                if (fast_trig == 1'b1) begin
                    next_state = S_IV_HOLD;
                end
            end

            // 5a) HOLD IV
            S_IV_HOLD: begin
                adc_trig = 1'b1;
                if (adc_done == 1'b1) begin
                    next_state = S_UNLOADING;
                end
            end

            // =================================================================
            // BRANCH RETENTION (mode == 1)
            // =================================================================

            // 4b) ACTIVE START RETENTION
            S_RET_START: begin
                enable_fast = 1'b1;
                dac_trig    = 1'b1;
                if (fast_trig == 1'b1) begin
                    next_state = S_RET_HOLD;
                end
            end

            // 5b) HOLD RETENTION (Wait for external long-term timer)
            S_RET_HOLD: begin
                enable_slow = 1'b1;
                if (slow_trig == 1'b1) begin
                    next_state = S_RET_READ_ACT;
                end
            end

            // 6b) ACTIVE READ RETENTION
            S_RET_READ_ACT: begin
                enable_fast = 1'b1;
                dac_trig    = 1'b1;
                if (fast_trig == 1'b1) begin
                    next_state = S_RET_READ_HOLD;
                end
            end

            // 7b) HOLD READ RETENTION
            S_RET_READ_HOLD: begin
                adc_trig = 1'b1;
                if (adc_done == 1'b1) begin
                    next_state = S_UNLOADING;
                end
            end

            // =================================================================
            // BRANCH DSP (mode == 2)
            // =================================================================

            // 4c) DSP BATCH LOOP
            S_DSP_BATCH: begin
                // Keep DSM active to bias the NEOS test cell
                dsm_enable = 1'b1;
                
                // Assert dsp_measuring to increment cycle_counter
                // and wait for ADC comparator to fire
                dsp_measuring = 1'b1;
                adc_trig      = 1'b1;
                
                if (adc_done == 1'b1) begin
                    // Drop dsp_measuring to 0 to trigger statistical updates
                    next_state = S_LOADING;
                end
            end
            
            // 5c) DSP UART START STATE
            S_DSP_UART_START: begin
                dsm_enable = 1'b0; // Safely discharge test cell
                uart_start = 1'b1; // Trigger transmission pulse
                
                // Immediately step to wait state on the next cycle
                next_state = S_DSP_UART_WAIT;
            end
            
            // 6c) DSP UART WAIT STATE
            S_DSP_UART_WAIT: begin
                dsm_enable = 1'b0;
                
                // Await completion edge from DSP transmitter
                if (uart_busy == 1'b0) begin
                    next_state = S_UNLOADING;
                end
            end

            // =================================================================
            // END OF BRANCHING
            // =================================================================

            // 8) UNLOADING: 5-cycle analogue settling before returning to IDLE, restart loop
            S_UNLOADING: begin
                enable_fixed = 1'b1;
                if (fixed_trig == 1'b1) begin
                    next_state = S_IDLE;
                end
            end

            default: begin
                next_state = S_IDLE;
            end
        endcase
    end

endmodule
