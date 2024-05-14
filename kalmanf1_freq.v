`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/15/2024 01:09:58 PM
// Design Name: 
// Module Name: kalmanf1
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module kalmanf1
#(
    parameter M_PI = 34'b11_00100011110101110000101000111101,
    parameter ADC_WIDTH = 14,
    parameter AXIS_TDATA_WIDTH = 32,
    parameter COUNT_WIDTH = 32,
    parameter HIGH_THRESHOLD = -100,
    parameter LOW_THRESHOLD = -150,
    parameter INTEGER_BITS = 0,       // Number of integer bits for T_s
    parameter FRACTIONAL_BITS = 24,   // Number of fractional bits for T_s
    parameter x_init = 32'b0_00000000000000000001000011000110,  // initial guess for x_0|0 (units of power)
    parameter x0 = 34'b0_0000000000000000000101000010000111, // initial guess for the current state 0 (units of power)
    parameter e_pre_var0 = 64'b0_0000000000000000001000011000110111101111010000010110101111011011, //x0-x_init, 2*1e-6
    parameter T_s = 32'b0_00000000000000001010011111000101,  // sampling time (s)=10 microsec
    parameter tau = 32'b0_00000000010000011000100100110111, // lock-in rolloff time (1/f)
    parameter omega = 64'b1100010001011_001011110111000001000111100100001011100001001001100, // 2*M_PI/tau
    parameter oT = 64'b0_0001000000010101101111111001001000010101001011011001110000010100, //omega*T_s
    parameter phi = 64'b1_000100000001010110111111100100011100100000110110010111000111111, //1+omega*T_s
    parameter phi_sq = 64'b1_00100001001011100011100011101111110110000000110000010010011111, //phi^2
    parameter d_var = 64'b0_0000000000111010111110110111111010010000111111111001011100100100, //(V^2)
    parameter o_prime = 64'b10011001011001011110_10000111110110111111010010000111111111001011, // 2pi*f=2pi*100kHz
    parameter g_0 = 'd0023,
    parameter k_omega = 64'b100_0101100100001011001000010110010001011100010100111001101110010, //o_prime/(g_0*omega) control law
    parameter s_var = 64'b0_0010000010011101101111101100001001001000000011101000110010001010 // process and measurement noise (units of power (V^2))
)
(
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 125000000" *)
    input [AXIS_TDATA_WIDTH-1:0]   S_AXIS_IN_tdata,
    input                          S_AXIS_IN_tvalid,
    input                          clk,
    input                          rst,
    input [COUNT_WIDTH-1:0]        Ncycles,
    input                          K_next_in,
    output reg                     M_AXIS_OUT_tvalid,
    output reg                     K_next_num,
    output reg                     K_next_denom,
    output reg [COUNT_WIDTH-1:0]   x_n,  
    output reg [15:0]              x_data
);
    
    wire signed [ADC_WIDTH-1:0]    y_measured;
    reg                            state, state_next;
    reg [COUNT_WIDTH-1:0]          counter=0, counter_next=0, counter1024=0, counter1024_next=0;
    reg [COUNT_WIDTH-1:0]          counter_output=0, counter_output_next=0;
    reg [COUNT_WIDTH-1:0]          cycle=0, cycle_next=0;
    
    
    // Extract only the 14-bits of ADC data 
    assign  y_measured = S_AXIS_IN_tdata[ADC_WIDTH-1:0];
    
    // Handling of the state buffer for finding signal transition at the threshold
    always @(posedge clk) 
    begin
        if (~rst) 
            state <= 1'b0;
        else
            state <= state_next;
    end
    
    
    always @*            // logic for state buffer
    begin
        if (y_measured > HIGH_THRESHOLD)
            state_next = 1;
        else if (y_measured < LOW_THRESHOLD)
            state_next = 0;
        else
            state_next = state;
    end
    

    // Handling of counter, counter_output and cycle buffer
    always @(posedge clk) 
    begin
        if (~rst) 
        begin
            counter <= 0;
            counter_output <= 0;
            // counter1024 <= 0;
            cycle <= 0;
        end
        else
        begin
            counter <= counter_next;
            counter1024 <= counter1024_next;
            counter_output <= counter_output_next;
            cycle <= cycle_next;
        end
    end


    always @* // logic for counter, counter_output, and cycle buffer
    begin
        counter_next = counter + 1; // increment on each clock cycle
        counter1024_next = counter1024 + 1;
        counter_output_next = counter_output;
        cycle_next = cycle;
        
        if (state < state_next) // high to low signal transition
        begin
            cycle_next = cycle + 1; // increment on each signal transition
            if (cycle >= Ncycles-1) 
            begin
                counter_next = 0;
                counter1024_next = 0;
                counter_output_next = counter;
                cycle_next = 0;
            end
        end
   end
    

    reg [31:0] y, y_init, u; 
    reg [31:0] x_curr, x_curr1, x_curr2, x_next, x_next1, x_next2;
    reg [31:0] e_pre_var, e_next_var, e_next_var1, e_next_var2;
    reg [31:0] K_pre, one_K_next_sq, K_next_sq; // kalman gain
    
    reg [1:0] counter_eq_1024;

    always @ (posedge clk) begin
        if (counter1024 == 1024) begin
            counter_eq_1024 = 1'b1;
            counter1024 = 1'b0;
            counter1024_next = 1'b0;     
        end else begin
            counter_eq_1024 = 1'b0;
        end
    end
      
     
    always@(posedge clk) begin
    
        if (counter == 0) begin
                y_init = y_measured;
                x_next1 = phi*x_init;
                x_next2 = k_omega*y_init;
                x_next = x_next1 - oT*x_next2; // predict the next state x_n
                e_pre_var = e_pre_var0; // make sure this is a positive number   
        end
            
        if (counter_eq_1024) begin             
             
            y = y_measured;
            u = -oT*k_omega*y;
            
            K_next_num = (phi_sq*e_pre_var + d_var);
            K_next_denom = (phi_sq*e_pre_var + d_var + s_var);
//            K_next = K_next_num / K_next_denom;
	    M_AXIS_OUT_tvalid = 1'b1; // send the numerator and denominator values to the Divider Generator
	    K_next = K_next_in;
		
            x_curr1 = (1-K_next)*x_next;
            x_curr2 = K_next*y;
            x_curr = x_curr1 + x_curr2; // estimate the current state
            
            x_next = phi*x_curr + u; // predict the next state
            
            //e_next_var = (1-K_next)*(1-K_next)*(phi*phi*e_pre_var + d_var) + K_next*K_next*s_var;
            one_K_next_sq = (1-K_next)*(1-K_next);
            K_next_sq = K_next*K_next;
            e_next_var1 = one_K_next_sq*K_next_num;
            e_next_var2 = K_next_sq*s_var;
            e_next_var = e_next_var1 + e_next_var2;
            
            // make the next values the old values for next cycle
            K_pre = K_next;
            e_pre_var = e_next_var;
     
            
            // Assign the calculated value to the output signal
            x_n = x_next;
            
            // Store sampled data in memory
            x_data = x_next;
            
            M_AXIS_OUT_tvalid = 1'b0; // stop data flow to Divider Generator
        end
    end
    

endmodule

