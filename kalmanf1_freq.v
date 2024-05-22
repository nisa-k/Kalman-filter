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


module kalmanf1ave
#(
//    parameter M_PI = 3.14159265358979323846,
//    parameter ADC_WIDTH = 14,
//    parameter AXIS_TDATA_WIDTH = 32,
//    parameter COUNT_WIDTH = 32,
//    parameter HIGH_THRESHOLD = -100,
//    parameter LOW_THRESHOLD = -150,
//    parameter INTEGER_BITS = 0,       // Number of integer bits for T_s
//    parameter FRACTIONAL_BITS = 24,   // Number of fractional bits for T_s
//    parameter x_init = 1e-6,  // initial guess for x_0|0 (units of power)
//    parameter x0 = 1.2e-6, // initial guess for the current state 0 (units of power)
//    parameter T_s = 1e-5,  // sampling time (s)=10 microsec
//    parameter tau = 1e-3, // lock-in rolloff time (1/f)
//    parameter omega = 6283.185307, // 2*M_PI/tau
//    parameter oT = 0.06283185307, //omega*T_s
//    parameter phi = 1.062831853, //1+omega*T_s
//    parameter phi_sq = 1.129611548, //phi^2
//    parameter d_var = 1.27407e-5 , //(V^2)
//    parameter o_prime = 628318.5307, // 2pi*f=2pi*100kHz
//    parameter g_0 = 1,
//    parameter k_omega = 4.347826087, //o_prime/(g_0*omega) control law
//    parameter s_var = 9e-4 // process and measurement noise (units of power (V^2))
    
    parameter M_PI = 34'b11_00100011110101110000101000111101,
    parameter ADC_WIDTH = 14,
    parameter AXIS_TDATA_WIDTH = 32,
    parameter COUNT_WIDTH = 32,
    parameter HIGH_THRESHOLD = -100,
    parameter LOW_THRESHOLD = -150,
    parameter INTEGER_BITS = 0,       // Number of integer bits for T_s
    parameter FRACTIONAL_BITS = 24,   // Number of fractional bits for T_s
    parameter x_init = 16'b0_00000000000000000001000011000110,  // initial guess for x_0|0 (units of power)
    parameter x0 = 16'b0_0000000000000000000101000010000111, // initial guess for the current state 0 (units of power)
    parameter e_pre_var0 = 32'b0_0000000000000000001000011000110111101111010000010110101111011011, //x0-x_init, 2*1e-6
    parameter T_s = 16'b0_00000000000000001010011111000101,  // sampling time (s)=10 microsec
    parameter tau = 16'b0_00000000010000011000100100110111, // lock-in rolloff time (1/f)
    parameter omega = 16'b1100010001011_001011110111000001000111100100001011100001001001100, // 2*M_PI/tau
    parameter oT = 16'b0_0001000000010101101111111001001000010101001011011001110000010100, //omega*T_s
    parameter phi = 16'b1_000100000001010110111111100100011100100000110110010111000111111, //1+omega*T_s
    parameter phi_sq = 16'b1_00100001001011100011100011101111110110000000110000010010011111, //phi^2
    parameter d_var = 16'b0_0010000010011101101111101100001001001000000011101000110010001010, //(V^2)
    parameter o_prime = 16'b10011001011001011110_10000111110110111111010010000111111111001011, // 2pi*f=2pi*100kHz
    parameter g_0 = 1'b1,
    parameter k_omega = 8'b1100100, //o_prime/(g_0*omega) control law
    parameter s_var = 16'b0_0000000000111010111110110111111010010000111111111001011100100100,
    parameter x_next1 = 16'b0_0000000000000000000100011101010011010011111110100011100111100001 // process and measurement noise (units of power (V^2))
)
(
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 125000000" *)
    input [AXIS_TDATA_WIDTH-1:0]   S_AXIS_IN_tdata,
    input                          S_AXIS_IN_tvalid,
    input                          S_AXIS_IN_tvalid_kal,
    input                          S_AXIS_IN_tready_kal,
    input                          S_AXIS_IN_tdata_kal,
    input                          clk,
    input                          rst,
    input [COUNT_WIDTH-1:0]        Ncycles,
//    input                          K_next_in,
    output reg [AXIS_TDATA_WIDTH-1:0]  M_AXIS_OUT_tdata,
    output                         M_AXIS_OUT_tvalid,
//    output reg                     M_AXIS_OUT_tvalid_kal,
    output reg [AXIS_TDATA_WIDTH-1:0]  M_AXIS_OUT_DIVISOR_tdata,
    output reg                     M_AXIS_OUT_DIVISOR_tvalid=0,
    output reg [AXIS_TDATA_WIDTH-1:0]  M_AXIS_OUT_DIVIDEND_tdata,
    output reg                     M_AXIS_OUT_DIVIDEND_tvalid=0,
    reg                     K_next_num,
    reg                     K_next_denom,
    output reg                     measured,
//    output reg [COUNT_WIDTH-1:0]   x_n,  
    output reg [15:0]              x_data
);
    
    wire signed [ADC_WIDTH-1:0]    y_measured;
    reg                            state, state_next;
    reg [COUNT_WIDTH-1:0]          counter=0, counter_next=0, counter1024=0, counter1024_next=0;
    reg [COUNT_WIDTH-1:0]          counter_output=0, counter_output_next=0;
    reg [COUNT_WIDTH-1:0]          cycle=0, cycle_next=0;
    reg [1:0]                      counter_eq_1024;
    
    
    // Extract only the 14-bits of ADC data 
    assign  y_measured = S_AXIS_IN_tdata[ADC_WIDTH-1:0];
    assign  M_AXIS_OUT_tvalid = S_AXIS_IN_tvalid;
    
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
            counter1024 <= 0;
            cycle <= 0;
        end
        else if (counter1024 == 1024) begin
            counter_eq_1024 = 1'b1;
            counter1024 = 1'b0;
            counter1024_next = 1'b0;     
        end else
        begin
            counter <= counter_next;
            counter1024 <= counter1024_next;
            counter_output <= counter_output_next;
            cycle <= cycle_next;
            counter_eq_1024 <= 1'b0;
        end
        
    end


    always @* // logic for counter, counter_output, and cycle buffer
    begin
        counter_next = counter + 1; // increment on each clock cycle
//        if (counter > 1024) begin
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
    

    reg [15:0] y, y_init, u; 
    reg [15:0] x_curr, x_curr0, x_curr1, x_curr2, x_next, x_next2, x_next3;
    reg [15:0] e_pre_var, e_next_var, e_next_var1, e_next_var2;
    reg [31:0] K_pre, K_next, one_K_next_sq, K_next_sq; // kalman gain
    
  
    initial begin
        y_init = 32'b0;
        x_curr = 32'b0; 
        x_curr0 = 32'b0;
        x_curr1 = 32'b0; 
        x_curr2 = 32'b0; 
        x_next = 32'b0; 
        x_next2 = 32'b0; 
        x_next3 = 32'b0;
        e_pre_var = 32'b0; 
        e_next_var = 32'b0; 
        e_next_var1 = 32'b0; 
        e_next_var2 = 32'b0;
        K_pre = 32'b0; 
        K_next = 32'b0; 
        one_K_next_sq = 32'b0; 
        K_next_sq = 32'b0;
        x_data = 32'b0;
        measured = 16'b0;
    end
     
    always@(posedge clk) begin
    
        if (counter == 9) begin
                y_init = y_measured;
//                x_next1 = phi*x_init;
                $display("Time %0t: %d, %d, y_init=%d", $time, counter, counter1024, y_init);
        end
        else if (counter == 10) begin
                x_next2 = k_omega*y_init;
                x_next3 = oT*x_next2;
                measured = x_next3;
                $display("Time %0t: %d, %d, x_next2=%d, x_next3=%d", $time, counter, counter1024, x_next2, x_next3);
        end
        else if (counter == 11) begin
                x_next = x_next1 - x_next3; // predict the next state x_n
                e_pre_var = e_pre_var0; // make sure this is a positive number
                can_go = 1;
                $display("Time %0t: %d, %d, x_next=%d, e_pre_var=%d", $time, counter, counter1024, x_next, e_pre_var);   
        end
            
        else if (counter_eq_1024 && can_go == 1) begin             
             
            y = y_measured;
            u = -oT*k_omega*y;
            $display("Time %0t: %d, %d, y=%d, u=%d", $time, counter, counter1024, y, u);
        end
        else if (counter1024 == 1 && can_go == 1) begin
            
            K_next_num = phi_sq*e_pre_var + d_var;
            K_next_denom = phi_sq*e_pre_var + d_var + s_var;
//            K_next = K_next_num / K_next_denom;
//            $display("K_next_num=%d, K_next_denom=%d", K_next_num, K_next_denom);
            M_AXIS_OUT_DIVISOR_tdata = K_next_denom;
            M_AXIS_OUT_DIVIDEND_tdata = K_next_num;
            M_AXIS_OUT_DIVISOR_tvalid = 1'b1; // send the numerator and denominator values to the Divider Generator
            M_AXIS_OUT_DIVIDEND_tvalid = 1'b1;
            $display("Time %0t: %d, %d, e_pre_var=%d, K_next_num=%d S_AXIS_IN_tdata_kal=%d, M_AXIS_OUT_DIVISOR_tdata=%d, M_AXIS_OUT_DIVIDEND_tdata=%d", $time, counter, counter1024, e_pre_var, K_next_num, S_AXIS_IN_tdata_kal, M_AXIS_OUT_DIVISOR_tdata, M_AXIS_OUT_DIVIDEND_tdata);
            can_go = 0;
            
            if (S_AXIS_IN_tready_kal && S_AXIS_IN_tvalid_kal) begin
                K_next = S_AXIS_IN_tdata_kal;
                can_go = 1'b1;
            
//            K_next = 16'd80;
//                $display("Time %0t: %d, %d, K_next_num=%d, S_AXIS_IN_tdata_kal=%d, phi_sq=%d, e_pre_var=%d", $time, counter, counter1024, K_next_num, S_AXIS_IN_tdata_kal, phi_sq, e_pre_var);
            end
        end 
        else if (counter1024 == 500 && can_go == 1) begin
		
		    K_next = S_AXIS_IN_tdata_kal;
            x_curr0 = 1-K_next;
            x_curr1 = x_curr0*x_next;
            x_curr2 = K_next*y;
            
        end 
        else if (counter1024 == 501 && can_go == 1) begin
            x_curr = x_curr1 + x_curr2; // estimate the current state
            
            x_next = phi*x_curr + u; // predict the next state
            
        end 
        else if (counter1024 == 502 && can_go == 1) begin
            //e_next_var = (1-K_next)*(1-K_next)*(phi*phi*e_pre_var + d_var) + K_next*K_next*s_var;
            one_K_next_sq = (1-K_next)*(1-K_next);
            K_next_sq = K_next*K_next;
        end
        else if(counter1024 == 503 && can_go == 1) begin
            e_next_var1 = one_K_next_sq*K_next_num;
            e_next_var2 = K_next_sq*s_var;
            e_next_var = e_next_var1 + e_next_var2;
            
        end
        else if (counter1024 == 504 && can_go == 1) begin

            // make the next values the old values for next cycle
            K_pre = K_next;
            e_pre_var = e_next_var;
     
            
            // Assign the calculated value to the output signal
            M_AXIS_OUT_tdata = x_next;
            
            // Store sampled data in memory
            x_data = x_next;
            
            M_AXIS_OUT_DIVISOR_tvalid = 1'b0; // stop data flow to Divider Generator
            M_AXIS_OUT_DIVIDEND_tvalid = 1'b0;
//            M_AXIS_OUT_tvalid_kal = 1'b0; // stop data flow to Divider Generator
            can_go = 0;
            
        end
    end
    

endmodule

