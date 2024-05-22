`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/05/2024 12:23:42 PM
// Design Name: 
// Module Name: test_bench
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


module test_bench
   (DDR_addr,
    DDR_ba,
    DDR_cas_n,
    DDR_ck_n,
    DDR_ck_p,
    DDR_cke,
    DDR_cs_n,
    DDR_dm,
    DDR_dq,
    DDR_dqs_n,
    DDR_dqs_p,
    DDR_odt,
    DDR_ras_n,
    DDR_reset_n,
    DDR_we_n,
    FIXED_IO_ddr_vrn,
    FIXED_IO_ddr_vrp,
    FIXED_IO_mio,
    FIXED_IO_ps_clk,
    FIXED_IO_ps_porb,
    FIXED_IO_ps_srstb,
    adc_clk_n_i,
    adc_clk_p_i,
    adc_csn_o,
    adc_dat_a_i,
    adc_dat_b_i,
    adc_enc_n_o,
    adc_enc_p_o,
    dac_clk_o,
    dac_dat_o,
    dac_pwm_o,
    dac_rst_o,
    dac_sel_o,
    dac_wrt_o,
    daisy_n_i,
    daisy_n_o,
    daisy_p_i,
    daisy_p_o,
    exp_n_tri_io,
    exp_p_tri_io,
    x_dat);
  inout [14:0]DDR_addr;
  inout [2:0]DDR_ba;
  inout DDR_cas_n;
  inout DDR_ck_n;
  inout DDR_ck_p;
  inout DDR_cke;
  inout DDR_cs_n;
  inout [3:0]DDR_dm;
  inout [31:0]DDR_dq;
  inout [3:0]DDR_dqs_n;
  inout [3:0]DDR_dqs_p;
  inout DDR_odt;
  inout DDR_ras_n;
  inout DDR_reset_n;
  inout DDR_we_n;
  inout FIXED_IO_ddr_vrn;
  inout FIXED_IO_ddr_vrp;
  inout [53:0]FIXED_IO_mio;
  inout FIXED_IO_ps_clk;
  inout FIXED_IO_ps_porb;
  inout FIXED_IO_ps_srstb;
  input adc_clk_n_i;
  input adc_clk_p_i;
  output adc_csn_o;
  input [13:0]adc_dat_a_i;
  input [13:0]adc_dat_b_i;
  output adc_enc_n_o;
  output adc_enc_p_o;
  output dac_clk_o;
  output [13:0]dac_dat_o;
  output [3:0]dac_pwm_o;
  output dac_rst_o;
  output dac_sel_o;
  output dac_wrt_o;
  input [1:0]daisy_n_i;
  output [1:0]daisy_n_o;
  input [1:0]daisy_p_i;
  output [1:0]daisy_p_o;
  inout [7:0]exp_n_tri_io;
  inout [7:0]exp_p_tri_io;
  output [15:0]x_dat;

  wire [14:0]DDR_addr;
  wire [2:0]DDR_ba;
  wire DDR_cas_n;
  wire DDR_ck_n;
  wire DDR_ck_p;
  wire DDR_cke;
  wire DDR_cs_n;
  wire [3:0]DDR_dm;
  wire [31:0]DDR_dq;
  wire [3:0]DDR_dqs_n;
  wire [3:0]DDR_dqs_p;
  wire DDR_odt;
  wire DDR_ras_n;
  wire DDR_reset_n;
  wire DDR_we_n;
  wire FIXED_IO_ddr_vrn;
  wire FIXED_IO_ddr_vrp;
  wire [53:0]FIXED_IO_mio;
  wire FIXED_IO_ps_clk;
  wire FIXED_IO_ps_porb;
  wire FIXED_IO_ps_srstb;
  wire adc_clk_n_i;
  wire adc_clk_p_i;
  wire adc_csn_o;
  wire [13:0]adc_dat_a_i;
  wire [13:0]adc_dat_b_i;
  wire adc_enc_n_o;
  wire adc_enc_p_o;
  wire dac_clk_o;
  wire [13:0]dac_dat_o;
  wire [3:0]dac_pwm_o;
  wire dac_rst_o;
  wire dac_sel_o;
  wire dac_wrt_o;
  wire [1:0]daisy_n_i;
  wire [1:0]daisy_n_o;
  wire [1:0]daisy_p_i;
  wire [1:0]daisy_p_o;
  wire [7:0]exp_n_tri_io;
  wire [7:0]exp_p_tri_io;
  wire [15:0]x_dat;

  reg clk = 0;
  reg [13:0] adc_data; // Assuming 14-bit ADC data width
  reg adc_valid = 1'b0; // Start with invalid data
  always #100 clk = ~clk;
  initial begin
    // Reset
    #50;
    adc_valid = 1'b1; // Start providing valid data
    
    // Simulate ADC data
    adc_data = 14'h123; // Example ADC data
    $display("Time %0t: adc_data = %h", $time, adc_data); // Print initial data
    #1000000000; // Simulate for some time
    adc_valid = 1'b0;
    adc_data = 14'h423;
    #50000000;
    adc_valid = 1'b1;
    // Change adc_data to 14'h223
    adc_data = 14'h223; // New ADC data
    $display("Time %0t: adc_data = %h", $time, adc_data); // Print new data
    #100; // Simulate for some more time
  end
   
  system system_i
       (.DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
        .adc_clk_n_i(clk),
        .adc_clk_p_i(~clk),
        .adc_csn_o(adc_csn_o),
        .adc_dat_a_i(adc_data),
        .adc_dat_b_i(adc_valid),
        .adc_enc_n_o(adc_enc_n_o),
        .adc_enc_p_o(adc_enc_p_o),
        .dac_clk_o(dac_clk_o),
        .dac_dat_o(dac_dat_o),
        .dac_pwm_o(dac_pwm_o),
        .dac_rst_o(dac_rst_o),
        .dac_sel_o(dac_sel_o),
        .dac_wrt_o(dac_wrt_o),
        .daisy_n_i(daisy_n_i),
        .daisy_n_o(daisy_n_o),
        .daisy_p_i(daisy_p_i),
        .daisy_p_o(daisy_p_o),
        .exp_n_tri_io(exp_n_tri_io),
        .exp_p_tri_io(exp_p_tri_io),
        .x_dat(x_dat));

endmodule
