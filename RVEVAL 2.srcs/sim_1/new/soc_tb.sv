`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/02/2023 04:06:54 PM
// Design Name: 
// Module Name: soc_tb
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


module soc_tb(

    );
    logic uart_rx, uart_tx, clk, reset;
    logic [31:0] gpio_i, gpio_o;    
    
    soc soc_inst(
        .gpio_i(gpio_i),
        .gpio_o(gpio_o),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .clk(clk),
        .reset(reset)
    );
    
    always #5 clk = ~clk;
    
    initial begin
        reset = 1;
        clk = 0;
        uart_rx = 1;
        gpio_i = 0;
        #10;
        reset = 0;
        
        #1ms;
        uart_rx = 0;
        #10us;
        uart_rx = 1;
        #1ms;
        uart_rx = 0;
        #10us;
        uart_rx = 1;
    end
    
endmodule
