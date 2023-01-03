`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/01/2023 03:11:35 PM
// Design Name: 
// Module Name: wb_interconnect_tb
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


module wb_interconnect_tb(
        
    );
    logic [31:0] wb_master_rdata, wb_master_wdata, wb_master_addr;
    logic [3:0] wb_master_wstrb;
    logic wb_master_cyc, wb_master_ack;
    
    logic [1:0] wb_slave_cyc_1, wb_slave_ack_1;
    logic [3:0] wb_slave_wstrb_1;
    logic wb_slave_wdata_1, wb_slave_rdata_1, wb_slave_addr_1;
    
    logic [63:0] wb_slave_rdata;
    
    logic [1:0] cyc;
    logic [1:0] ack;
    
    logic clk, reset;
    
    wb_interconnect_bus bus(
            .wb_master_rdata(wb_master_rdata),
            .wb_master_wdata(wb_master_wdata),
            .wb_master_addr(wb_master_addr),
            .wb_master_wstrb(wb_master_wstrb),
            .wb_master_cyc(wb_master_cyc),
            .wb_master_ack(wb_master_ack),
        
            .wb_slave_rdata(wb_slave_rdata),
            .wb_slave_wdata(wb_slave_wdata_1),
            .wb_slave_addr(wb_slave_addr_1),
            .wb_slave_wstrb(wb_slave_wstrb_1),
            .wb_slave_cyc(cyc),
            .wb_slave_ack(ack));
            
    initial
    begin
        wb_master_addr = 32'hA000_0000;
        wb_master_wdata = 32'hF0F0_F0F0;
        wb_master_cyc = 1;
        wb_master_wstrb = 4'b1111;
        
        ack = 2'b11;
        wb_slave_rdata = 64'hBABA_BABA_FCFC_FCFC;
        
        #100;
        
        wb_master_addr = 32'hF000_0000;
        wb_master_wdata = 32'hF0F0_F0F0;
        wb_master_cyc = 1;
        wb_master_wstrb = 4'b1111;
        
        #100;
        
        wb_master_addr = 32'hF100_0000;
        
        #100;
        
        wb_master_addr = 32'hF200_0000;
    end 
    
endmodule
