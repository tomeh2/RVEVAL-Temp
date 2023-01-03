----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/30/2022 03:08:05 PM
-- Design Name: 
-- Module Name: soc - rtl
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use WORK.CONFIG.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity soc is
    port(
        gpio_o : out std_logic_vector(31 downto 0);
        gpio_i : in std_logic_vector(31 downto 0);
    
        uart_tx : out std_logic;
        uart_rx : in std_logic;
    
        clk : in std_logic;
        reset : in std_logic
    );
end soc;

architecture rtl of soc is
    component picorv32
		generic(
			STACKADDR : std_logic_vector(31 downto 0) := STACK_ADDR;
			PROGADDR_RESET : std_logic_vector(31 downto 0) := RESET_PC;
			ENABLE_COUNTERS : boolean := true;
			ENABLE_COUNTERS64 : boolean := true;
			ENABLE_REGS_16_31 : boolean := true;
			ENABLE_REGS_DUALPORT : boolean := true;
			LATCHED_MEM_RDATA : boolean := false;
			TWO_STAGE_SHIFT : boolean := true;
			BARREL_SHIFTER : boolean := true;
			TWO_CYCLE_COMPARE : boolean := false;
			TWO_CYCLE_ALU : boolean := false;
			COMPRESSED_ISA : boolean := false;
			CATCH_ILLINSN : boolean := true;
			ENABLE_PCPI : boolean := false;
			ENABLE_MUL : boolean := false;
			ENABLE_DIV : boolean := false;
			ENABLE_FAST_MUL : boolean := false;
			ENABLE_IRQ : boolean := false;
			ENABLE_IRQ_QREGS : boolean := true;
			ENABLE_IRQ_TIMER : boolean := true;
			ENABLE_TRACE : boolean := false;
			REGS_INIT_ZERO : boolean := true;
			MASKED_IRQ : std_logic_vector(31 downto 0) := (others => '0');
			LATCHED_IRQ : std_logic_vector(31 downto 0) := (others => '0');
			PROGADDR_IRQ : std_logic_vector(31 downto 0) := (others => '0')
		);
		port(
			mem_valid : out std_logic;
			mem_instr : out std_logic;
			mem_ready : in std_logic;
			mem_addr : out std_logic_vector(31 downto 0);
			mem_wdata : out std_logic_vector(31 downto 0);
			mem_wstrb : out std_logic_vector(3 downto 0);
			mem_rdata : in std_logic_vector(31 downto 0);
			
			pcpi_wr : in std_logic;
			pcpi_rd : in std_logic_vector(31 downto 0);
			pcpi_ready : in std_logic;
			pcpi_wait : in std_logic;
			
			mem_la_read : out std_logic;
			mem_la_write : out std_logic;
			mem_la_addr : out std_logic_vector(31 downto 0);
			mem_la_wdata : out std_logic_vector(31 downto 0);
			mem_la_wstrb : out std_logic_vector(3 downto 0);
			
			pcpi_valid : out std_logic;
			pcpi_insn : out std_logic_vector(31 downto 0);
			pcpi_rs1 : out std_logic_vector(31 downto 0);
			pcpi_rs2 : out std_logic_vector(31 downto 0);
			
			irq : in std_logic_vector(31 downto 0);
			eoi : out std_logic_vector(31 downto 0);
			
			trace_valid : out std_logic;
			trace_data : out std_logic_vector(35 downto 0);
			
			trap : out std_logic;
			clk : in std_logic;
			resetn : in std_logic
		);
	end component;
    
    component simpleuart
		port(
			clk : in std_logic;
			resetn : in std_logic;
			
			ser_tx : out std_logic;
			ser_rx : in std_logic;
			
			reg_div_we : in std_logic_vector(3 downto 0);
			reg_div_di : in std_logic_vector(31 downto 0);
			reg_div_do : out std_logic_vector(31 downto 0);
			
			reg_dat_we : in std_logic;
			reg_dat_re : in std_logic;
			
			reg_dat_di : in std_logic_vector(31 downto 0);
			reg_dat_do : out std_logic_vector(31 downto 0);
			reg_dat_wait : out std_logic
		);
	end component;

    signal wb_cpu_rdata : std_logic_vector(31 downto 0);
    signal wb_cpu_wdata : std_logic_vector(31 downto 0);
    signal wb_cpu_addr : std_logic_vector(31 downto 0);
    signal wb_cpu_wstrb : std_logic_vector(3 downto 0);
    signal wb_cpu_cyc : std_logic;
    signal wb_cpu_ack : std_logic;
    
    signal wb_wdata : std_logic_vector(31 downto 0);
    signal wb_addr : std_logic_vector(31 downto 0);
    signal wb_wstrb : std_logic_vector(3 downto 0);
    
    signal wb_uart_rdata : std_logic_vector(31 downto 0);
    signal wb_uart_cyc : std_logic;
    signal wb_uart_ack : std_logic;
    signal uart_bus_write : std_logic;
    signal uart_byte_sel : std_logic_vector(3 downto 0);
    
    signal wb_ram_rdata : std_logic_vector(31 downto 0);
    signal wb_ram_cyc : std_logic;
    signal wb_ram_ack : std_logic;
    
    signal wb_rom_rdata : std_logic_vector(31 downto 0);
    signal wb_rom_cyc : std_logic;
    signal wb_rom_ack : std_logic;
    
    signal wb_gpio_rdata : std_logic_vector(31 downto 0);
    signal wb_gpio_cyc : std_logic;
    signal wb_gpio_ack : std_logic;
begin
    picorv32_inst: picorv32
        port map (
            clk => clk,
            resetn => not reset,
            mem_valid => wb_cpu_cyc,
            mem_instr => open,
            mem_ready => wb_cpu_ack,
            mem_addr => wb_cpu_addr,
            mem_wdata => wb_cpu_wdata,
            mem_wstrb => wb_cpu_wstrb,
            mem_rdata => wb_cpu_rdata,
        
            irq => (others => '0'),
        
            pcpi_wr => '0',
            pcpi_rd => X"0000_0000",
            pcpi_ready => '0',
            pcpi_wait => '0'
        );

    interconnect_inst : entity work.wb_interconnect_bus(rtl)
                        generic map(DECODER_ADDR_WIDTH => 24,
                                    NUM_SLAVES => 4,
                                    BASE_ADDRS => (X"FFFFFB", X"80----", X"00----", X"FFFFFF"))
                        port map(wb_master_rdata => wb_cpu_rdata,
                                 wb_master_wdata => wb_cpu_wdata,
                                 wb_master_addr => wb_cpu_addr,
                                 wb_master_wstrb => wb_cpu_wstrb,
                                 wb_master_cyc => wb_cpu_cyc,
                                 wb_master_ack => wb_cpu_ack,
                                 
                                 wb_slave_rdata(31 downto 0) => wb_uart_rdata,
                                 wb_slave_rdata(63 downto 32) => wb_ram_rdata,
                                 wb_slave_rdata(95 downto 64) => wb_rom_rdata,
                                 wb_slave_rdata(127 downto 96) => wb_gpio_rdata,
                                 wb_slave_wdata => wb_wdata,
                                 wb_slave_addr => wb_addr,
                                 wb_slave_wstrb => wb_wstrb,
                                 
                                 wb_slave_cyc(0) => wb_uart_cyc,
                                 wb_slave_cyc(1) => wb_ram_cyc,
                                 wb_slave_cyc(2) => wb_rom_cyc,
                                 wb_slave_cyc(3) => wb_gpio_cyc,
                                 wb_slave_ack(0) => wb_uart_ack,
                                 wb_slave_ack(1) => wb_ram_ack,
                                 wb_slave_ack(2) => wb_rom_ack,
                                 wb_slave_ack(3) => wb_gpio_ack);
                                 
    -- 64KB RAM INSTANCE
    ram_memory_inst : entity work.ram_memory(rtl)
                      generic map(SIZE_BYTES => 65536)
                      port map(bus_addr => wb_addr(15 downto 0),
                               bus_wdata => wb_wdata,
                               bus_rdata => wb_ram_rdata,
                               bus_wstrb => wb_wstrb,
                               bus_ready => wb_ram_ack,
                               
                               en => wb_ram_cyc,
                               clk => clk,
                               resetn => not reset);
                               
    rom_memory_inst : entity work.rom_memory(rtl)
                      generic map(SIZE_BYTES => 1024)
                      port map(bus_addr => wb_addr(9 downto 0),
                               bus_rdata => wb_rom_rdata,
                               bus_ready => wb_rom_ack,
                               
                               en => wb_rom_cyc,
                               clk => clk,
                               resetn => not reset);
                                 
    sio_instance: entity work.sio
        generic map (
            C_clk_freq => 100,
            C_break_detect => false)
        port map (
            clk => clk,
            ce => wb_uart_cyc,
            txd => uart_tx,
            rxd => uart_rx,
            bus_write => uart_bus_write,
            byte_sel => uart_byte_sel,
            bus_in => wb_wdata,
            bus_out => wb_uart_rdata,
            bus_ack => wb_uart_ack,
            break => open
        );
    --wb_uart_ack <= wb_uart_cyc and not wb_uart_ack when rising_edge(clk);
    uart_bus_write <= '1' when wb_wstrb /= "0000" else '0';
    uart_byte_sel <= wb_wstrb when uart_bus_write = '1' else "0001";
    
    gpio_instance : entity work.gpio
                    port map(gpio_i => gpio_i,
                             gpio_o => gpio_o,
                             
                             bus_wdata => wb_wdata,
                             bus_rdata => wb_gpio_rdata,
                             bus_stbw => wb_wstrb,
                             bus_cyc => wb_gpio_cyc,
                             bus_ack => wb_gpio_ack,
                             
                             clk => clk,
                             reset => reset);

end rtl;






