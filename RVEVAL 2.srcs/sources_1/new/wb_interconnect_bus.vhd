library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use WORK.CONFIG.ALL;

entity wb_interconnect_bus is
    generic(
        DECODER_ADDR_WIDTH : integer := 8;
        NUM_SLAVES : integer := 2;
        BASE_ADDRS : MEMMAP_type := (X"F0", X"F1");
        SEGMENT_SIZES : SEGSIZE_type := (16, 16)          -- In 16-byte multiples
    );
    port(
        wb_master_rdata : out std_logic_vector(31 downto 0);
        wb_master_wdata : in std_logic_vector(31 downto 0);
        wb_master_addr : in std_logic_vector(31 downto 0);
        wb_master_wstrb : in std_logic_vector(3 downto 0);
        wb_master_cyc : in std_logic;
        wb_master_ack : out std_logic;
        
        wb_slave_rdata : in std_logic_vector(NUM_SLAVES * 32 - 1 downto 0);
        wb_slave_wdata : out std_logic_vector(31 downto 0);
        wb_slave_addr : out std_logic_vector(31 downto 0);
        wb_slave_wstrb : out std_logic_vector(3 downto 0);
        wb_slave_cyc : out std_logic_vector(NUM_SLAVES - 1 downto 0);
        wb_slave_ack : in std_logic_vector(NUM_SLAVES - 1 downto 0)
    );
end wb_interconnect_bus;

architecture rtl of wb_interconnect_bus is

begin
    process(all)
    begin
        wb_master_rdata <= (others => '0');
        wb_slave_cyc <= (others => '0');
        wb_master_ack <= '0';
        
        for i in 0 to NUM_SLAVES - 1 loop
            if (std_match(wb_master_addr(31 downto 32 - DECODER_ADDR_WIDTH), BASE_ADDRS(i))) then
                wb_master_rdata <= wb_slave_rdata(32 * (i + 1) - 1 downto 32 * i);
                wb_slave_cyc(i) <= wb_master_cyc;
                wb_master_ack <= wb_slave_ack(i);
            end if;
        end loop;
    end process;
    
    wb_slave_wdata <= wb_master_wdata;
    wb_slave_addr <= wb_master_addr;
    wb_slave_wstrb <= wb_master_wstrb;

end rtl;
