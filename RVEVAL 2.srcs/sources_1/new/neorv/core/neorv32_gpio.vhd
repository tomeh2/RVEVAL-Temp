-- #################################################################################################
-- # << NEORV32 - General Purpose Parallel Input/Output Port (GPIO) >>                             #
-- # ********************************************************************************************* #
-- # 64-bit general purpose parallel input & output port unit. Input/outputs are split into two    #
-- # 32-bit memory-mapped registers each.                                                          #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_gpio is
  port (
    -- host access --
    clk_i  : in  std_ulogic; -- global clock line
    rstn_i : in  std_ulogic; -- global reset line, low-active, async
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    rden_i : in  std_ulogic; -- read enable
    wren_i : in  std_ulogic; -- write enable
    data_i : in  std_ulogic_vector(31 downto 0); -- data in
    data_o : out std_ulogic_vector(31 downto 0); -- data out
    ack_o  : out std_ulogic; -- transfer acknowledge
    err_o  : out std_ulogic; -- transfer error
    -- parallel io --
    gpio_o : out std_ulogic_vector(63 downto 0);
    gpio_i : in  std_ulogic_vector(63 downto 0)
  );
end neorv32_gpio;

architecture neorv32_gpio_rtl of neorv32_gpio is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(gpio_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- accessible regs --
  signal din_hi,  din_lo  : std_ulogic_vector(31 downto 0); -- r/-: parallel input hi/lo
  signal dout_hi, dout_lo : std_ulogic_vector(31 downto 0); -- r/w: parallel output hi/lo

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = gpio_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= gpio_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dout_lo <= (others => '0');
      dout_hi <= (others => '0');
    elsif rising_edge(clk_i) then
      if (wren = '1') then
        if (addr = gpio_out_lo_addr_c) then
          dout_lo <= data_i;
        end if;
        if (addr = gpio_out_hi_addr_c) then
          dout_hi <= data_i;
        end if;
      end if;
    end if;
  end process write_access;

  -- GPIO output --
  gpio_o <= dout_hi & dout_lo;


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- bus handshake --
      ack_o <= (wren and addr(3)) or rden;
      err_o <= wren and (not addr(3)); -- INPUT registers are read only!
      -- read data --
      data_o <= (others => '0');
      if (rden = '1') then
        case addr(3 downto 2) is
          when "00"   => data_o <= din_lo;
          when "01"   => data_o <= din_hi;
          when "10"   => data_o <= dout_lo;
          when others => data_o <= dout_hi;
        end case;
      end if;
    end if;
  end process read_access;

  -- sample buffer to prevent metastability --
  input_buffer: process (clk_i)
  begin
    if rising_edge(clk_i) then
      din_lo <= gpio_i(31 downto 00);
      din_hi <= gpio_i(63 downto 32);
    end if;
  end process input_buffer;


end neorv32_gpio_rtl;
