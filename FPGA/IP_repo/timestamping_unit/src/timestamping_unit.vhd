-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: spi_master_adcs7476
-- // Project Name: EKF by HLS
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description:
-- //   This ip is jsut a simple counter that increase by 1 with every
-- //   Tick of the input clk. its output is the counter current value.
-- //   This value is being used as a global time stamp for the whole design
-- //   and get latched for each new IMU reading. it is design to run on 100 MHz clk
-- //
-- // Revision 0.01 - File Created
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;

entity timestamping_unit is
    port(
        clk    : in  std_logic;
        rst_n  : in  std_logic;    -- active-low synchronous reset

        time_stamp : out std_logic_vector(63 downto 0)
    );
end entity timestamping_unit;

architecture rtl of timestamping_unit is

    signal counter : unsigned(63 downto 0) := (others => '0');

begin
    time_stamp <= counter;

    counter_p : process(clk) is 
    begin 
        if rising_edge(clk) then 
            if rst_n = '0' then
                counter <= (others => '0');
            else
                counter <= counter + 1;
            end if;
        end if;  
    end process counter_p;
    
end architecture rtl;