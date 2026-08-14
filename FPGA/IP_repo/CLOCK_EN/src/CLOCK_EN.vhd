-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: ce_pulse_gen
-- // Project Name: EKF with HLS
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description:
-- // Simple free-running clock-enable pulse generator. this pulse rate in our project ispecifically 
-- // is the actual sampling rate driver. 
-- //
-- // subject to whatever rounding CLK_FREQ_HZ/TARGET_HZ leaves behind.
-- // 
-- //
-- // Revision 0.01 - File Created
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;

entity ce_pulse_gen is
    generic (
        CLK_FREQ_HZ : natural := 100_000_000;  -- system clock frequency
        TARGET_HZ   : natural := 48_000        -- desired ce pulse rate
    );
    port (
        clk   : in  std_logic;
        rst_n : in  std_logic;
        ce    : out std_logic                  -- 1-cycle pulse at ~TARGET_HZ
    );
end entity;

architecture rtl of ce_pulse_gen is

    constant DIV_MAX : natural := (CLK_FREQ_HZ / TARGET_HZ) - 1;

    signal count : natural range 0 to DIV_MAX := 0;
    signal ce_r  : std_logic := '0';

begin

    ce <= ce_r;

    div_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                count <= 0;
                ce_r  <= '0';
            else
                ce_r <= '0';   -- default: 1-cycle pulse, cleared unless set below

                if count = DIV_MAX then
                    count <= 0;
                    ce_r  <= '1';
                else
                    count <= count + 1;
                end if;
            end if;
        end if;
    end process div_p;

end architecture rtl;