-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: tb_ce_pulse_gen
-- // Project Name: EKF with HLS
-- //
-- // Description:
-- // Self-checking testbench for ce_pulse_gen. Drives a 100 MHz clock, targets a
-- // 1125 Hz CE pulse rate, measures the actual cycle count between two
-- // consecutive CE pulses, and reports target vs. achieved frequency.
-- //
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use std.env.all;
use std.textio.all;
use ieee.std_logic_textio.all;

entity tb_ce_pulse_gen is
end entity;

architecture sim of tb_ce_pulse_gen is

    constant CLK_FREQ_HZ : natural := 100_000_000;
    constant TARGET_HZ   : natural := 1125;
    constant CLK_PERIOD  : time    := 1 sec / CLK_FREQ_HZ;  -- 10 ns

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal ce    : std_logic;

    signal sim_done : boolean := false;

begin

    ----------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------
    dut : entity work.ce_pulse_gen
        generic map (
            CLK_FREQ_HZ => CLK_FREQ_HZ,
            TARGET_HZ   => TARGET_HZ
        )
        port map (
            clk   => clk,
            rst_n => rst_n,
            ce    => ce
        );

    ----------------------------------------------------------------
    -- Clock generation: free-running until sim_done
    ----------------------------------------------------------------
    clk_p : process
    begin
        while not sim_done loop
            clk <= '0';
            wait for CLK_PERIOD / 2;
            clk <= '1';
            wait for CLK_PERIOD / 2;
        end loop;
        wait;
    end process clk_p;

    ----------------------------------------------------------------
    -- Reset
    ----------------------------------------------------------------
    rst_p : process
    begin
        rst_n <= '0';
        wait for CLK_PERIOD * 5;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait;
    end process rst_p;

    ----------------------------------------------------------------
    -- Measurement: count cycles between two consecutive CE pulses
    ----------------------------------------------------------------
    meas_p : process
        variable cycle_count   : natural := 0;
        variable actual_hz     : real;
        variable target_hz_r   : real := real(TARGET_HZ);
        variable error_pct     : real;
        variable l             : line;
    begin
        -- Wait out reset
        wait until rst_n = '1';
        wait until rising_edge(clk);

        -- Sync to the first CE pulse (start of a period)
        wait until rising_edge(clk) and ce = '1';

        -- Count cycles until the next CE pulse
        loop
            wait until rising_edge(clk);
            cycle_count := cycle_count + 1;
            exit when ce = '1';
        end loop;

        actual_hz := real(CLK_FREQ_HZ) / real(cycle_count);
        error_pct := (actual_hz - target_hz_r) / target_hz_r * 100.0;

        write(l, string'("Measured period    : "));
        write(l, cycle_count);
        write(l, string'(" clock cycles"));
        writeline(output, l);

        write(l, string'("Target frequency    : "));
        write(l, target_hz_r, right, 0, 4);
        write(l, string'(" Hz"));
        writeline(output, l);

        write(l, string'("Actual frequency    : "));
        write(l, actual_hz, right, 0, 4);
        write(l, string'(" Hz"));
        writeline(output, l);

        write(l, string'("Error               : "));
        write(l, error_pct, right, 0, 6);
        write(l, string'(" %"));
        writeline(output, l);

        assert abs(error_pct) < 1.0
            report "FAIL: actual frequency deviates from target by more than 1%"
            severity error;

        report "PASS: ce_pulse_gen produces expected pulse rate within tolerance"
            severity note;

        sim_done <= true;
        wait for CLK_PERIOD;
        std.env.finish;
    end process meas_p;

    ----------------------------------------------------------------
    -- Watchdog: fail safely if CE never toggles
    ----------------------------------------------------------------
    watchdog_p : process
    begin
        wait for CLK_PERIOD * 200_000;  -- generous margin over one expected period (~88888 cycles)
        if not sim_done then
            report "FAIL: watchdog timeout, ce never observed as expected"
                severity failure;
        end if;
        wait;
    end process watchdog_p;

end architecture sim;