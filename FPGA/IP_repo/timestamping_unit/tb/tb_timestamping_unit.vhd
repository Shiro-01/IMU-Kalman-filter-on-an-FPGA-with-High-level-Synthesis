-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- // Testbench for: timestamping_unit
-- // Project Name: EKF by HLS
-- // Tool: GHDL --std=08
-- //
-- // Description:
-- //   Self-checking testbench for the free-running timestamp counter.
-- //   Uses an 8-bit instance of the DUT (TIMESTAMP_WIDTH is just the width of a plain
-- //   unsigned counter, so behaviour is generic-independent -- an 8-bit instance lets
-- //   us actually exercise the wrap-around path in a sane number of clock cycles,
-- //   something that's impossible to do directly on the real 64-bit instance).
-- //   A second, separate instance at the real design's default width (64) free-runs
-- //   alongside the 8-bit one the whole test, to confirm the default-generic
-- //   configuration used in the real design compiles and counts correctly too.
-- //
-- //   Both DUTs are advanced through one shared "tick" procedure that mirrors the
-- //   DUT's own reset/increment behaviour as a golden reference model -- this avoids
-- //   the classic VHDL gotcha of reading a signal in the same delta cycle as its own
-- //   scheduled assignment (which silently reads the pre-update value).
-- //
-- // Checks:
-- //   1) time_stamp = 0 while rst_n is held low
-- //   2) after release, counter increments by exactly 1 every clock, matching the
-- //      golden reference model
-- //   3) a synchronous reset asserted mid-count snaps the counter back to 0 on the
-- //      next rising edge, and counting resumes correctly from 0 afterwards
-- //   4) the counter wraps from all-ones back to 0 (8-bit instance: 0xFF -> 0x00)
-- //      and continues counting correctly past the wrap
-- //   5) default-width (64-bit) instance free-runs the whole test and is checked
-- //      against its own independent reference count, then reset independently of
-- //      the 8-bit instance to confirm the two are not accidentally coupled
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_timestamping_unit is
end entity tb_timestamping_unit;

architecture sim of tb_timestamping_unit is

    constant CLK_PERIOD : time := 10 ns;  -- 100 MHz

    -- DUT #1 -- narrow width, used for the bulk of the checks incl. wrap-around
    constant W8 : natural := 8;
    signal clk         : std_logic := '0';
    signal rst_n       : std_logic := '0';
    signal time_stamp8 : std_logic_vector(W8 - 1 downto 0);

    -- DUT #2 -- default width (64), just to confirm the real configuration works
    constant W64 : natural := 64;
    signal rst_n64      : std_logic := '0';
    signal time_stamp64 : std_logic_vector(W64 - 1 downto 0);

    -- Golden reference model, updated by the tick() procedure below
    signal expected8   : unsigned(W8 - 1 downto 0)  := (others => '0');
    signal expected64  : unsigned(W64 - 1 downto 0) := (others => '0');

    signal checks_run    : natural := 0;
    signal checks_passed : natural := 0;

begin

    -- DUT #1: 8-bit instance, drives most of the testing (incl. wrap-around)
    dut8 : entity work.timestamping_unit
        generic map (
            TIMESTAMP_WIDTH => W8
        )
        port map (
            clk        => clk,
            rst_n      => rst_n,
            time_stamp => time_stamp8
        );

    -- DUT #2: default-width instance (matches real design usage, 64 bits)
    dut64 : entity work.timestamping_unit
        generic map (
            TIMESTAMP_WIDTH => W64
        )
        port map (
            clk        => clk,
            rst_n      => rst_n64,
            time_stamp => time_stamp64
        );

    -- Clock generation (shared by both DUTs)
    clk_gen_p : process
    begin
        clk <= '0';
        wait for CLK_PERIOD / 2;
        clk <= '1';
        wait for CLK_PERIOD / 2;
    end process clk_gen_p;

    stim : process
        variable checks_run_v    : natural := 0;
        variable checks_passed_v : natural := 0;

        procedure check(constant cond : boolean; constant msg : string) is
        begin
            checks_run_v := checks_run_v + 1;
            if cond then
                checks_passed_v := checks_passed_v + 1;
                report "PASS: " & msg;
            else
                report "FAIL: " & msg severity error;
            end if;
            checks_run    <= checks_run_v;
            checks_passed <= checks_passed_v;
        end procedure;

        -- Advances one clock, updates the golden reference model to mirror
        -- exactly what the DUTs themselves should do (sync reset / increment),
        -- then settles 1ns past the edge so signal reads see the update.
        procedure tick is
        begin
            wait until rising_edge(clk);
            if rst_n = '0' then
                expected8 <= (others => '0');
            else
                expected8 <= expected8 + 1;
            end if;

            if rst_n64 = '0' then
                expected64 <= (others => '0');
            else
                expected64 <= expected64 + 1;
            end if;
            wait for 1 ns;
        end procedure;

    begin
        report "==== timestamping_unit testbench start ====";

        ------------------------------------------------------------------
        -- 1) Reset held: time_stamp must stay at 0
        ------------------------------------------------------------------
        rst_n   <= '0';
        rst_n64 <= '0';
        tick;
        check(unsigned(time_stamp8) = 0, "time_stamp8 = 0 while rst_n held low");

        for i in 0 to 4 loop
            tick;
            check(unsigned(time_stamp8) = 0,
                  "time_stamp8 stays 0 during extended reset, cycle " & integer'image(i));
        end loop;

        ------------------------------------------------------------------
        -- 2) Release reset, counter increments by 1 every clock
        ------------------------------------------------------------------
        rst_n   <= '1';
        rst_n64 <= '1';

        for i in 1 to 20 loop
            tick;
            check(unsigned(time_stamp8) = expected8,
                  "time_stamp8 increments correctly, tick " & integer'image(i) &
                  " (got " & integer'image(to_integer(unsigned(time_stamp8))) &
                  ", expected " & integer'image(to_integer(expected8)) & ")");
        end loop;

        ------------------------------------------------------------------
        -- 3) Mid-count synchronous reset: snaps back to 0, then resumes counting
        ------------------------------------------------------------------
        rst_n <= '0';
        tick;
        check(unsigned(time_stamp8) = 0, "time_stamp8 snaps to 0 on synchronous reset mid-count");

        rst_n <= '1';
        for i in 1 to 5 loop
            tick;
            check(unsigned(time_stamp8) = expected8,
                  "time_stamp8 resumes counting correctly after reset, tick " & integer'image(i));
        end loop;

        ------------------------------------------------------------------
        -- 4) Wrap-around: run the 8-bit counter all the way to 0xFF and past it
        --    (fixed iteration count computed from the known current value --
        --    avoids reading a signal in the same delta cycle as its own update)
        ------------------------------------------------------------------
        while expected8 /= to_unsigned(255, W8) loop
            tick;
        end loop;
        check(unsigned(time_stamp8) = 255, "time_stamp8 reaches 0xFF just before wrap");

        tick;  -- one more increment: 255 + 1 wraps to 0
        check(unsigned(time_stamp8) = 0, "time_stamp8 wraps from 0xFF back to 0x00");

        for i in 1 to 5 loop
            tick;
            check(unsigned(time_stamp8) = expected8,
                  "time_stamp8 continues counting correctly after wrap, tick " & integer'image(i));
        end loop;

        ------------------------------------------------------------------
        -- 5) Default-width (64-bit) instance: it's been free-running (in step
        --    with the golden model) the whole test -- confirm it matches
        ------------------------------------------------------------------
        check(unsigned(time_stamp64) = expected64,
              "time_stamp64 (default TIMESTAMP_WIDTH=64) matches expected count " &
              "(got " & integer'image(to_integer(unsigned(time_stamp64))) &
              ", expected " & integer'image(to_integer(expected64)) & ")");

        -- reset the 64-bit instance on its own and confirm it responds independently
        rst_n64 <= '0';
        tick;
        check(unsigned(time_stamp64) = 0, "time_stamp64 resets to 0 independently of the 8-bit instance");
        check(unsigned(time_stamp8) /= 0, "time_stamp8 unaffected by resetting the independent 64-bit instance");

        ------------------------------------------------------------------
        -- Summary
        ------------------------------------------------------------------
        report "==== timestamping_unit testbench done: " &
               integer'image(checks_passed_v) & " / " & integer'image(checks_run_v) &
               " checks passed ====";

        if checks_passed_v = checks_run_v then
            report "ALL CHECKS PASSED";
        else
            report "SOME CHECKS FAILED" severity error;
        end if;

        std.env.stop;
    end process stim;

end architecture sim;