-- Self-checking testbench for spi_master.
-- Loops spi_miso back to spi_mosi so every transferred byte should read back
-- exactly as sent. Run with GHDL:
--
--   ghdl -a --std=08 spi_master.vhd tb_spi_master.vhd
--   ghdl -e --std=08 tb_spi_master
--   ghdl -r --std=08 tb_spi_master --stop-time=10us --wave=tb_spi_master.ghw
--
-- (the --wave option is optional -- open the .ghw in gtkwave if you want to look
--  at the waveforms; the testbench itself will PASS/FAIL every check via report)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_spi_master is
end entity;

architecture sim of tb_spi_master is
    constant CLK_PERIOD       : time    := 10 ns;   -- 100 MHz
    constant CLK_DIV_HALF     : natural := 4;        -- fast divider, just to keep sim short
    constant SCLK_HIGH_H_TIME : natural := 6;
    constant CS_HOLD_TIME     : natural := 3;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal s_axis_tready : std_logic;
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_axis_tlast  : std_logic := '0';
    signal read_byte     : std_logic_vector(7 downto 0);

    signal spi_sclk : std_logic;
    signal spi_cs_n : std_logic;
    signal spi_mosi : std_logic;
    signal spi_miso : std_logic;

    signal sim_done : boolean := false;

    -- spi_sclk idle-hold monitor
    signal prev_sclk       : std_logic := '1';
    signal monitor_idle    : boolean   := false;
    signal idle_toggle_cnt : natural   := 0;

    -- CS-stays-low-mid-burst monitor
    signal prev_cs        : std_logic := '1';
    signal watch_cs_low   : boolean   := false;
    signal cs_glitch_cnt  : natural   := 0;

    procedure send_byte(
        signal   tvalid : out std_logic;
        signal   tdata  : out std_logic_vector(7 downto 0);
        signal   tlast  : out std_logic;
        signal   tready : in  std_logic;
        constant data   : std_logic_vector(7 downto 0);
        constant last   : std_logic;
        signal   sysclk : in  std_logic
    ) is
    begin
        wait until rising_edge(sysclk) and tready = '1';
        tdata  <= data;
        tlast  <= last;
        tvalid <= '1';
        wait until rising_edge(sysclk);
        tvalid <= '0';
        tlast  <= '0';
    end procedure;

begin

    dut : entity work.spi_master
        generic map (
            CLK_DIV_HALF     => CLK_DIV_HALF,
            SCLK_HIGH_H_TIME => SCLK_HIGH_H_TIME,
            CS_HOLD_TIME     => CS_HOLD_TIME
        )
        port map (
            clk => clk, rst_n => rst_n,
            s_axis_tready => s_axis_tready, s_axis_tvalid => s_axis_tvalid,
            s_axis_tdata  => s_axis_tdata,  s_axis_tlast  => s_axis_tlast,
            read_byte     => read_byte,
            spi_sclk => spi_sclk, spi_cs_n => spi_cs_n,
            spi_mosi => spi_mosi, spi_miso => spi_miso
        );

    -- direct loopback slave model: whatever the master sends, it reads back
    spi_miso <= spi_mosi;

    clk_gen : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PERIOD / 2;
            clk <= '1'; wait for CLK_PERIOD / 2;
        end loop;
        wait;
    end process;

    monitors : process(clk)
    begin
        if rising_edge(clk) then
            prev_sclk <= spi_sclk;
            prev_cs   <= spi_cs_n;

            if monitor_idle and spi_sclk /= prev_sclk then
                idle_toggle_cnt <= idle_toggle_cnt + 1;
                report "!! unexpected spi_sclk edge while idle, at " & time'image(now);
            end if;

            if watch_cs_low and spi_cs_n = '1' and prev_cs = '0' then
                cs_glitch_cnt <= cs_glitch_cnt + 1;
                report "!! spi_cs_n released unexpectedly mid-burst, at " & time'image(now);
            end if;
        end if;
    end process;

    stim : process
        -- variables, not signals: updates are immediate, with no delta-cycle
        -- overwrite issue when several checks run back-to-back with no wait between them
        variable checks_run  : natural := 0;
        variable checks_pass : natural := 0;

        procedure check(constant cond : boolean; constant msg : string) is
        begin
            checks_run := checks_run + 1;
            if cond then
                checks_pass := checks_pass + 1;
                report "PASS: " & msg;
            else
                report "FAIL: " & msg severity error;
            end if;
        end procedure;
    begin
        rst_n <= '0';
        wait for 5 * CLK_PERIOD;
        rst_n <= '1';
        wait for 3 * CLK_PERIOD;

        ---------------------------------------------------------------
        report "=== Test 1: single-byte transfer + loopback readback ===";
        ---------------------------------------------------------------
        send_byte(s_axis_tvalid, s_axis_tdata, s_axis_tlast, s_axis_tready, "10110101", '1', clk);
        wait until rising_edge(clk) and s_axis_tready = '1';
        check(read_byte = "10110101", "single-byte readback matches sent byte");
        check(spi_cs_n = '1', "CS released after single-byte transaction");
        check(spi_sclk = '1', "SCLK idle-high after transaction");

        ---------------------------------------------------------------
        report "=== Test 2: SCLK must stay still while genuinely idle ===";
        ---------------------------------------------------------------
        monitor_idle <= true;
        wait for 40 * CLK_PERIOD;
        monitor_idle <= false;
        check(idle_toggle_cnt = 0, "no spurious SCLK edges during idle window after transaction");

        ---------------------------------------------------------------
        report "=== Test 3: 3-byte burst, CS must stay low until tlast ===";
        ---------------------------------------------------------------
        watch_cs_low <= true;
        send_byte(s_axis_tvalid, s_axis_tdata, s_axis_tlast, s_axis_tready, "00000001", '0', clk);
        wait until rising_edge(clk) and s_axis_tready = '1';
        check(read_byte = "00000001", "burst byte 1 readback correct");
        check(spi_cs_n = '0', "CS still asserted after non-last burst byte 1");

        send_byte(s_axis_tvalid, s_axis_tdata, s_axis_tlast, s_axis_tready, "00000010", '0', clk);
        wait until rising_edge(clk) and s_axis_tready = '1';
        check(read_byte = "00000010", "burst byte 2 readback correct");
        check(spi_cs_n = '0', "CS still asserted after non-last burst byte 2");

        wait until rising_edge(clk) and s_axis_tready = '1';
        s_axis_tdata  <= "00000011";
        s_axis_tlast  <= '1';
        s_axis_tvalid <= '1';
        wait until rising_edge(clk);
        s_axis_tvalid <= '0';
        s_axis_tlast  <= '0';
        watch_cs_low  <= false;  -- CS is *expected* to drop finishing this (tlast) byte
        wait until rising_edge(clk) and s_axis_tready = '1';
        check(read_byte = "00000011", "burst byte 3 (tlast) readback correct");
        check(spi_cs_n = '1', "CS released after tlast burst byte");
        check(cs_glitch_cnt = 0, "CS never glitched high during bytes 1-2 of the burst");

        ---------------------------------------------------------------
        report "=== Test 4: a fresh transaction after the burst still works ===";
        ---------------------------------------------------------------
        wait for 20 * CLK_PERIOD;
        send_byte(s_axis_tvalid, s_axis_tdata, s_axis_tlast, s_axis_tready, "11100011", '1', clk);
        wait until rising_edge(clk) and s_axis_tready = '1';
        check(read_byte = "11100011", "post-burst transaction readback correct");

        ---------------------------------------------------------------
        report "=== SUMMARY: " & natural'image(checks_pass) & " / " & natural'image(checks_run) & " checks passed ===";
        if checks_pass = checks_run then
            report "ALL CHECKS PASSED";
        else
            report "SOME CHECKS FAILED -- see above" severity error;
        end if;

        sim_done <= true;
        wait;
    end process;

end architecture sim;