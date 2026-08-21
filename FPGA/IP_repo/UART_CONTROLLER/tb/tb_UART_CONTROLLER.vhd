-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: tb_uart_controller
-- // Project Name: EKF with HLS
-- //
-- // Description:
-- //   Self-checking testbench for UART_CONTROLLER (MODE = 0). Drives two
-- //   back-to-back 15-word frames through the AXI-Stream fifo input and checks
-- //   every word of the emitted frame: SYNC, MODE/LEN, all 15 payload echoes,
-- //   and the CHK word. Running two distinct frames back-to-back specifically
-- //   regression-tests that chk_word is not carried over between frames.
-- //
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use std.env.all;

entity tb_uart_controller is
end entity;

architecture sim of tb_uart_controller is

    constant CLK_PERIOD : time := 10 ns;  -- 100 MHz, arbitrary for this logic-only test

    constant MODE_G : natural := 0;

    constant SYNC_WORD     : std_logic_vector(15 downto 0) := x"AA55";
    constant MODE_LEN_WORD : std_logic_vector(15 downto 0) := x"001E";  -- matches MODE_0 for MODE_G = 0

    type payload_array_t is array (0 to 14) of std_logic_vector(15 downto 0);

    -- Two distinct payloads so a stale chk_word carried over from frame 1 into
    -- frame 2 would produce a mismatching CHK and get caught.
    -- Layout: 4 time-stamp words, accx/y/z, gyrox/y/z, temp, magx/y/z, status (15 total).
    constant PAYLOAD1_FULL : payload_array_t := (
        x"0001", x"0002", x"0003", x"0004",
        x"0AA1", x"0AA2", x"0AA3",
        x"0BB1", x"0BB2", x"0BB3",
        x"00C1",
        x"0DD1", x"0DD2", x"0DD3",
        x"00E1"
    );

    constant PAYLOAD2_FULL : payload_array_t := (
        x"1001", x"1002", x"1003", x"1004",
        x"1AA1", x"1AA2", x"1AA3",
        x"1BB1", x"1BB2", x"1BB3",
        x"10C1",
        x"1DD1", x"1DD2", x"1DD3",
        x"10E1"
    );

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal s_axis_fifo_tdata  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_axis_fifo_tvalid : std_logic := '0';
    signal s_axis_fifo_tlast  : std_logic := '0';
    signal s_axis_fifo_tready : std_logic;

    signal m_axis_tdata  : std_logic_vector(15 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1';  -- always-ready sink

    signal sim_done : boolean := false;

    function xor_all(payload : payload_array_t) return std_logic_vector is
        variable result : std_logic_vector(15 downto 0) := (others => '0');
    begin
        for i in payload'range loop
            result := result xor payload(i);
        end loop;
        return result;
    end function;

begin

    ----------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------
    dut : entity work.UART_CONTROLLER
        generic map (
            MODE => MODE_G
        )
        port map (
            clk                => clk,
            rst_n              => rst_n,
            s_axis_fifo_tdata  => s_axis_fifo_tdata,
            s_axis_fifo_tvalid => s_axis_fifo_tvalid,
            s_axis_fifo_tlast  => s_axis_fifo_tlast,
            s_axis_fifo_tready => s_axis_fifo_tready,
            m_axis_tdata       => m_axis_tdata,
            m_axis_tvalid      => m_axis_tvalid,
            m_axis_tready      => m_axis_tready
        );

    ----------------------------------------------------------------
    -- Clock
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
    -- Driver + checker: send two frames back-to-back, verify every word
    ----------------------------------------------------------------
    test_p : process
        variable payload   : payload_array_t;
        variable exp_chk    : std_logic_vector(15 downto 0);
        variable fail_count : natural := 0;
    begin
        wait until rst_n = '1';
        wait until rising_edge(clk);

        for frame in 0 to 1 loop

            if frame = 0 then
                payload := PAYLOAD1_FULL;
            else
                payload := PAYLOAD2_FULL;
            end if;
            exp_chk := xor_all(payload);

            -- send payload word 0 (latched internally, forwarded after MODE/LEN)
            s_axis_fifo_tdata  <= payload(0);
            s_axis_fifo_tvalid <= '1';
            s_axis_fifo_tlast  <= '0';
            wait until rising_edge(clk) and s_axis_fifo_tready = '1';
            s_axis_fifo_tvalid <= '0';

            -- expect SYNC
            wait until rising_edge(clk) and m_axis_tvalid = '1';
            if m_axis_tdata /= SYNC_WORD then
                report "FAIL frame " & integer'image(frame) & ": SYNC mismatch, got 0x" &
                       to_hstring(m_axis_tdata) severity error;
                fail_count := fail_count + 1;
            end if;

            -- expect MODE/LEN
            wait until rising_edge(clk) and m_axis_tvalid = '1';
            if m_axis_tdata /= MODE_LEN_WORD then
                report "FAIL frame " & integer'image(frame) & ": MODE/LEN mismatch, got 0x" &
                       to_hstring(m_axis_tdata) severity error;
                fail_count := fail_count + 1;
            end if;

            -- expect echoed payload word 0
            wait until rising_edge(clk) and m_axis_tvalid = '1';
            if m_axis_tdata /= payload(0) then
                report "FAIL frame " & integer'image(frame) & ": payload[0] mismatch, got 0x" &
                       to_hstring(m_axis_tdata) severity error;
                fail_count := fail_count + 1;
            end if;

            -- send + check payload words 1..14
            for i in 1 to 14 loop
                s_axis_fifo_tdata  <= payload(i);
                s_axis_fifo_tvalid <= '1';
                if i = 14 then
                    s_axis_fifo_tlast <= '1';
                else
                    s_axis_fifo_tlast <= '0';
                end if;
                wait until rising_edge(clk) and s_axis_fifo_tready = '1';
                s_axis_fifo_tvalid <= '0';
                s_axis_fifo_tlast  <= '0';

                wait until rising_edge(clk) and m_axis_tvalid = '1';
                if m_axis_tdata /= payload(i) then
                    report "FAIL frame " & integer'image(frame) & ": payload[" &
                           integer'image(i) & "] mismatch, got 0x" & to_hstring(m_axis_tdata) &
                           " expected 0x" & to_hstring(payload(i)) severity error;
                    fail_count := fail_count + 1;
                end if;
            end loop;

            -- expect CHK
            wait until rising_edge(clk) and m_axis_tvalid = '1';
            if m_axis_tdata /= exp_chk then
                report "FAIL frame " & integer'image(frame) & ": CHK mismatch, got 0x" &
                       to_hstring(m_axis_tdata) & " expected 0x" & to_hstring(exp_chk) severity error;
                fail_count := fail_count + 1;
            else
                report "PASS frame " & integer'image(frame) & ": CHK correct (0x" &
                       to_hstring(exp_chk) & ")" severity note;
            end if;

        end loop;

        if fail_count = 0 then
            report "PASS: all frames received correctly, no mismatches" severity note;
        else
            report "FAIL: " & integer'image(fail_count) & " mismatch(es) detected" severity error;
        end if;

        sim_done <= true;
        wait for CLK_PERIOD * 2;
        std.env.finish;
    end process test_p;

    ----------------------------------------------------------------
    -- Watchdog
    ----------------------------------------------------------------
    watchdog_p : process
    begin
        wait for CLK_PERIOD * 2000;
        if not sim_done then
            report "FAIL: watchdog timeout, DUT stalled" severity failure;
        end if;
        wait;
    end process watchdog_p;

end architecture sim;