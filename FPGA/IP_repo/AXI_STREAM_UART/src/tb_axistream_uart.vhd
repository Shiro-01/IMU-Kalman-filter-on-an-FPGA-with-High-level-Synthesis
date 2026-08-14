-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: tb_axistream_uart
-- // Description:
-- // Self-checking loopback testbench. Ties uart_txd straight into uart_rxd
-- // (exactly what the physical FTDI link does on hardware), sends a small
-- // list of test bytes through din, and checks each one comes back out on
-- // dout unchanged, with no frame_error/parity_error. Includes 0x0A first,
-- // since that's the exact value used during hardware bring-up debugging.
-- //
-- // Uses the SAME generics as the real hardware config (100MHz, 115200,
-- // 8N1) so a pass/fail here says something directly about the real
-- // config, not just "the RTL works in some generic sense".
-- //
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_axistream_uart is
end entity;

architecture sim of tb_axistream_uart is

    constant CLK_PERIOD : time := 10 ns;   -- 100 MHz

    -- DUT ports
    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal uart_txd     : std_logic;
    signal uart_rxd     : std_logic;
    signal din          : std_logic_vector(7 downto 0) := (others => '0');
    signal din_valid    : std_logic := '0';
    signal din_ready    : std_logic;
    signal dout         : std_logic_vector(7 downto 0);
    signal dout_valid   : std_logic;
    signal dout_ready   : std_logic := '1';   -- always ready to accept in this tb
    signal frame_error  : std_logic;
    signal parity_error : std_logic;

    -- test bytes, 0x0A first to match the exact hardware debug scenario
    type byte_array_t is array (natural range <>) of std_logic_vector(7 downto 0);
    constant TEST_BYTES : byte_array_t := (x"0A", x"55", x"FF", x"00", x"A5");

    signal sim_done : boolean := false;

begin

    -- Loopback: exactly what the physical UART wire does on hardware
    uart_rxd <= uart_txd;

    dut : entity work.axistream_uart
        generic map (
            BAUDRATE      => 1152000,
            PARITY_MODE   => 0,
            EIGHT_BIT     => true,
            TWO_STOP_BITS => false,
            OVERSAMPLE    => 16,
            CLK_FREQ_HZ   => 100_000_000
        )
        port map (
            clk          => clk,
            rst_n        => rst_n,
            uart_txd     => uart_txd,
            uart_rxd     => uart_rxd,
            din          => din,
            din_valid    => din_valid,
            din_ready    => din_ready,
            dout         => dout,
            dout_valid   => dout_valid,
            dout_ready   => dout_ready,
            frame_error  => frame_error,
            parity_error => parity_error
        );

    clk_gen_p : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PERIOD / 2;
            clk <= '1'; wait for CLK_PERIOD / 2;
        end loop;
        wait;
    end process clk_gen_p;

    stim_p : process
        variable pass_count : natural := 0;
        variable fail_count : natural := 0;
    begin
        rst_n <= '0';
        wait for CLK_PERIOD * 10;
        rst_n <= '1';
        wait for CLK_PERIOD * 5;

        for i in TEST_BYTES'range loop
            -- send
            wait until rising_edge(clk) and din_ready = '1';
            din       <= TEST_BYTES(i);
            din_valid <= '1';
            wait until rising_edge(clk);
            din_valid <= '0';

            -- receive, with a watchdog so a stuck design doesn't hang forever
            wait until rising_edge(clk) and dout_valid = '1' for 5 ms;

            if dout_valid /= '1' then
                report "FAIL: byte " & integer'image(i) &
                       " (expected 0x" & to_hstring(TEST_BYTES(i)) &
                       ") -- TIMEOUT, dout_valid never asserted"
                       severity error;
                fail_count := fail_count + 1;
            elsif dout = TEST_BYTES(i) and frame_error = '0' and parity_error = '0' then
                report "PASS: byte " & integer'image(i) &
                       " = 0x" & to_hstring(TEST_BYTES(i));
                pass_count := pass_count + 1;
            else
                report "FAIL: byte " & integer'image(i) &
                       " -- expected 0x" & to_hstring(TEST_BYTES(i)) &
                       " got 0x" & to_hstring(dout) &
                       " frame_error=" & std_logic'image(frame_error) &
                       " parity_error=" & std_logic'image(parity_error)
                       severity error;
                fail_count := fail_count + 1;
            end if;

            wait for CLK_PERIOD * 20;   -- idle gap between bytes
        end loop;

        report "=================================";
        report "RESULT: " & integer'image(pass_count) & " passed, " &
               integer'image(fail_count) & " failed";
        report "=================================";

        sim_done <= true;
        wait;
    end process stim_p;

end architecture sim;