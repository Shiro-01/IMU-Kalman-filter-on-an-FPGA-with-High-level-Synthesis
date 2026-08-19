-- //////////////////////////////////////////////////////////////////////////////////
-- // tb_spi_controller.vhd
-- //
-- // Self-checking testbench for spi_controller. Models a realistic SPI slave stub
-- // that takes a full byte period (8 SCLK cycles @ 5MHz) to complete each transfer,
-- // rather than handshaking instantly -- this matters specifically for verifying
-- // the tail of the running-FSM's burst read (the ST2 fetch), where handshake
-- // timing determines correctness.
-- //
-- // During setup phase, the stub echoes back the golden expected value for the
-- // current SETUP_SEQUENCE row (read via setup_idx_dbg), so every SETUP_READ_VERIFY
-- // passes and the DUT reaches SETUP_DONE without retries.
-- //
-- // During running phase, the stub returns the payload byte's own index (0,1,2...22)
-- // as its value, resetting to 0 each time it sees the ACCEL_XOUT_H address byte
-- // (start of a new burst), per the requested "payload is just its own index" model.
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_spi_controller is
end entity;

architecture sim of tb_spi_controller is

    constant CLK_PERIOD        : time    := 10 ns;      -- 100 MHz system clock
    constant SPI_SCLK_FREQ     : natural := 5_000_000;   -- assumed SPI SCLK
    constant CYCLES_PER_BIT    : natural := 100_000_000 / SPI_SCLK_FREQ;  -- 20
    constant BYTE_PERIOD_CYCLES: natural := CYCLES_PER_BIT * 8;           -- 160

    constant TIMESTAMP_WORDS_G : natural := 4;
    constant BYTES_TO_READ_G   : natural := 23;

    constant READ_COM  : std_logic_vector(7 downto 0) := x"80";
    constant ICM20948_ACCEL_XOUT_H : std_logic_vector(7 downto 0) := x"2D";
    constant ADDR_BYTE : std_logic_vector(7 downto 0) := READ_COM or ICM20948_ACCEL_XOUT_H;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal sim_done : boolean := false;

    signal IMU_INT   : std_logic := '0';
    signal timestamp : std_logic_vector((TIMESTAMP_WORDS_G * 16) - 1 downto 0) := x"0123456789ABCDEF";

    signal m_axis_fifo_tdata  : std_logic_vector(15 downto 0);
    signal m_axis_fifo_tvalid : std_logic;
    signal m_axis_fifo_tready : std_logic := '1';  -- always-ready sink
    signal m_axis_fifo_tlast  : std_logic;

    signal m_axis_spi_tdata  : std_logic_vector(7 downto 0);
    signal m_axis_spi_tvalid : std_logic;
    signal m_axis_spi_tready : std_logic := '0';
    signal m_axis_spi_tlast  : std_logic;
    signal read_byte         : std_logic_vector(7 downto 0) := (others => '0');

    signal spi_failure    : std_logic;
    signal setup_idx_dbg  : std_logic_vector(7 downto 0);
    signal setup_done_dbg : std_logic;

    -- Golden expected "data" field for each SETUP_SEQUENCE row (copied from the
    -- DUT's own table), used by the stub to answer SETUP_READ_VERIFY reads.
    type byte_array_t is array (natural range <>) of std_logic_vector(7 downto 0);
    constant SETUP_EXPECTED_DATA : byte_array_t(0 to 50) := (
        x"00", x"EA", x"01", x"00", x"20", x"00", x"3F", x"00", x"00", x"00",
        x"3F", x"00", x"01", x"01", x"00", x"22", x"20", x"20", x"10", x"30",
        x"07", x"0C", x"01", x"32", x"32", x"80", x"00", x"8C", x"00", x"80",
        x"00", x"8C", x"01", x"80", x"00", x"0C", x"08", x"31", x"80", x"00",
        x"8C", x"31", x"80", x"00", x"08", x"8C", x"10", x"89", x"89", x"10",
        x"00"
    );

    -- Expected FIFO burst words, precomputed for a payload where byte[i] = i.
    -- word4..10: big-endian pairing of consecutive index bytes (H=even,L=odd).
    -- word11..13: mag registers, little-endian on the wire, normalized to {H,L}.
    -- word14: {ST2=index22, ST1=index14}.
    type word_array_t is array (natural range <>) of std_logic_vector(15 downto 0);
    constant EXPECTED_BURST : word_array_t(0 to 14) := (
        x"0123", x"4567", x"89AB", x"CDEF",  -- timestamp words 0-3
        x"0001", x"0203", x"0405",           -- accel x,y,z
        x"0607", x"0809", x"0A0B",           -- gyro x,y,z
        x"0C0D",                             -- temp
        x"100F", x"1211", x"1413",           -- mag x,y,z (normalized H,L)
        x"160E"                              -- {ST2, ST1}
    );

begin

    ----------------------------------------------------------------
    -- DUT
    ----------------------------------------------------------------
    dut : entity work.spi_controller
        generic map (
            BYTES_TO_READ       => BYTES_TO_READ_G,
            TIMESTAMP_WORDS      => TIMESTAMP_WORDS_G,
            RESET_WAIT_CYCLES   => 20,
            GENERAL_WAIT_CYCLES => 20
        )
        port map (
            clk   => clk,
            rst_n => rst_n,
            IMU_INT => IMU_INT,
            timestamp => timestamp,
            m_axis_fifo_tdata  => m_axis_fifo_tdata,
            m_axis_fifo_tvalid => m_axis_fifo_tvalid,
            m_axis_fifo_tready => m_axis_fifo_tready,
            m_axis_fifo_tlast  => m_axis_fifo_tlast,
            m_axis_spi_tdata  => m_axis_spi_tdata,
            m_axis_spi_tvalid => m_axis_spi_tvalid,
            m_axis_spi_tready => m_axis_spi_tready,
            m_axis_spi_tlast  => m_axis_spi_tlast,
            read_byte         => read_byte,
            spi_failure       => spi_failure,
            setup_idx_dbg     => setup_idx_dbg,
            setup_done_dbg    => setup_done_dbg
        );

    ----------------------------------------------------------------
    -- Clock
    ----------------------------------------------------------------
    clk_p : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PERIOD / 2;
            clk <= '1'; wait for CLK_PERIOD / 2;
        end loop;
        wait;
    end process clk_p;

    ----------------------------------------------------------------
    -- Reset
    ----------------------------------------------------------------
    rst_p : process
    begin
        rst_n <= '0';
        wait for CLK_PERIOD * 10;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait;
    end process rst_p;

    ----------------------------------------------------------------
    -- SPI slave stub: takes a full BYTE_PERIOD_CYCLES to complete each
    -- transfer, matching a real 5MHz-SCLK SPI shift, rather than an
    -- instant handshake.
    ----------------------------------------------------------------
    spi_stub_p : process(clk)
        variable busy          : boolean := false;
        variable busy_counter  : natural range 0 to BYTE_PERIOD_CYCLES := 0;
        variable pending_setup_idx : natural range 0 to 51 := 0;
        variable pending_is_address : boolean := false;
        variable running_xfer_idx  : natural range 0 to 255 := 0;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                busy := false;
                busy_counter := 0;
                running_xfer_idx := 0;
                m_axis_spi_tready <= '0';
                read_byte <= (others => '0');
            else
                if not busy then
                    m_axis_spi_tready <= '1';
                    if m_axis_spi_tvalid = '1' then
                        -- handshake this cycle -- byte accepted, go busy
                        busy := true;
                        busy_counter := 0;
                        m_axis_spi_tready <= '0';
                        if setup_done_dbg = '0' then
                            pending_setup_idx := to_integer(unsigned(setup_idx_dbg));
                        else
                            if m_axis_spi_tdata = ADDR_BYTE then
                                running_xfer_idx := 0;
                                pending_is_address := true;
                            else
                                pending_is_address := false;
                            end if;
                        end if;
                    end if;
                else
                    if busy_counter < BYTE_PERIOD_CYCLES - 1 then
                        busy_counter := busy_counter + 1;
                    else
                        busy := false;
                        busy_counter := 0;
                        m_axis_spi_tready <= '1';
                        if setup_done_dbg = '0' then
                            if pending_setup_idx <= 50 then
                                read_byte <= SETUP_EXPECTED_DATA(pending_setup_idx);
                            else
                                read_byte <= x"00";
                            end if;
                        else
                            if pending_is_address then
                                read_byte <= x"FF";  -- don't-care response to address byte
                            else
                                read_byte <= std_logic_vector(to_unsigned(running_xfer_idx, 8));
                                running_xfer_idx := running_xfer_idx + 1;
                            end if;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process spi_stub_p;

    ----------------------------------------------------------------
    -- IMU_INT stimulus: pulse once setup completes, then again after
    -- each burst finishes, to exercise two full bursts.
    ----------------------------------------------------------------
    int_stim_p : process
    begin
        wait until setup_done_dbg = '1';
        wait until rising_edge(clk);
        report "PASS: setup completed, setup_done_dbg asserted" severity note;

        for burst in 0 to 1 loop
            IMU_INT <= '1';
            wait until rising_edge(clk);
            IMU_INT <= '0';
            -- wait for this burst's fifo tlast (end of burst) before triggering the next
            wait until rising_edge(clk) and m_axis_fifo_tvalid = '1' and m_axis_fifo_tready = '1' and m_axis_fifo_tlast = '1';
            wait for CLK_PERIOD * 10;  -- let running_state settle back to IDLE
        end loop;
        wait;
    end process int_stim_p;

    ----------------------------------------------------------------
    -- FIFO checker: captures each accepted word, compares the full
    -- 15-word burst against EXPECTED_BURST once tlast is seen.
    ----------------------------------------------------------------
    checker_p : process
        variable word_idx   : natural := 0;
        variable fail_count : natural := 0;
        variable burst_num  : natural := 0;
    begin
        wait until setup_done_dbg = '1';

        for burst in 0 to 1 loop
            word_idx := 0;
            loop
                wait until rising_edge(clk) and m_axis_fifo_tvalid = '1' and m_axis_fifo_tready = '1';
                if m_axis_fifo_tdata /= EXPECTED_BURST(word_idx) then
                    report "FAIL burst " & integer'image(burst) & " word " & integer'image(word_idx) &
                           ": got 0x" & to_hstring(m_axis_fifo_tdata) &
                           " expected 0x" & to_hstring(EXPECTED_BURST(word_idx))
                           severity error;
                    fail_count := fail_count + 1;
                end if;
                if word_idx = 14 then
                    if m_axis_fifo_tlast /= '1' then
                        report "FAIL burst " & integer'image(burst) & ": tlast not asserted on final word"
                               severity error;
                        fail_count := fail_count + 1;
                    else
                        report "PASS: burst " & integer'image(burst) & " completed, all 15 words correct, tlast asserted"
                               severity note;
                    end if;
                    exit;
                else
                    if m_axis_fifo_tlast = '1' then
                        report "FAIL burst " & integer'image(burst) & ": tlast asserted early at word " & integer'image(word_idx)
                               severity error;
                        fail_count := fail_count + 1;
                    end if;
                end if;
                word_idx := word_idx + 1;
            end loop;
        end loop;

        if fail_count = 0 then
            report "PASS: all bursts received correctly, no mismatches" severity note;
        else
            report "FAIL: " & integer'image(fail_count) & " mismatch(es) detected" severity error;
        end if;

        sim_done <= true;
        wait for CLK_PERIOD * 5;
        std.env.finish;
    end process checker_p;

    ----------------------------------------------------------------
    -- Watchdog
    ----------------------------------------------------------------
    watchdog_p : process
    begin
        wait for 2 ms;
        if not sim_done then
            report "FAIL: watchdog timeout -- setup or running FSM stalled" severity failure;
        end if;
        wait;
    end process watchdog_p;

end architecture sim;