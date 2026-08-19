-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: spi_master
-- // Project Name: EKF with HLS
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description: 
-- //   spi_controller drives an ICM-20948 (6-axis IMU + AK09916 magnetometer) over SPI
-- //   via a downstream spi_master IP, and streams timestamped sensor samples out over
-- //   an AXI4-Stream interface to a FIFO.
-- //
-- //   Operation is split into two internally-multiplexed FSMs sharing the single
-- //   AXI-Stream SPI port:
-- //
-- //     1) Setup FSM (setup_state) — runs once after reset. Walks a table-driven
-- //        register-init sequence (SETUP_SEQUENCE) that resets the ICM-20948, configures
-- //        the accelerometer/gyroscope (ODR, full-scale range, DLPF), brings up the
-- //        ICM's internal I2C master, resets and configures the AK09916 magnetometer
-- //        via SLV4 one-shot transactions (with WHO_AM_I verification), and finally
-- //        arms SLV0 for continuous background magnetometer reads. Every write step can
-- //        optionally be read back and verified (SETUP_READ_VERIFY); a mismatch sets a
-- //        sticky spi_failure flag and halts the FSM in place for ILA-based debugging
-- //        rather than retrying or aborting silently.
-- //
-- //     2) Running FSM (running_state) — takes over once setup completes (setup_state =
-- //        SETUP_DONE) and IMU_INT is asserted (synchronized + rising-edge detected).
-- //        On each interrupt it latches the current timestamp, pushes it out as
-- //        TIMESTAMP_WORDS 16-bit AXI-Stream words, then issues a single burst SPI read
-- //        starting at ACCEL_XOUT_H, streaming BYTES_TO_READ bytes back from the ICM.
-- //        Bytes are paired into 16-bit words and pushed to the FIFO as they arrive —
-- //        big-endian pairing (H before L) for accel/gyro/temp, little-endian pairing
-- //        (L before H) for the AK09916 magnetometer registers — with ST1/ST2 packed
-- //        together into a final status word.
-- //
-- //   Interfaces:
-- //     - m_axis_spi_*  : AXI4-Stream master to spi_master (8-bit, address+data framing,
-- //                       tlast ends each SPI CS cycle). Internally muxed between the
-- //                       setup and running FSMs based on setup_state.
-- //     - m_axis_fifo_* : AXI4-Stream master to the downstream sample FIFO (16-bit words:
-- //                       timestamp words followed by the packed sensor sample words).
-- //     - read_byte     : byte received back from spi_master for the in-flight SPI byte.
-- //     - spi_failure   : sticky flag, asserted if any setup-time register verification
-- //                       fails; intended as an ILA trigger for bring-up debugging.
-- // 
-- //
-- // 
-- // IMU registers layout
-- // *   data[0]  = ACCEL_XOUT_H       data[1]  = ACCEL_XOUT_L
-- // *   data[2]  = ACCEL_YOUT_H       data[3]  = ACCEL_YOUT_L
-- // *   data[4]  = ACCEL_ZOUT_H       data[5]  = ACCEL_ZOUT_L
-- // *
-- // *   data[6]  = GYRO_XOUT_H        data[7]  = GYRO_XOUT_L
-- // *   data[8]  = GYRO_YOUT_H        data[9]  = GYRO_YOUT_L
-- // *   data[10] = GYRO_ZOUT_H        data[11] = GYRO_ZOUT_L
-- // *
-- // *   data[12] = TEMP_OUT_H         data[13] = TEMP_OUT_L
-- // *
-- // *   data[14] = EXT_SLV_SENS_DATA_00  (AK09916 ST1  — bit0 = DRDY, bit1 = DOR)
-- // *   data[15] = EXT_SLV_SENS_DATA_01  (AK09916 HXL)
-- // *   data[16] = EXT_SLV_SENS_DATA_02  (AK09916 HXH)
-- // *   data[17] = EXT_SLV_SENS_DATA_03  (AK09916 HYL)
-- // *   data[18] = EXT_SLV_SENS_DATA_04  (AK09916 HYH)
-- // *   data[19] = EXT_SLV_SENS_DATA_05  (AK09916 HZL)
-- // *   data[20] = EXT_SLV_SENS_DATA_06  (AK09916 HZH)
-- // *   data[21] = EXT_SLV_SENS_DATA_07  (reserved / TMPS, unused)
-- // *   data[22] = EXT_SLV_SENS_DATA_08  (AK09916 ST2  — bit3 = HOFL overflow)
-- //
-- // FIFO burst layout (one burst per IMU_INT event, 15 words, tlast on final word):
-- //   word 0-3  = timestamp[63:48], [47:32], [31:16], [15:0]
-- //   word 4    = {ACCEL_XOUT_H, ACCEL_XOUT_L}
-- //   word 5    = {ACCEL_YOUT_H, ACCEL_YOUT_L}
-- //   word 6    = {ACCEL_ZOUT_H, ACCEL_ZOUT_L}
-- //   word 7    = {GYRO_XOUT_H,  GYRO_XOUT_L}
-- //   word 8    = {GYRO_YOUT_H,  GYRO_YOUT_L}
-- //   word 9    = {GYRO_ZOUT_H,  GYRO_ZOUT_L}
-- //   word 10   = {TEMP_OUT_H,   TEMP_OUT_L}
-- //   word 11   = {HXH, HXL}    -- mag X (byte order normalized to MSB=H, LSB=L)
-- //   word 12   = {HYH, HYL}    -- mag Y
-- //   word 13   = {HZH, HZL}    -- mag Z
-- //   word 14   = {ST2, ST1}    -- status byte pair, tlast = '1', end of burst
-- //
-- // Known limitation:
-- //   - This IP assumes m_axis_fifo_tready backpressure never exceeds one SPI byte period;
-- //   - if violated, SPI byte tracking will desync.
-- //
-- // Revision 0.01 - File Created
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_controller is
    generic (
        BYTES_TO_READ   : natural := 23;                -- Number of bytes to read.
        TIMESTAMP_WORDS : natural := 4;                 -- How many of 16 bits words the timestamp holds. 
                                                        -- ie, if timestmap is 64 bits, the timestamp words will be 4
        RESET_WAIT_CYCLES   : natural := 10_000_000;    -- ~100 ms @ 100 MHz.  from data sheet
        GENERAL_WAIT_CYCLES : natural := 10_000_000     -- ~100 ms @ 100 MHz — every other wait_after row + mag retry
    
    );

    port (
        -- clk & rst
        clk   : in std_logic;
        rst_n : in std_logic;

        -- IMU Interupt
        IMU_INT : in std_logic;

        -- Global timestamp from time stamping unit
        timestamp : in std_logic_vector((TIMESTAMP_WORDS * 16) - 1 downto 0);

        -- AXI-Stream Master FIFO Interface
        m_axis_fifo_tdata  : out std_logic_vector(16 - 1 downto 0);
        m_axis_fifo_tvalid : out std_logic;
        m_axis_fifo_tready : in  std_logic;
        m_axis_fifo_tlast  : out  std_logic;

        -- AXI-Stream SPI interface
        m_axis_spi_tdata  : out std_logic_vector(7 downto 0);
        m_axis_spi_tvalid : out std_logic;
        m_axis_spi_tready : in  std_logic;
        m_axis_spi_tlast  : out std_logic;
        read_byte         : in  std_logic_vector(7 downto 0);

        -- Debuging interface 
        spi_failure       : out std_logic;
        setup_idx_dbg     : out std_logic_vector(7 downto 0);
        setup_done_dbg    : out std_logic
    );
end entity spi_controller;


architecture rtl of spi_controller is
-- Contants --------------------------------------------------------
    -- Word width
    constant WORD_WIDTH         : natural := 16;       -- Word width of each data point. IMU is 16 bits for each word. 
    constant RESET_ROW_IDX      : natural := 2;        -- Reset row index inside the sequance table
    constant MAF_CONFIG_ROW_IDX : natural := 15;       --  row index inside the sequance table of the start of the mag config
    -- Commands
    constant READ_COM  : std_logic_vector(7 downto 0) := x"80";                   -- read command
    constant WRITE_COM : std_logic_vector(7 downto 0) := x"00";                   -- write command

    -- Addresses
    constant ICM20948_ACCEL_XOUT_H : std_logic_vector(7 downto 0) := x"2D";       -- ACCEL_X_H Address
    constant REG_BANK_SEL   : std_logic_vector(7 downto 0) := x"7F";              -- Bank select address
    constant WHO_AM_I_ADDR  : std_logic_vector(7 downto 0) := x"00";              -- WHO_AM_I register Address
    
    -- Values 
    constant DUMMY_BYTE         : std_logic_vector(7 downto 0)  := x"00";         -- Dummy byte works as a place hodler while shifting the inout data
    constant WHO_AM_I_VAL       : std_logic_vector(7 downto 0)  := x"EA";
    constant ICM20948_BANK_0    : std_logic_vector(7 downto 0)  := x"00";
    constant ICM20948_BANK_1    : std_logic_vector(7 downto 0)  := x"10";
    constant ICM20948_BANK_2    : std_logic_vector(7 downto 0)  := x"20";
    constant ICM20948_BANK_3    : std_logic_vector(7 downto 0)  := x"30";
    constant WHO_AM_I_1         : std_logic_vector(7 downto 0)  := x"48";
    constant WHO_AM_I_2         : std_logic_vector(7 downto 0)  := x"09";

-- Setup sequence table
    type setup_op_t is (SETUP_WRITE, SETUP_READ_VERIFY);

    type setup_entry_t is record
        op         : setup_op_t;
        addr       : std_logic_vector(7 downto 0);
        data       : std_logic_vector(7 downto 0);  -- write value, or expected read value
        wait_after : std_logic;                     -- '1' -> insert RESET_WAIT_CYCLES delay 
    end record;

    type setup_table_t is array (natural range <>) of setup_entry_t;

    constant SETUP_SEQUENCE : setup_table_t := (
        (SETUP_WRITE,       REG_BANK_SEL, ICM20948_BANK_0, '0'),  -- 0) select Bank 0
        (SETUP_READ_VERIFY, WHO_AM_I_ADDR, WHO_AM_I_VAL, '0'),    -- 1) verify WHO_AM_I
        --(SETUP_WRITE,       x"06", x"80", '1'),                   -- 2) PWR_MGMT_1 = RESET, then wait
        (SETUP_WRITE,       x"06", x"01", '0'),                   -- 3) PWR_MGMT_1 = CLK_PLL -- Wake up + auto clock (PLL)
        (SETUP_WRITE,       x"07", x"00", '0'),                   -- 4) PWR_MGMT_2 = accel+gyro on - Both enabled
        (SETUP_WRITE,       REG_BANK_SEL, ICM20948_BANK_2, '0'),  -- 5) select Bank 2
        (SETUP_WRITE,       x"00", x"00", '0'),                   -- 6) Gyro ODR — 1.125 kHz, GYRO_SMPLRT_DIV
        (SETUP_WRITE,       x"01", x"3F", '0'),                   -- 7) GYRO_CONFIG_1 (running: ±2000dps, DLPF 361.4Hz)
        (SETUP_WRITE,       x"02", x"00", '0'),                   -- 8) GYRO_CONFIG_2 - No self-test, no extra averaging
        (SETUP_WRITE,       x"10", x"00", '0'),                   -- 9) ACCEL_SMPLRT_DIV_1 -- Accel ODR — 1.125 kHz
        (SETUP_WRITE,       x"11", x"00", '0'),                   -- 10) ACCEL_SMPLRT_DIV_2 - rest of the Accel ODR divide
        (SETUP_WRITE,       x"14", x"3F", '0'),                   -- 11) ACCEL_CONFIG (running: ±16g, DLPF 473Hz)
        (SETUP_WRITE,       x"15", x"00", '0'),                   -- 12) ACCEL_CONFIG_2 - No self test, no extra averaging
        (SETUP_WRITE,       x"09", x"01", '0'),                   -- 13) ODR_ALIGN_EN — sync accel & gyro start time
        (SETUP_READ_VERIFY, x"09", x"01", '0'),                   -- 17) confirm 
        (SETUP_WRITE,       REG_BANK_SEL, ICM20948_BANK_0, '0'),  -- 14) restore Bank 0 -- Starting Mag Config

        -- MAG Config
        (SETUP_WRITE,       x"03", x"22", '1'),                   -- 15) I2C_MST_EN=1 | I2C_MST_RST=1 -- reset the aux I2C state machine (self-clearing), then wait
        --(SETUP_READ_VERIFY, x"03", x"20", '0'),                   -- 17) confirm I2C_MST_RST self-cleared (0x20 == master enabled, reset done

        (SETUP_WRITE,       x"03", x"20", '0'),                   -- 15) Enable internal I2C Master of ICM20948
        (SETUP_READ_VERIFY, x"03", x"20", '0'),                   -- 17) confirm 

        (SETUP_WRITE,       x"0f", x"10", '0'),                   -- 18) INT pin: active high, 50us pulse, push-pull, cleared on any read, BYPASS disabled
        (SETUP_WRITE,       REG_BANK_SEL, ICM20948_BANK_3, '0'),  -- 19) select Bank 3
        (SETUP_WRITE,       x"01", x"07", '1'),                   -- 20) I2C_MST_CLK = 7 (345.6 kHz / 46.67% duty cycle)

        -- 9.2) One-time: write the mag's mode register to reset it, via SLV4
        (SETUP_WRITE,       x"13", x"0C", '0'),                   -- 21) SLV4_ADDR = AK09916(0x0C) | write bit
        (SETUP_WRITE,       x"16", x"01", '0'),                   -- 22) SLV4_DO = 0x01 (mag reset value)
        (SETUP_WRITE,       x"14", x"32", '0'),                   -- 23) SLV4_REG = 0x32 (CNTL3)
        (SETUP_READ_VERIFY, x"14", x"32", '0'),                   -- 25) confirm 
        (SETUP_WRITE,       x"15", x"80", '1'),                   -- 24) SLV4_CTRL = trigger(0x80)
        (SETUP_READ_VERIFY, x"15", x"00", '0'),                   -- 25) confirm EN cleared (transaction done)

        -- 9.3) check who am i _ 1
        (SETUP_WRITE,       x"13", x"8C", '0'),                   -- 26) SLV4_ADDR = AK09916(0x0C) | read bit
        (SETUP_WRITE,       x"14", x"00", '0'),                   -- 27) SLV4_REG = WIA_1(0x00)
        (SETUP_WRITE,       x"15", x"80", '1'),                   -- 28) SLV4_CTRL = trigger(0x80)
        (SETUP_READ_VERIFY, x"15", x"00", '0'),                   -- 29) confirm EN cleared
        -- (SETUP_READ_VERIFY, x"17", WHO_AM_I_1, '0'),               -- 30) check company ID 0x48

        -- 9.4) check who am i _ 2
        (SETUP_WRITE,       x"13", x"8C", '0'),                   -- 26) SLV4_ADDR = AK09916(0x0C) | read bit
        (SETUP_WRITE,       x"14", x"01", '0'),                   -- 31) SLV4_REG = WIA_2(0x01) — SLV4_ADDR unchanged, reused
        (SETUP_WRITE,       x"15", x"80", '1'),                   -- 32) SLV4_CTRL = trigger(0x80)
        (SETUP_READ_VERIFY, x"15", x"00", '0'),                   -- 33) confirm EN cleared
        -- (SETUP_READ_VERIFY, x"17", WHO_AM_I_2, '0'),               -- 34) check device ID 0x09

        -- 9.5) writing the MAG Mode 4 -> 100 Hz
        (SETUP_WRITE,       x"13", x"0C", '0'),                   -- 35) SLV4_ADDR = AK09916(0x0C) | write bit
        (SETUP_WRITE,       x"16", x"08", '0'),                   -- 36) SLV4_DO = 0x08 (100Hz, mode 4)
        (SETUP_WRITE,       x"14", x"31", '0'),                   -- 37) SLV4_REG = 0x31 (CNTL2)
        (SETUP_WRITE,       x"15", x"80", '1'),                   -- 38) SLV4_CTRL = trigger(0x80)
        (SETUP_READ_VERIFY, x"15", x"00", '0'),                   -- 39) confirm EN cleared

        -- 9.6) checking the MAG Mode 4 -> 100 Hz
        (SETUP_WRITE,       x"13", x"8C", '0'),                   -- 40) SLV4_ADDR = AK09916(0x0C) | read bit
        (SETUP_WRITE,       x"14", x"31", '0'),                   -- 41) SLV4_REG = 0x31 (CNTL2)
        (SETUP_WRITE,       x"15", x"80", '1'),                   -- 42) SLV4_CTRL = trigger(0x80)
        (SETUP_READ_VERIFY, x"15", x"00", '0'),                   -- 43) confirm EN cleared
        (SETUP_READ_VERIFY, x"17", x"08", '0'),                   -- 44) confirm mode == 0x08

        -- 9.7) enable continuous read
        (SETUP_WRITE,       x"03", x"8C", '0'),                   -- 45) SLV0_ADDR = AK09916(0x0C) | read bit
        (SETUP_WRITE,       x"04", x"10", '0'),                   -- 46) SLV0_REG = ST1(0x10) — start of read stream
        (SETUP_WRITE,       x"05", x"89", '1'),                   -- 47) SLV0_CTRL = enable(0x80) | 9 bytes
        (SETUP_READ_VERIFY, x"05", x"89", '0'),                   -- 48) verify enable read
        (SETUP_READ_VERIFY, x"04", x"10", '0'),                   -- 49) verify SLV0_REG

        (SETUP_WRITE,       REG_BANK_SEL, ICM20948_BANK_0, '0')   -- 50) restore Bank 0
    );


-- Signals ---------------------------------------------------
-- Ports Signals
    -- AXI-Stream Master FIFO Interface signals
    signal m_axis_fifo_tdata_r  : std_logic_vector(WORD_WIDTH - 1 downto 0) := (others => '0');
    signal m_axis_fifo_tvalid_r : std_logic := '0';
    signal m_axis_fifo_tlast_r  : std_logic := '0';


    -- AXI-Stream SPI interface signals
    signal m_axis_spi_tdata_r_setup  : std_logic_vector(7 downto 0) := (others => '0');
    signal m_axis_spi_tvalid_r_setup : std_logic := '0';
    signal m_axis_spi_tlast_r_setup  : std_logic := '0';

    signal m_axis_spi_tdata_r_running  : std_logic_vector(7 downto 0) := (others => '0');
    signal m_axis_spi_tvalid_r_running : std_logic := '0';
    signal m_axis_spi_tlast_r_running  : std_logic := '0';

-- Interupt Sync & edge detection process signals
    signal imu_int_ff1     : std_logic := '0';
    signal imu_int_ff2     : std_logic := '0';
    signal imu_int_ff2_dly : std_logic := '0';

-- Set-Up FSM Signals
    type setup_state_t is (SETUP_CMD, SETUP_DATA, SETUP_VERIFY, SETUP_WAIT_HANDSHAKE, SETUP_WAIT, SETUP_DONE);

    -- Set up signals
    signal setup_state        : setup_state_t := SETUP_CMD;
    signal setup_idx          : natural range 0 to SETUP_SEQUENCE'length := 0;
    signal wait_counter       : natural range 0 to RESET_WAIT_CYCLES     := 0;
    signal spi_failure_r      : std_logic := '0';

-- Running FSM Signals
    type running_state_t is (IDLE, SENDING_TS, SENDING_DUMMY, WAIT_FIRST_DUMMY, READING, SENDING);

    -- Running Signals 
    signal running_state      : running_state_t := IDLE;
    signal timestamp_r        : std_logic_vector(((TIMESTAMP_WORDS - 1) * WORD_WIDTH) - 1 downto 0) := (others => '0');   -- (TIMESTAMP_WORDS - 1) is because the first word is send immeditly to the fifo and won't be latched
    signal ts_word_counter    : natural range 0 to TIMESTAMP_WORDS:= 0;
    signal read_bytes_counter : natural range 0 to BYTES_TO_READ := 0;
    signal read_byte_r        : std_logic_vector(7 downto 0) := (others => '0');  -- lacthes the read byte
    signal second_read        : std_logic := '0';
    signal status_reg         : std_logic_vector(7 downto 0) := (others => '0');  -- Holds status register 1 of the mag

begin 

-- Signals to ports assinations 
    m_axis_fifo_tdata  <= m_axis_fifo_tdata_r;
    m_axis_fifo_tvalid <= m_axis_fifo_tvalid_r;
    m_axis_fifo_tlast  <= m_axis_fifo_tlast_r;

    m_axis_spi_tdata  <= m_axis_spi_tdata_r_running  when setup_state = SETUP_DONE else m_axis_spi_tdata_r_setup;
    m_axis_spi_tvalid <= m_axis_spi_tvalid_r_running when setup_state = SETUP_DONE else m_axis_spi_tvalid_r_setup;
    m_axis_spi_tlast  <= m_axis_spi_tlast_r_running  when setup_state = SETUP_DONE else m_axis_spi_tlast_r_setup;

    spi_failure <= spi_failure_r;
    setup_done_dbg <= '1'when setup_state = SETUP_DONE else '0';

    setup_idx_dbg <= std_logic_vector(to_unsigned(setup_idx, setup_idx_dbg'length));
-- Interupt Sync & edge detection process
    int_sync_p : process(clk) 
    begin 
        if rising_edge(clk) then
            if rst_n = '0' then
                imu_int_ff1     <= '0';
                imu_int_ff2     <= '0';
                imu_int_ff2_dly <= '0';
            else
                imu_int_ff1     <= IMU_INT;
                imu_int_ff2     <= imu_int_ff1;
                imu_int_ff2_dly <= imu_int_ff2;
            end if;
        end if;
    end process int_sync_p;

-- Setup FSM (SETUP_CMD, SETUP_DATA, SETUP_VERIFY, SETUP_WAIT_HANDSHAKE, SETUP_WAIT, SETUP_DONE)
    setup_fsm_p : process(clk)
    begin
        if rising_edge(clk) then 
            if rst_n = '0' then
                setup_state <= SETUP_CMD;
                setup_idx <= 0;

                wait_counter  <= 0;
                spi_failure_r <= '0';
                
                m_axis_spi_tdata_r_setup   <= (others => '0');
                m_axis_spi_tvalid_r_setup  <= '0';
                m_axis_spi_tlast_r_setup   <= '0';
            else
                case setup_state is 
                    when SETUP_CMD =>
                        if m_axis_spi_tvalid_r_setup = '0' then  -- either first ever command or handshake already occured
                            if setup_idx < SETUP_SEQUENCE'length  then  
                                if SETUP_SEQUENCE(setup_idx).op = SETUP_WRITE then
                                    m_axis_spi_tdata_r_setup <= WRITE_COM or SETUP_SEQUENCE(setup_idx).addr;
                                else
                                    m_axis_spi_tdata_r_setup <= READ_COM or SETUP_SEQUENCE(setup_idx).addr;
                                end if;
                                m_axis_spi_tvalid_r_setup <= '1'; 
                                m_axis_spi_tlast_r_setup  <= '0';
                                setup_state               <= SETUP_DATA;
                            else
                                setup_state <= SETUP_DONE;     -- in case we are somming from verify or wait, then no handshake and we are done
                            end if;
                                
                        elsif m_axis_spi_tvalid_r_setup = '1' and m_axis_spi_tready = '1' then
                            if setup_idx < SETUP_SEQUENCE'length  then  
                                if SETUP_SEQUENCE(setup_idx).op = SETUP_WRITE then
                                    m_axis_spi_tdata_r_setup <= WRITE_COM or SETUP_SEQUENCE(setup_idx).addr;
                                else
                                    m_axis_spi_tdata_r_setup <= READ_COM or SETUP_SEQUENCE(setup_idx).addr;
                                end if;
                                m_axis_spi_tvalid_r_setup <= '1'; 
                                m_axis_spi_tlast_r_setup  <= '0';
                                setup_state               <= SETUP_DATA;
                            else -- last data point has been received by the master, set valid to 0, and no need to check the next state if the spi is done. it will be checked either way in the running fsm.
                                m_axis_spi_tvalid_r_setup <= '0';                               -- This was a bug, this line was missing, the valid flag was kept high after the recepince handshake of the last data point
                                setup_state               <= SETUP_DONE ;                       -- hand shake occured and the last data byte was sent
                            end if;
                        end if;


                    when SETUP_DATA =>
                        if m_axis_spi_tvalid_r_setup = '1' and m_axis_spi_tready = '1' then -- first command is sent
                            
                            if SETUP_SEQUENCE(setup_idx).op = SETUP_WRITE then
                                m_axis_spi_tdata_r_setup <=  SETUP_SEQUENCE(setup_idx).data;
                            else -- sending Dummy byte if read
                                m_axis_spi_tdata_r_setup <= DUMMY_BYTE;
                            end if;

                            m_axis_spi_tvalid_r_setup <= '1';
                            m_axis_spi_tlast_r_setup  <= '1';

                            if SETUP_SEQUENCE(setup_idx).op = SETUP_WRITE then
                                if SETUP_SEQUENCE(setup_idx).wait_after = '0' then 
                                    setup_state <= SETUP_CMD;
                                    setup_idx <= setup_idx + 1;
                                else -- u need to wait afterwards
                                    setup_state <= SETUP_WAIT_HANDSHAKE;   
                                end if;   
                            else
                                setup_state <= SETUP_WAIT_HANDSHAKE;
                            end if;
                        end if;

                    when SETUP_WAIT_HANDSHAKE => -- the last byte is accepted and being processed
                        if m_axis_spi_tvalid_r_setup = '1' and m_axis_spi_tready = '1' then  -- dummy byte or the last write byte just got accepted by the SPI Master and being processed
                                m_axis_spi_tvalid_r_setup <= '0';

                            if SETUP_SEQUENCE(setup_idx).op = SETUP_READ_VERIFY then -- if there is a wait_after flag for SETUP_READ_VERIFY oP, it is meaningless and will be disregarded
                                setup_state <= SETUP_VERIFY;
                             -- as we only get into this state from two conditions (either we are write with wait_after or send, the other else will be definetly for waiting for write)
                            elsif SETUP_SEQUENCE(setup_idx).op = SETUP_WRITE and SETUP_SEQUENCE(setup_idx).wait_after = '1' then   -- this line just for clarity
                                setup_state   <= SETUP_WAIT;
                            end if;
                        end if;

                    when SETUP_VERIFY =>  -- wait till the spi finsih its transaction
                        if m_axis_spi_tready = '1' then  -- read byte finally here
                            m_axis_spi_tvalid_r_setup <= '0';     --  repeated, does nothing, but good for clearityy
                            if read_byte /= SETUP_SEQUENCE(setup_idx).data then
                                spi_failure_r <=  '1';  
                                
                                if setup_idx > MAF_CONFIG_ROW_IDX then 
                                    setup_idx   <= MAF_CONFIG_ROW_IDX;  -- restart at the first cmd of mag config section
                                    setup_state <= SETUP_CMD;           -- this was a bug, this line was missing, i was reseting the index to restart the setup sequence but never reseted the state back to SETUP_CMD. - now we have a loop till done
                                else
                                    setup_idx     <= 0;                 -- resart from the begining
                                    setup_state   <= SETUP_CMD;         -- this was a bug, this line was missing, i was reseting the index to restart the setup sequence but never reseted the state back to SETUP_CMD. - now we have a loop till done
                                end if;
                            else
                                setup_idx     <= setup_idx + 1;
                                setup_state   <= SETUP_CMD;
                            end if;
                        end if;
                    
                    when SETUP_WAIT =>
                        if m_axis_spi_tready = '1' then  -- SPI Transaction is done and the last byte has been sent and recieved by the salve module
                            if setup_idx = RESET_ROW_IDX then  -- Reset Period
                                if wait_counter < RESET_WAIT_CYCLES then
                                    wait_counter <= wait_counter + 1;
                                else
                                    wait_counter <= 0;
                                    setup_idx           <= setup_idx + 1;
                                    setup_state         <= SETUP_CMD;

                                end if;
                            else
                                if wait_counter < GENERAL_WAIT_CYCLES then
                                    wait_counter <= wait_counter + 1;
                                else
                                    wait_counter <= 0;
                                    setup_state         <= SETUP_CMD;
                                    setup_idx           <= setup_idx + 1;
                                end if;
                            end if;
                        end if;

                    when SETUP_DONE => 
                    
                end case;
            end if;
        end if;
    end process setup_fsm_p;

-- FSM (SETUP, IDLE, SENDING_TS, SENDING_DUMMY, READING, SENDING)
    running_fsm : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                running_state        <= IDLE;
                -- Running signals resets
                m_axis_fifo_tdata_r  <= (others => '0');
                m_axis_fifo_tvalid_r <='0';
                m_axis_fifo_tlast_r  <='0';
                timestamp_r          <= (others => '0');
                ts_word_counter      <= 0;
                read_bytes_counter   <= 0;
                read_byte_r          <= (others => '0');
                second_read          <= '0';      
                status_reg           <= (others => '0');
                m_axis_spi_tdata_r_running   <= (others => '0');
                m_axis_spi_tvalid_r_running  <= '0';
                m_axis_spi_tlast_r_running   <= '0';
            else
                case running_state is
                    when IDLE => 
                        if imu_int_ff2_dly = '1' and imu_int_ff2 = '0' and setup_state = SETUP_DONE then. -- rising edge
                            -- latch the time stamp from 47 down to 0
                            timestamp_r <= timestamp(((TIMESTAMP_WORDS - 1) * WORD_WIDTH) - 1 downto 0);            -- no need to subtract 2 from the time stamp for the delay of the two sync flip flops. 
                                                                                                                    -- This because dt wil be the differnce between timestamps. so the twos will cross out .
                            -- Sending directly timestamp(TS) two Most significant bytes to the fifo
                            m_axis_fifo_tdata_r  <= timestamp((TIMESTAMP_WORDS * WORD_WIDTH) - 1 downto (TIMESTAMP_WORDS * WORD_WIDTH) - WORD_WIDTH);      -- sending the MSByte of the timestamp
                            m_axis_fifo_tvalid_r <= '1';
                            m_axis_fifo_tlast_r  <= '0';
                            ts_word_counter      <= ts_word_counter + 1;
                            running_state        <= SENDING_TS;
                        end if;

                    when SENDING_TS =>   -- takes 4 cycles
                        if ts_word_counter < TIMESTAMP_WORDS  then
                            if m_axis_fifo_tready = '1' and m_axis_fifo_tvalid_r = '1' then 
                                m_axis_fifo_tdata_r  <= timestamp_r((TIMESTAMP_WORDS * WORD_WIDTH) - WORD_WIDTH * ts_word_counter - 1 downto (TIMESTAMP_WORDS * WORD_WIDTH) - WORD_WIDTH * ts_word_counter - WORD_WIDTH);      -- sending the MSByte of the timestamp
                                m_axis_fifo_tvalid_r <= '1';
                                m_axis_fifo_tlast_r  <= '0';
                                ts_word_counter      <= ts_word_counter + 1;                     -- updating words counter counter

                                running_state        <= SENDING_TS;
                            end if;
                        else 
                            -- ts_word_counter <= 0;  -- first bug found. don't reset teh counter unless the handshake occurs
                            if m_axis_fifo_tready = '1' and m_axis_fifo_tvalid_r = '1' then    -- hand shake of the last word occured
                                m_axis_fifo_tvalid_r <= '0';

                                -- Sending the read command to the spi
                                m_axis_spi_tdata_r_running  <= (READ_COM or ICM20948_ACCEL_XOUT_H);
                                m_axis_spi_tvalid_r_running <= '1';
                                m_axis_spi_tlast_r_running  <= '0';
                                ts_word_counter <= 0;                                               -- correct place of this line
                                running_state  <= SENDING_DUMMY;
                             end if;

                        end if;

                    when SENDING_DUMMY => 
                        if m_axis_spi_tvalid_r_running = '1' and m_axis_spi_tready = '1' then  
                            m_axis_spi_tdata_r_running  <= DUMMY_BYTE;
                            m_axis_spi_tvalid_r_running <= '1';
                            m_axis_spi_tlast_r_running  <= '0';
                            running_state       <= WAIT_FIRST_DUMMY;
                        end if;

                    when WAIT_FIRST_DUMMY =>
                            if m_axis_spi_tvalid_r_running = '1' and m_axis_spi_tready = '1' then  -- Dummy byte has been received
                                m_axis_spi_tdata_r_running  <= DUMMY_BYTE;   -- sending the next dummy
                                m_axis_spi_tvalid_r_running <= '1';
                                m_axis_spi_tlast_r_running  <= '0';
                                running_state       <= READING;
                            end if;

                    when READING => 
                        if read_bytes_counter < BYTES_TO_READ then
                            if read_bytes_counter = 22 then
                                if m_axis_spi_tready = '1' then
                                    m_axis_fifo_tdata_r  <= read_byte & status_reg;
                                    m_axis_fifo_tvalid_r <= '1';   
                                    m_axis_fifo_tlast_r  <= '1';
                                    running_state        <= SENDING;
                                    read_bytes_counter  <= read_bytes_counter + 1;
                                end if;
                            else
                                if m_axis_spi_tvalid_r_running = '1' and m_axis_spi_tready = '1' then    -- now each handshake has a return value
                                    if read_bytes_counter = 14 then 
                                            m_axis_spi_tdata_r_running  <= DUMMY_BYTE;
                                            m_axis_spi_tvalid_r_running <= '1';
                                            m_axis_spi_tlast_r_running  <= '0';
                                            read_bytes_counter  <= read_bytes_counter + 1;

                                            status_reg  <= read_byte;
                                            second_read <= '0';
                                        elsif read_bytes_counter = 20 then -- byte 20 reveived, dummy 21 already sent and now send last dummy
                                            -- send the last dummy 
                                            m_axis_spi_tdata_r_running  <= DUMMY_BYTE;
                                            m_axis_spi_tvalid_r_running <= '1';
                                            m_axis_spi_tlast_r_running  <= '1';
                                        
                                            read_bytes_counter  <= read_bytes_counter + 1;
                                            m_axis_fifo_tdata_r <= read_byte & read_byte_r;
                                            m_axis_fifo_tvalid_r <= '1';   
                                            m_axis_fifo_tlast_r  <= '0';
                                            running_state        <= SENDING;

                                        elsif read_bytes_counter = 21 then  -- this unused byte
                                            -- Stoping SPI transaction
                                            m_axis_spi_tvalid_r_running  <= '0'; -- stoping spi transactions, the last byte dummy already sent
                                            second_read          <= '0';
                                            read_bytes_counter   <= read_bytes_counter + 1;  -- to exist the cunter condition
                                        
                                        else
                                            m_axis_spi_tdata_r_running  <= DUMMY_BYTE;
                                            m_axis_spi_tvalid_r_running <= '1';
                                            m_axis_spi_tlast_r_running  <= '0';

                                            read_bytes_counter  <= read_bytes_counter + 1;
                                            second_read         <= not second_read;

                                            if second_read = '1' then
                                                if read_bytes_counter > 14 then   -- byte swaping for mag litle endian, while imu is big
                                                    m_axis_fifo_tdata_r <= read_byte & read_byte_r;
                                                else 
                                                    m_axis_fifo_tdata_r <= read_byte_r & read_byte;
                                                end if;

                                                m_axis_fifo_tvalid_r <= '1';   
                                                m_axis_fifo_tlast_r  <= '0';
                                                running_state        <= SENDING;
                                            else
                                                read_byte_r <= read_byte;                                
                                            end if;

                                        end if;
                                end if;
                            end if;
                        else
                            read_bytes_counter  <=  0;
                            second_read         <= '0';                                            
                            running_state       <= IDLE;
                        end if;

                    when SENDING => 
                        if m_axis_fifo_tready = '1' and m_axis_fifo_tvalid_r = '1' then 
                            m_axis_fifo_tvalid_r <= '0';
                            running_state        <= READING;
                        end if;
                end case;
            end if;
        end if;
    end process running_fsm;

end architecture rtl;
