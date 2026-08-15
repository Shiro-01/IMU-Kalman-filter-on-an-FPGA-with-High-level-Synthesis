-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: spi_master
-- // Project Name: EKF with HLS
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description:
-- // 
-- // This is a full SPI Master in Mode 3 ((CPOL = 1, CPHA = 1)). with two engines MOSI and MISO. Purpsoe
-- // of this module is to provide a comunication interface with ICM IMU chip
-- //
-- // Known limitation:
-- //    - in case of streaming several bytes. the controller must provide the next
-- //    - byte immediatly or any time less than (200 ns (8 sclk periods) * 8) = 1.6 µsec since the percious handshake
-- //    - Otherwise, the system will send whatever last bit was on MOSI line to the slave instead. 
-- //    - this is due to the fact that if tlast was 0 (not the last byte), the sclk generator won't top counting
-- //    - and will procced as normal
-- //
-- // Revision 0.01 - File Created
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;

entity spi_master is
    generic (
        CLK_DIV_HALF     : natural := 10;  -- system clk cycles per SPI half-period. SCLK freq = clk / (2*CLK_DIV_HALF). 
                                           -- Default: 100MHz / (2*10) = 5 MHz SCLK. will be using 5 MHz. (max possible 6.6666 MHz)
        CS_HOLD_TIME     : natural := 5;   -- Min time for holding CS line high in (10 ns).so DO is reseted. (min 50 ns)
        SCLK_HIGH_H_TIME : natural := 50   -- Min Hold time between sclk rising edge and CS de-assertion in (10 ns). min (500 ns)
    );

    port (
        clk    : in  std_logic;
        rst_n  : in  std_logic;    -- active-low synchronous reset

        -- AXI Stream input interface
        s_axis_tready  : out  std_logic;                                
        s_axis_tvalid  : in std_logic;                                  
        s_axis_tdata   : in std_logic_vector(7 downto 0);               
        s_axis_tlast   : in std_logic;                                  -- Flags the last byte.

        -- AXI Stream Output interface
        m_axis_tready  : in  std_logic;                                
        m_axis_tvalid  : out std_logic;                                  
        m_axis_tdata   : out std_logic_vector(7 downto 0);               
        m_axis_tlast   : out std_logic;                                  -- Flags the last byte.

        -- SPI interface the OLED Screen
        spi_sclk  : out std_logic;     -- Mode 3
        spi_cs_n  : out std_logic;     -- active low
        spi_mosi  : out std_logic;     -- SDATA
        spi_miso  : in std_logic      -- DO
    );

end entity spi_master;

architecture rtl of spi_master is

-- Ports' Signals
    -- spi Ports' Signals
    signal spi_sclk_r : std_logic := '1';
    signal spi_cs_n_r : std_logic := '1';
    signal spi_mosi_r : std_logic := '0';

    -- Axis Ports' Signals
    signal s_axis_tready_r : std_logic := '1';
    signal m_axis_tvalid_r : std_logic := '0';
    signal m_axis_tdata_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal m_axis_tlast_r  : std_logic := '1';

-- Sclk Generation signals
    signal sclk_counter : natural range 0 to CLK_DIV_HALF - 1 := CLK_DIV_HALF - 1;

-- MOSI FSM
    type mosi_engine_st_t is (IDLE, STREAMING, WAITING_RISING_EDGE, SCLK_HOLD, CS_HOLD);

    -- MOSI FSM Signals
    signal mosi_engine_st : mosi_engine_st_t := IDLE;
    signal start_spi_sclk : std_logic := '0';
    signal shift_out_reg  : std_logic_vector(7 downto 0) := (others => '0');
    signal bits_counter   : natural range 0 to 8 := 0;
    signal cshold_counter : natural range 0 to CS_HOLD_TIME - 1 := 0;
    signal s_axis_tlast_r : std_logic := '0';
    signal sclk_hold_counter : natural range 0 to SCLK_HIGH_H_TIME - 1 := 0;

-- MISO FSM
    type miso_engine_st_t is (IDLE, RECIEVING);

    -- MOSI FSM Signals
    signal miso_engine_st : miso_engine_st_t := IDLE;



begin
-- Signal to Ports assignation
    -- spi Ports' Signals assignation
    spi_sclk <= spi_sclk_r;
    spi_cs_n <= spi_cs_n_r;
    spi_mosi <= spi_mosi_r;

    -- Axis Ports' Signals assignation
    s_axis_tready <= s_axis_tready_r;
    m_axis_tvalid <= m_axis_tvalid_r;
    m_axis_tdata  <= m_axis_tdata_r;
    m_axis_tlast  <= m_axis_tlast_r;

    sclk_generation_p : process(clk) 
    begin 
        if rising_edge(clk) then
            if rst_n = '0' then
                sclk_counter <= CLK_DIV_HALF - 1;
                spi_sclk_r <= '1';
            else
                if start_spi_sclk = '1' then
                    if sclk_counter =  CLK_DIV_HALF - 1 then
                        spi_sclk_r <= not spi_sclk_r;
                        sclk_counter <= 0;
                    else
                        sclk_counter <= sclk_counter + 1;
                    end if;

                elsif start_spi_sclk = '0' then 
                    spi_sclk_r <= '1';
                    sclk_counter <= CLK_DIV_HALF - 1;
                end if;
            end if;
        end if;
    end process sclk_generation_p;


    mosi_engine_fsm_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                mosi_engine_st    <= IDLE;
                start_spi_sclk    <= '0';
                spi_cs_n_r        <= '1';
                spi_mosi_r        <= '0';
                s_axis_tready_r   <= '1';
                s_axis_tlast_r    <= '0'; 
                bits_counter      <= 0;
                sclk_hold_counter <= 0;
                cshold_counter    <= 0;
                shift_out_reg  <= (others => '0');
            else
                case mosi_engine_st is
                    when IDLE => 
                        if s_axis_tready_r  = '1' and s_axis_tvalid = '1' then 
                            spi_cs_n_r     <= '0';
                            s_axis_tready_r  <= '0';
                            start_spi_sclk <= '1';                            -- the clock wills start the next sclk after CS line goes low. which statisfy the 5 ns min set up time
                            shift_out_reg  <= s_axis_tdata(7 downto 0);         -- latch the stream byte!
                            s_axis_tlast_r   <= s_axis_tlast;                     -- Latch Tlast Flag
                            
                            mosi_engine_st <= STREAMING;
                        end if;
                    when STREAMING => 
                        if bits_counter < 8 then
                            if sclk_counter = CLK_DIV_HALF - 1 and spi_sclk_r = '1' then  -- this moment is exactly the moment where the falling edge occur!
                                spi_mosi_r    <= shift_out_reg(7);
                                shift_out_reg <= shift_out_reg(6 downto 0) & '0';
                                bits_counter  <= bits_counter + 1;
                            end if;
                        else
                            bits_counter <= 0;
                            if s_axis_tlast_r = '1' then 
                                mosi_engine_st <= WAITING_RISING_EDGE;                      -- wait for the last rising edge.
                                s_axis_tlast_r <= '0';
                            else 
                                s_axis_tready_r <= '1';
                                mosi_engine_st <= IDLE;
                            end if;
                        end if;

                    when WAITING_RISING_EDGE =>
                        if sclk_counter = CLK_DIV_HALF - 1 and spi_sclk_r = '0' then     -- this moment is exactly the moment where the rising edge occur! aka.. oled sampled the last bit
                            start_spi_sclk <= '0';                                       -- this was the last rising edge - stop the sclk generator
                            mosi_engine_st <= SCLK_HOLD;                                 -- minimum sclk hold high before desserting cs
                        end if;

                    when SCLK_HOLD => 
                        if sclk_hold_counter = SCLK_HIGH_H_TIME - 1 then
                            sclk_hold_counter <= 0;
                            spi_cs_n_r     <= '1';
                            mosi_engine_st <= CSHOLD;
                        else
                            sclk_hold_counter <= sclk_hold_counter + 1; 
                        end if;

                    when CS_HOLD => 
                        if cshold_counter = CS_HOLD_TIME - 1 then
                            cshold_counter <= 0;
                            s_axis_tready_r  <= '1';     
                            mosi_engine_st <= IDLE;
                        else
                            cshold_counter <= cshold_counter + 1;
                        end if;
                end case;
            end if;             
        end if;
    end process mosi_engine_fsm_p;

end architecture rtl;


