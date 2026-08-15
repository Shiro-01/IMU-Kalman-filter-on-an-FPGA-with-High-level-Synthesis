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
-- // This is a full SPI Master in Mode 3 ((CPOL = 1, CPHA = 1)). Purpsoe of this module is to provide a comunication 
-- // interface with ICM IMU chip. The Module is design to work only at 100 MHz clock
-- //
-- // Known limitation:
-- //    - CLK_DIV_HALF msut be >= 2, SCLK_HIGH_H_TIME, CS_HOLD_TIME >= 1. otherwise, a failure error will raise!
-- //    - This is because start_sclk & hold_sclk needs 1 clk cycle to get udated. and if the half div = 1, the sclk will do
-- //    - one more toggle before it stops, which breaks the logic. The otehr two check, so we don't have negative numbers, as
-- //    - the two other generic terms gets subtracted by 1 in the logic
-- //
-- //    - in case of burst mode, the sclk gets on hold high after the last bit being sample and waits for
-- //    - the axis hand shake so it can proceed with the counter. it does NOT go down imeditly, it needs to count 
-- //    - the minimum high pulse width.
-- // 
-- // Revision 0.01 - File Created
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_master is
    -- Generic parameters
    generic(
        CLK_DIV_HALF     : natural := 10;   -- Half of the target clock divider for SPI clock. if the system clk is 100Mhz. and the desired spi_sclk is 5 MHz, then this value shall be (10). note CLK_DIV_HALF msz be >= 2
        SCLK_HIGH_H_TIME : natural := 50;   -- Min Hold time between sclk rising edge and CS de-assertion in (10 ns). min (500 ns)
        CS_HOLD_TIME     : natural := 5     -- Min time for holding CS line high in (10 ns).so DO is reseted. (min 50 ns)
    );

    port(
        -- clk & rst
        clk : in std_logic;                                  
        rst_n : in std_logic;

        -- AXI Stream Controller interface + read byte
        s_axis_tready  : out  std_logic;                                
        s_axis_tvalid  : in std_logic;                                  
        s_axis_tdata   : in std_logic_vector(7 downto 0);               
        s_axis_tlast   : in std_logic;   

        read_byte      : out std_logic_vector(7 downto 0);

        -- SPI Interface
        spi_sclk : out std_logic;                            -- SPI clock line
        spi_cs_n : out std_logic;                            -- chip select line
        spi_mosi : out std_logic;                            -- Master out slave in spi  line
        spi_miso : in std_logic                              -- MAster in salve out spi line
    );

end spi_master;


architecture rtl of spi_master is 
-- Ports Signals ---------------------------------------------------
    --  SPI Interface signals
    signal spi_sclk_r : std_logic := '1';
    signal spi_cs_n_r : std_logic := '1';
    signal spi_mosi_r : std_logic := '0';

    -- Controller interface Signals
    signal s_axis_tready_r : std_logic := '0';

-- SPI FSM 
    type spi_fsm_st_t is (IDLE, RUNNING, SCLK_HOLD, CS_HOLD);

-- SPI sclk Generation process signals
    signal sclk_counter : natural range 0 to CLK_DIV_HALF - 1 := CLK_DIV_HALF - 1;

-- SPI FSM Signals 
    signal spi_fsm_st     : spi_fsm_st_t := IDLE;
    signal shift_out_reg  : std_logic_vector(7 downto 0) := (others => '0');
    signal shift_in_reg   : std_logic_vector(7 downto 0) := (others => '0');
    signal s_axis_tlast_r : std_logic := '1';
    signal start_sclk     : std_logic := '0';
    signal hold_sclk      : std_logic := '0';

    signal bits_counter      : natural range 0 to 8 := 0;
    signal sclk_hold_counter : natural range 0 to SCLK_HIGH_H_TIME - 1 := 0;
    signal cs_hold_counter   : natural range 0 to CS_HOLD_TIME - 1 := 0;
begin

-- Generic sanity checks (happen at elaboration-time only with zero hardware cost)
    assert CLK_DIV_HALF >= 2
        report "CLK_DIV_HALF must be >= 2 -- at 1, the sclk generator produces spurious extra edges before start_sclk/hold_sclk can react"
        severity failure;

    assert SCLK_HIGH_H_TIME >= 1 and CS_HOLD_TIME >= 1
        report "SCLK_HIGH_H_TIME and CS_HOLD_TIME must each be >= 1"
        severity failure;

-- Signals to ports assinations 
    spi_sclk <= spi_sclk_r;
    spi_cs_n <= spi_cs_n_r;
    spi_mosi <= spi_mosi_r;
    s_axis_tready <= s_axis_tready_r;

-- SPI sclk Generation process 
    sclk_generation_p : process(clk) 
    begin
        if rising_edge(clk) then 
            if rst_n = '0' then 
                sclk_counter <= CLK_DIV_HALF - 1;
                spi_sclk_r <= '1';
            else
                if start_sclk = '1' then
                    if hold_sclk = '0' then 
                        if sclk_counter = CLK_DIV_HALF - 1 then
                            spi_sclk_r <= not spi_sclk_r;
                            sclk_counter <= 0;
                        else
                            sclk_counter <= sclk_counter + 1;
                        end if;
                    end if;
                else
                    sclk_counter <= CLK_DIV_HALF - 1;
                    spi_sclk_r <= '1';
                end if;
            end if;
        end if;
    end process sclk_generation_p;

-- SPI Master FSM (IDLE, RUNNING, SCLK_HOLD, CS_HOLD)
    spi_master_fsm : process (clk) 
    begin
        if rising_edge(clk) then
            if rst_n = '0' then 
                spi_fsm_st      <= IDLE;
                spi_cs_n_r      <= '1';
                spi_mosi_r      <= '0';
                s_axis_tready_r <= '1';

                shift_out_reg  <= (others => '0');
                shift_in_reg   <= (others => '0');
                read_byte      <= (others => '0');

                s_axis_tlast_r <= '0';
                start_sclk     <= '0';   
                hold_sclk      <= '0'; 

                bits_counter      <= 0;    
                sclk_hold_counter <= 0;    
                cs_hold_counter   <= 0;
            else
                case spi_fsm_st is
                    when IDLE => 
                        if s_axis_tvalid = '1' and s_axis_tready_r = '1' then
                            s_axis_tready_r <= '0';
                            spi_cs_n_r      <= '0';
                            start_sclk      <= '1';         -- send start singal to sclk generation process
                                                            -- as it takes 1 clk to get updates. this satisfies the min duration
                                                            -- between cs low and sck low
                            hold_sclk       <= '0';         -- unhold sclk if any. clock is free to run again

                            shift_out_reg  <= s_axis_tdata; -- latch the sent byte
                            s_axis_tlast_r <= s_axis_tlast; -- latch tlast flag
                            spi_fsm_st     <= RUNNING;
                        else
                            s_axis_tready_r <= '1';
                        end if;

                    when RUNNING =>
                        if bits_counter < 8 then
                            if sclk_counter =  CLK_DIV_HALF - 1 and spi_sclk_r = '1' then     -- if falling edge, Shift out the data
                                spi_mosi_r    <= shift_out_reg(7);
                                shift_out_reg <= shift_out_reg(6 downto 0) & '0';
                            
                            elsif sclk_counter =  CLK_DIV_HALF - 1 and spi_sclk_r = '0' then  -- if rising edge, shift in the miso line
                                 shift_in_reg <=  shift_in_reg(6 downto 0) & spi_miso;
                                bits_counter <= bits_counter + 1;
                            end if;
                        else
                            bits_counter <= 0;
                            read_byte <= shift_in_reg;

                            if s_axis_tlast_r = '1' then 
                                start_sclk <= '0';
                                spi_fsm_st <= SCLK_HOLD;
                            else
                                hold_sclk <= '1';
                                s_axis_tready_r <= '1';
                                spi_fsm_st <= IDLE;
                            end if;

                        end if;

                    when SCLK_HOLD =>  -- Holding sclk line high
                        if sclk_hold_counter = SCLK_HIGH_H_TIME - 1 then
                            sclk_hold_counter <= 0;
                            spi_cs_n_r        <= '1';
                            spi_fsm_st        <= CS_HOLD;
                        else
                            sclk_hold_counter <= sclk_hold_counter + 1;
                        end if;

                    when CS_HOLD => 
                        if cs_hold_counter = CS_HOLD_TIME - 1 then
                            cs_hold_counter <= 0;
                            spi_fsm_st <= IDLE;
                            s_axis_tready_r <= '1';
                        else
                            cs_hold_counter <= cs_hold_counter + 1;
                        end if;
                end case;
            end if;
        end if;
    end process spi_master_fsm;
end architecture rtl;
