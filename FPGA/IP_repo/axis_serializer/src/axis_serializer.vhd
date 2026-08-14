-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: axis_serializer.vhd
-- // Project Name: EKF by HLS
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description:
-- // This ip is Desiened to take the data from the SPI Controller and 
-- // Pass it to the UART Prieferal. The necisity of this Module is because
-- // the payload is n*8 bits. while uart protocol is just 7/8 bit. so This
-- // ip handels the serialization process and burst the data in form of 8 bits burts. 
-- //
-- // the IP has 2 AXI-Stream interfaces. One 16 bit wide (adjsutable) for the SPI samle (input)  
-- // and another 8 bits wide for the UART side (output).
-- // 
-- // ** -- Note
-- //       -- This UART controller designed to segment the payload into mutlipules of
-- //          8 bits. and Accordingly, it works only for 8 bits Mode.
-- //
-- // Revision 0.01
-- //
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;

entity axis_serializer is
    generic (
        DATA_WIDTH : natural := 16      -- Choose a width that match ur input side. must be multiple of 8 bits
                                        -- if not will be approximated to the upper closest 8 bits multipule
    );
    port (
        clk   : in std_logic;
        rst_n : in std_logic;


        -- AXI-Stream input interface
        s_axis_tdata  : in  std_logic_vector((((DATA_WIDTH + 7) / 8) * 8) - 1 downto 0);
        s_axis_tvalid : in  std_logic;
        s_axis_tready : out std_logic;

        -- AXI-Stream output (byte-aligned, see note above)
        m_axis_tdata  : out std_logic_vector(7 downto 0);     -- Output always 8 bits for UART -- not if you are in 7 bit
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic
    );
end entity axis_serializer;

architecture rtl of axis_serializer is

    -- CONSTANTS
    constant TOTAL_ITERATIONS : natural := (DATA_WIDTH + 7) / 8;

    -- FSM
    type state_t is (IDLE, SENDING);
    signal state : state_t := IDLE;

    -- Signals
    signal iteration_count : natural range 0 to TOTAL_ITERATIONS := 0;

    signal s_axis_tready_r : std_logic := '1';
    signal s_axis_tdata_r  : std_logic_vector((((DATA_WIDTH + 7) / 8) * 8) - 1 downto 0) := (others => '0');    -- A register to latch the input payload bits
    signal m_axis_tdata_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal m_axis_tvalid_r : std_logic := '0';    
begin

    -- Signals to ports assignations
    s_axis_tready <= s_axis_tready_r;
    m_axis_tdata <= m_axis_tdata_r;
    m_axis_tvalid <= m_axis_tvalid_r;

    -- axis_serializer FSM
    axis_serializer_p : process(clk) is
    begin 
        if rising_edge(clk) then
            if rst_n = '0' then
                state <= IDLE;
                iteration_count <= 0;

                s_axis_tready_r <= '1';
                s_axis_tdata_r  <= (others => '0');
                m_axis_tdata_r  <= (others => '0');
                m_axis_tvalid_r <= '0';
            else
                case state is 
                    when IDLE => 
                        if s_axis_tready_r = '1' and s_axis_tvalid = '1' then
                            -- reciever side
                            s_axis_tready_r <= '0';
                            s_axis_tdata_r <= s_axis_tdata;     -- LAtching the payload
                            
                            -- uart side, sending the MSB
                            m_axis_tdata_r <= s_axis_tdata((((DATA_WIDTH + 7) / 8) * 8) - 1 downto (((DATA_WIDTH + 7) / 8) * 8) - 8);       -- sending the first Most significant byte 
                            m_axis_tvalid_r <='1';

                            iteration_count <= iteration_count + 1;

                            state <= SENDING;
                        else
                            s_axis_tready_r <= '1';         -- aditional line not neceeray but keept for clearity
                        end if;

                    when SENDING => 
                        if m_axis_tready = '1' and m_axis_tvalid_r = '1' then 
                            if iteration_count < TOTAL_ITERATIONS then
                                m_axis_tdata_r <= s_axis_tdata_r((((DATA_WIDTH + 7) / 8) * 8) - iteration_count * 8 - 1 downto (((DATA_WIDTH + 7) / 8) * 8)  - iteration_count * 8 - 8);       -- sending the first Most significant byte 
                                m_axis_tvalid_r  <='1';

                                iteration_count <= iteration_count + 1;
                            else
                                iteration_count <= 0;
                                m_axis_tvalid_r  <='0';
                                s_axis_tready_r <= '1';
                                state <= IDLE;
                            end if;
                        end if;
                end case;
            end if;
        end if;
    end process axis_serializer_p;
end architecture rtl;
