-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: ce_pulse_gen
-- // Project Name: EKF with HLS
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description:
-- //   This is a UART controller module. its main purpsoe is to read the data from
-- //   Axis fifo and sequence the data in form of a frame and provides a CHK test.
-- //   The controller Burst the data to an axis serilizer as the word is 16 bits not 8.
-- //   then the serilizer sends the data to the UART IP. all using axis protocol. 
-- //
-- // UART Controller Modes:
-- // the UART controller has two modes:
-- //   1) Mode 0 => will read the ICM data directly from Axis FIFO and bypass the EKF
-- //   2) Mode 1 => will read beside the input data + kalman filter outputs
-- //
-- // Frame layout:
-- //   1) SYNC1 & SYNC2  => both 1 word
-- //   2) MODE and LEN => both 1 word
-- //   3) PAYLOAD
-- //   4) CHK
-- // 
-- // Mode 1 payload  layout (15 words)
-- //   1) time stamp 4 words
-- //   2) accx       1 word
-- //   3) accy       1 word
-- //   4) accz       1 word
-- //   5) gyrox      1 word
-- //   6) gyroy      1 word
-- //   7) gyroz      1 word
-- //   8) temp       1 word
-- //   9) magx       1 word
-- //   10) magy      1 word
-- //   11) magz      1 word
-- //   12) status    1 word
-- //
-- // CHK is a simple XOR for each word sent. 
-- //
-- // Revision 0.01 - File Created
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////

library ieee;
use ieee.std_logic_1164.all;

entity UART_CONTROLLER is
    generic (
        MODE : natural := 0                -- O = bypass filter, 1 or other : add the filter -- can be replaced with a switch later
    );
    port (
        clk   : in  std_logic;
        rst_n : in  std_logic;

        -- AXI-Stream input interface (fifo_out)
        s_axis_fifo_tdata  : in  std_logic_vector(15 downto 0);
        s_axis_fifo_tvalid : in  std_logic;
        s_axis_fifo_tlast  : in  std_logic;
        s_axis_fifo_tready : out std_logic; 


        -- AXI-Stream output (serilizar input)
        m_axis_tdata  : out std_logic_vector(15 downto 0);    
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic

    );
end entity UART_CONTROLLER;


architecture rtl of UART_CONTROLLER is
-- Constants
    constant SYNC   : std_logic_vector(15 downto 0)  := x"AA55";
    constant MODE_0 :std_logic_vector(15 downto 0)   := x"001E";     -- 00 for mode 0, 1E = 30 bytes
    constant MODE_1 :std_logic_vector(15 downto 0)   := x"012A";     -- 01 for mode 1, 2A = 42 bytes

-- Ports signals 
    signal s_axis_fifo_tready_r : std_logic := '1';

    signal m_axis_tvalid_r      : std_logic := '0';
    signal m_axis_tdata_r       : std_logic_vector(15 downto 0) := (others => '0');


-- Controller FSM
    type state_t is (IDLE, SENDING_SYNC,  SENDING_MODE_LEN, SERILIZER_HANDSHAKE, READ_DATA, SEND_DATA, SEND_CHK);
    
    -- FSM signals
    signal state         : state_t := IDLE;
    signal received_word : std_logic_vector(15 downto 0) := (others => '0');            -- latches the first word
    signal received_word_tlast_reg :std_logic := '0';                                   -- stores last signal for transaction
    signal chk_word      : std_logic_vector(15 downto 0) := (others => '0');            -- holds the Check sum word (XOR)

begin

-- Signal to port Assignation
    s_axis_fifo_tready <= s_axis_fifo_tready_r;

    m_axis_tvalid <= m_axis_tvalid_r;
    m_axis_tdata <= m_axis_tdata_r;

-- Controller FSM (IDLE, SENDING_SYNC,  SENDING_MODE_LEN, SERILIZER_HANDSHAKE, READ_DATA, SEND_DATA, SEND_CHK)
    fsm_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                state <= IDLE;

                s_axis_fifo_tready_r <= '1';
                m_axis_tvalid_r      <= '0';
                m_axis_tdata_r       <=  (others => '0');

                received_word        <=  (others => '0');
                chk_word             <=  (others => '0');
                received_word_tlast_reg <= '0';

            else
                case state is 
                    when IDLE =>
                        if s_axis_fifo_tvalid = '1' and s_axis_fifo_tready_r = '1' then
                            s_axis_fifo_tready_r <= '0';                 -- de asserting ready flag
                            received_word        <= s_axis_fifo_tdata;   -- latching the first word
                            chk_word             <= chk_word XOR s_axis_fifo_tdata;

                            m_axis_tdata_r  <= SYNC;                     -- sending Sync command
                            m_axis_tvalid_r <= '1';
                            state <= SENDING_SYNC;
                        end if;

                    when SENDING_SYNC => 
                        if m_axis_tvalid_r = '1' and m_axis_tready = '1' then  -- serilizar recieve the SYNC word and will be sending it 
                            m_axis_tvalid_r <= '1';

                            if MODE = 0 then
                                m_axis_tdata_r <= MODE_0;
                            else 
                                m_axis_tdata_r <= MODE_1;
                            end if;

                            state <= SENDING_MODE_LEN;
                        end if;

                    when SENDING_MODE_LEN => 
                        if m_axis_tvalid_r = '1' and m_axis_tready = '1' then  -- serilizar recieve the MODE word and will be sending it 
                            m_axis_tvalid_r <= '1';
                            m_axis_tdata_r  <= received_word;                      -- sending the firt word latched in the begining

                            state <= SERILIZER_HANDSHAKE;
                        end if;

                    when SERILIZER_HANDSHAKE =>
                        if m_axis_tvalid_r = '1' and m_axis_tready = '1' then  -- last word is received by the serilizer 
                            state           <= READ_DATA;
                            m_axis_tvalid_r <= '0';        -- De asserting the ready flag;

                            s_axis_fifo_tready_r <= '1';   -- we are ready for receiving a new word from the  fifo;
                        end if;

                    when READ_DATA => 
                        if s_axis_fifo_tvalid = '1' and s_axis_fifo_tready_r = '1' then
                            s_axis_fifo_tready_r    <= '0';                             -- de asserting ready fifo flag - stop receiving

                            m_axis_tdata_r          <= s_axis_fifo_tdata;               -- sending the new word directly  to the serilizer
                            m_axis_tvalid_r         <= '1';                             -- raising ready flag for the serilizer
                            received_word_tlast_reg <= s_axis_fifo_tlast;               -- latching TLAST flag of the received word
                            chk_word                <= chk_word XOR s_axis_fifo_tdata;  -- Updating chk sum. 

                            state <= SEND_DATA;
                        end if;

                    when SEND_DATA => 
                        if m_axis_tvalid_r = '1' and m_axis_tready = '1' then  -- serilizar just recieved the word
                            if received_word_tlast_reg = '1' then              -- if this was the last byte. the pay load is done. send chksum
                                s_axis_fifo_tready_r <= '0';                   -- we are not ready yet to recieve a new word. 
                                m_axis_tdata_r       <= chk_word;                   -- send CHK
                                m_axis_tvalid_r      <= '1'; 
                                state                <= SEND_CHK;
                            else 
                                s_axis_fifo_tready_r <= '1';                        -- asserting ready fifo flag - we are back ready to receive data
                                m_axis_tvalid_r      <= '0';                        -- de-asserting the serilizer valid flag, whatever next is not valid
                                state                <= READ_DATA;
                            end if;
                        end if;

                    when SEND_CHK => 
                        if m_axis_tvalid_r = '1' and m_axis_tready = '1' then  -- serilizar just recieved the last word chk sum
                            m_axis_tvalid_r      <= '0'; 
                            s_axis_fifo_tready_r <= '1';                       -- asserting ready fifo flag - we are back ready to receive data
                            chk_word             <= (others => '0');           -- reseting the chk word for the next cycle
                            state                <= IDLE;
                        end if;
                end case;
            end if;
        end if;

    end process fsm_p;
    
end architecture rtl;