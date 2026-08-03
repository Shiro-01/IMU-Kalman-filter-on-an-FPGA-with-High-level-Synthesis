-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala
-- // 
-- // Create Date: 3/08/2026 
-- // Design Name: axistream_uart
-- // Project Name: EKF on FPGA with HLS & Brass Sense project
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description: 
-- // this iP is a full custamiszable axi stream Uart module
-- // Revision 0.01 - File Created, baudtick logic, Rx engine
-- // Revision 0.02 - Added Tx engines + revision
-- // VHDL Version: VHDL-2008
-- //
-- // - Additional Comments:
-- //       - the data is only valid when the valid flag is high! this due to tieing the 
-- //       - Shift in register to the dout port, this applies too on parity_error, frame_error
-- //
-- //       - the output will always be 8 bts regardless of 7 or 8 bits mode. in 7 bits mode
-- //       - u need to drop the MSB manually in ur design.
-- //
-- //       - this codes using VHDL 2008. otherwise will be there compilation error
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;  

entity axistream_uart is 
    generic (
        BAUDRATE      : natural   := 115200;         -- Minmum baud rate = 110
        PARITY_BIT    : character := 'N';            -- Types of Parities: N: none, E: Even and O: Odd, any invalid Char will be treated as "none"
        EIGHT_BIT     : boolean   := true;           -- True : Eight bits mode. False: 7 bits mode, any invalid entery will be treated as True: 8 bits
        TWO_STOP_BITS : boolean   := false;           -- False: one Stop bit / True: two Stop bits, invalid input will be treated as False: one stop bit
        OVERSAMPLE    : natural   := 16;             -- RX oversampling factor (ticks per bit)
        CLK_FREQ_HZ   : natural   := 100_000_000    -- system clock frequency
    );

    Port (
        -- Clock and Resets
        clk          : in std_logic;     -- system clk 100 MHz
        rst_n        : in std_logic;     

        -- UART interface
        uart_txd    : out std_logic;   -- UART TX port
        uart_rxd    : in std_logic;    -- UART TX port

        -- Input interface / Write
        din         : in  std_logic_vector(7 downto 0);
        din_valid   : in  std_logic;   -- AXI Stream valid input flag
        din_ready   : out std_logic;    -- Axi Stream Ready to take input flag

        -- Output Interface / Read
        dout         : out std_logic_vector(7 downto 0);
        dout_valid   : out std_logic;   -- AXI Stream valid output flag / data valid to read
        dout_ready   : in  std_logic;    -- Axi Stream Ready to take input flag
        frame_error  : out std_logic;   -- when FRAME_ERROR = 1, stop bit was invalid 
        parity_error : out std_logic    -- when PARITY_ERROR = 1, parity bit was invalid
    );
    

end entity;

architecture RTL of axistream_uart is

    -- Constants 
    ------------------------------------------------------------------
    -- Sampling clock generation: fractional NCO (phase accumulator)
    -- ---------------------------------------------------------------
    -- Instead of a fixed integer divisor (which bakes in one rounding error
    -- on every tick), we accumulate a fractional increment each clock and
    -- let baud_tick fire on accumulator overflow. The long-run average tick
    -- rate converges to BAUDRATE*OVERSAMPLE with error bounded by 1/2^ACC_WIDTH,
    -- Desired Period -> 2^24. then System period -> step? . it is a ratio question.
    -- Solve it and convert to the Freq domain. u will get the same final eq. of the Step
    -- resulting integer constant is synthesized, no real-valued hardware.
    constant  NCO_ACC_WIDTH     : natural := 24;                    --  Numerically Controlled Oscillator resolution
    constant  NCO_STEP : natural := natural(round((real(BAUDRATE) * real(OVERSAMPLE) / real(CLK_FREQ_HZ)) * (2.0 ** NCO_ACC_WIDTH)));
    
    -- UART RX Cross Domain Sync flipflops
    signal uart_rx_sync_ff1 : std_logic := '0';
    signal uart_rx_sync_ff2 : std_logic := '0';

    -- Frame geometry Signals
    signal parity_bits  : natural range 0 to 1 := 0;
    signal stop_bits    : natural range 1 to 2 := 1;
    signal payload_bits : natural range 7 to 8 := 8;
    signal frame_bits   : natural range 0 to 12 := 10;

    signal nco_acc           : unsigned(NCO_ACC_WIDTH - 1 downto 0) := (others => '0');   -- one extra bit to catch the overflow
    signal baud_tick         : std_logic := '0';    -- pulses once every OVERSAMPLE-th tick period


    -------------------------------------------------------------------
    -- RX Engine 
    -------------------------------------------------------------------
    -- Rx Engine state
    type rx_engine_st_t is (RX_IDLE, RX_SAMPLING, RX_PARITY, RX_STOPBITS, RX_TRANSFER);
    signal rx_engine_state : rx_engine_st_t := RX_IDLE;

    -- RX Tick counter process signals
    signal rx_tick_counter : natural range 0 to OVERSAMPLE - 1 := 0;

    -- RX Engine Signals
    signal rx_bit_counter       : natural range 0 to 8 := 0;
    signal rx_stop_bits_counter  : natural range 0 to 2 := 0;              -- counter to keep tracking how many stop bits should be sampled 
    signal shift_in_register : std_logic_vector(7 downto 0) := (others => '0');

    signal dout_valid_r      : std_logic := '0';
    signal frame_error_r     : std_logic := '0';
    signal parity_error_r    : std_logic := '0';
    signal start_rx_tick_counter: std_logic := '0';                        -- a start signal goes from RX engine FSM to rx tick counter Process.

    -------------------------------------------------------------------
    -- Tx Engine
    -------------------------------------------------------------------
    -- Tx Engine state
    type tx_engine_st_t is (TX_IDLE, TX_SAMPLING, TX_PARITY, TX_STOPBITS);
    signal tx_engine_state : tx_engine_st_t := TX_IDLE;


    -- TX Tick counter process signals
    signal tx_tick_counter : natural range 0 to OVERSAMPLE - 1 := 0;
    
    -- TX Engine Signals
    signal tx_bit_counter       : natural range 0 to 8 := 0;
    signal tx_stop_bits_counter : natural range 0 to 2 := 0;              -- counter to keep tracking how many stop bits should be sampled 
    signal shift_out_register   : std_logic_vector(7 downto 0) := (others => '0');
    
    signal din_ready_r          : std_logic := '0'; 
    signal start_tx_tick_counter: std_logic := '0';                        -- a start signal goes from RX engine FSM to rx tick counter Process.
    signal tx_parity_bit        : std_logic := '0';                        -- latches the Parity bit of the payload. 
    signal uart_txd_r           : std_logic := '1';
--=================================================================--
begin
    -- Frame geometry: concurrent (combinational) signal assignments
    parity_bits  <= 1 when (PARITY_BIT = 'O' or PARITY_BIT = 'E') else 0;
    payload_bits <= 8 when EIGHT_BIT else 7;
    stop_bits    <= 2 when TWO_STOP_BITS else 1;
 
    -- Start bit (1) + payload + parity (0 or 1) + stop bits
    frame_bits   <= 1 + payload_bits + parity_bits + stop_bits;

    -- RX engine Outout Ports assignation 
    dout <= shift_in_register;
    dout_valid <= dout_valid_r;
    frame_error <= frame_error_r;
    parity_error <= parity_error_r;

    -- TX engine Outout Ports assignation 
    din_ready <= din_ready_r;
    uart_txd <= uart_txd_r;

    nco_tick_gen_p : process(clk)
        variable sum : unsigned(NCO_ACC_WIDTH downto 0);   -- one extra bit catches the carry/overflow
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                nco_acc   <= (others => '0');
                baud_tick <= '0';
            else
                sum       := ('0' & nco_acc) + to_unsigned(NCO_STEP, NCO_ACC_WIDTH + 1);
                nco_acc   <= sum(NCO_ACC_WIDTH - 1 downto 0);
                baud_tick <= sum(NCO_ACC_WIDTH);   -- carry-out bit = overflow = tick pulse
            end if;
        end if;
    end process nco_tick_gen_p;

     uart_rx_synchronizer_p : process(clk) is 
     begin
        if rising_edge(clk) then
            if rst_n = '0' then
                uart_rx_sync_ff1 <= '0';
                uart_rx_sync_ff2 <= '0';
            else
                uart_rx_sync_ff1 <= UART_RXD;
                uart_rx_sync_ff2 <= uart_rx_sync_ff1;
            end if;
        end if;
     end process uart_rx_synchronizer_p;

    -------------------------------------------------------------------
        -- RX Engine 
    -------------------------------------------------------------------     
     rx_tick_counter_p : process(clk) is 
     begin
        if rising_edge(clk) then
            if rst_n = '0' then
                rx_tick_counter <= 0;      -- xxxxxxxxx1 because the counter started delayed by 1 cycle due to start_rx_tick_counter update - Wrong, differnt time doamin, here is tick while th other is clkxxxx . zero is correct as intial vlaue
            else
                if start_rx_tick_counter = '1' then
                    if baud_tick = '1' then
                        if rx_tick_counter = OVERSAMPLE - 1 then
                            rx_tick_counter <= 0;
                        else 
                            rx_tick_counter <= rx_tick_counter + 1;
                        end if;
                    end if;
                elsif start_rx_tick_counter = '0' then
                    rx_tick_counter <= 0;
                end if;
            end if;
        end if;
    end process rx_tick_counter_p;

    -- Rx Engine FSM (RX_IDLE, RX_SAMPLING, RX_PARITY, RX_STOPBITS, RX_TRANSFER);
    rx_engine_fsm : process(clk) is 
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                rx_bit_counter       <= 0;
                shift_in_register <= (others => '0');
                dout_valid_r      <= '0';
                rx_engine_state   <= RX_IDLE;
                parity_error_r    <= '0';
                frame_error_r     <= '0';
                start_rx_tick_counter <= '0';
                rx_stop_bits_counter   <= 0;
                
            else
                case rx_engine_state is 
                    when RX_IDLE =>
                        dout_valid_r      <= '0';
                        frame_error_r     <= '0';
                        parity_error_r    <= '0';           -- not necessary as it get updated at each step. but good for clarity

                        if uart_rx_sync_ff2 = '0' and uart_rx_sync_ff1 = '1' then  -- Detecting the falling edge of the rx idle signal
                            start_rx_tick_counter <= '1';
                        end if;

                        if rx_tick_counter = OVERSAMPLE / 2 then
                            if uart_rx_sync_ff2 = '1' then                     -- If this was a glitch. reset the counter and wait again, if was Zero, then start sampling!
                                start_rx_tick_counter <= '0';
                                rx_engine_state <= RX_IDLE;

                            elsif uart_rx_sync_ff2 = '0' then
                                rx_engine_state <= RX_SAMPLING;
                            end if;
                        end if;

                    when RX_SAMPLING => 
                        if rx_bit_counter < payload_bits then 
                            if rx_tick_counter = OVERSAMPLE / 2 then 
                                shift_in_register <= shift_in_register(6 downto 0) & uart_rx_sync_ff2;   -- the output will always be 8 bts regardless of 7 or 8 bits mode. in 7 bits mode u need to drop the MSB
                                rx_bit_counter <= rx_bit_counter + 1;
                            end if;
                        else 
                            rx_bit_counter <= 0;

                            if parity_bits = 0 then 
                                rx_engine_state <= RX_STOPBITS;
                            else
                                rx_engine_state <= RX_PARITY;
                            end if;
                        end if;
                    
                    when RX_PARITY => 
                        if rx_tick_counter = OVERSAMPLE / 2 then 
                            if PARITY_BIT = 'E' then
                                parity_error_r <= ((xor shift_in_register(payload_bits - 1 downto 0)) xor uart_rx_sync_ff2);
                            elsif PARITY_BIT = 'O' then
                                parity_error_r <= not((xor shift_in_register(payload_bits - 1 downto 0)) xor uart_rx_sync_ff2);
                            end if;

                            rx_engine_state <= RX_STOPBITS;
                        end if;
                    
                    when RX_STOPBITS => 
                        if rx_stop_bits_counter < stop_bits then
                            if rx_tick_counter = OVERSAMPLE / 2 then 
                                frame_error_r <= frame_error_r or (not uart_rx_sync_ff2);
                                rx_stop_bits_counter <= rx_stop_bits_counter +1;
                            end if;
                        else 
                            rx_stop_bits_counter <= 0;
                            dout_valid_r <= '1';                -- raising AXI stram valid signal, signaling the data is ready for consuming
                            start_rx_tick_counter <= '0';       -- signal the RX tick counter to stop and reset to it defualt value!
                            rx_engine_state <= RX_TRANSFER;
                        end if;
                    
                    when RX_TRANSFER => 
                        if (dout_ready and dout_valid_r) = '1' then             -- handshake max allawable delay is (fclk /(baudrate * oversamplingrate)) - 1 ticks of fclk 
                            dout_valid_r <= '0';
                            rx_engine_state <= RX_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process rx_engine_fsm;

    -------------------------------------------------------------------
    -- TX Engine 
    -------------------------------------------------------------------     
     tx_tick_counter_p : process(clk) is 
     begin
        if rising_edge(clk) then
            if rst_n = '0' then
                tx_tick_counter <= 0;      -- xxxxxxxxx1 because the counter started delayed by 1 cycle due to start_rx_tick_counter update - Wrong, differnt time doamin, here is tick while th other is clkxxxx . zero is correct as intial vlaue
            else
                if start_tx_tick_counter = '1' then
                    if baud_tick = '1' then
                        if tx_tick_counter = OVERSAMPLE - 1 then
                            tx_tick_counter <= 0;
                        else 
                            tx_tick_counter <= tx_tick_counter + 1;
                        end if;
                    end if;
                elsif start_tx_tick_counter = '0' then
                    tx_tick_counter <= 0;
                end if;
            end if;
        end if;
    end process tx_tick_counter_p;

    -- Tx engine FSM (TX_IDLE, TX_SAMPLING, TX_PARITY, TX_STOPBITS);
    tx_engine_fsm : process(clk) is 
    begin 
        if rising_edge(clk) then 
            if rst_n = '0' then 
                tx_engine_state <= TX_IDLE;
                uart_txd_r <= '1';          -- idle tx line

                start_tx_tick_counter <= '0';
                tx_bit_counter <= 0;

                tx_stop_bits_counter <= 0;
                din_ready_r <= '1';
                shift_out_register <= (others => '0');
                tx_parity_bit <='0';
            else
                case tx_engine_state is 
                    when TX_IDLE =>
                        if din_ready_r = '1' and din_valid = '1' then
                            din_ready_r <= '0';
                            shift_out_register <= din;

                            if parity_bits = 1 then 
                                if PARITY_BIT = 'E' then
                                    tx_parity_bit <= (xor din(payload_bits - 1 downto 0));
                                elsif PARITY_BIT = 'O' then
                                    tx_parity_bit <= not (xor din(payload_bits - 1 downto 0));
                                end if;
                            end if;

                            uart_txd_r <= '0';                                  -- pulling TX line down
                            start_tx_tick_counter <= '1';
                            tx_engine_state <= TX_SAMPLING;

                        else
                            din_ready_r <= '1';
                        end if;

                    when TX_SAMPLING => 
                        if tx_bit_counter < payload_bits then
                            if tx_tick_counter = OVERSAMPLE - 1 then
                                uart_txd_r <= shift_out_register(0);
                                shift_out_register <= '0' & shift_out_register(7 downto 1);
                                tx_bit_counter <= tx_bit_counter + 1;
                            end if;
                        else
                            tx_bit_counter <= 0;

                            if parity_bits = 0 then 
                                tx_engine_state <= TX_STOPBITS;
                            else
                                tx_engine_state <= TX_PARITY;
                            end if;          
                        end if;
                    
                    when TX_PARITY => 
                        if tx_tick_counter = OVERSAMPLE - 1 then
                            uart_txd_r <= tx_parity_bit;
                            tx_engine_state <= TX_STOPBITS;
                        end if;

                    when TX_STOPBITS => 
                        if tx_stop_bits_counter < stop_bits then 
                            if tx_tick_counter = OVERSAMPLE - 1 then
                                uart_txd_r <= '1';
                                tx_stop_bits_counter <= tx_stop_bits_counter +1;
                            end if;

                        else 
                            if tx_tick_counter = OVERSAMPLE - 1 then    -- waiting till the total period of the last stop bit has passed
                                tx_stop_bits_counter <= 0;
                                start_tx_tick_counter <= '0';           -- stop and reset the  tick counter
                                din_ready_r <= '1';                     -- ready for the next transaction
                                tx_engine_state <= TX_IDLE;
                            end if;
                        end if;

                end case;
            end if;
        end if;
    end process tx_engine_fsm;

end architecture RTL;