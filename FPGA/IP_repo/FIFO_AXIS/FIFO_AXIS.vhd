--------------------------------------------------------------------------------------
-- Engineer: Abdelrahman Hewala (Shiro)
--
-- Design Name: axis_fifo
-- Project Name: EKF with HLS
-- Target Devices: Xilinx Basys 3
-- Tool Versions: Vivado 2026.1
--
-- Description:
--   Generic synchronous AXI4-Stream FIFO. Both the slave (write) side and the master
--   (read) side carry tlast, which is stored in the FIFO alongside tdata and comes out
--   in order with its associated word (i.e. tlast is treated as one extra data bit,
--   not tracked separately). Depth and word width are both generic. Full/empty are
--   tracked with an explicit occupancy counter so FIFO_DEPTH does not need to be a
--   power of two.
--
-- Generics:
--   DATA_WIDTH : width of tdata, in bits.
--   FIFO_DEPTH : number of entries the FIFO can hold (>= 2).
--
-- Interfaces:
--   s_axis_* : AXI4-Stream slave (write) port.
--   m_axis_* : AXI4-Stream master (read) port.
--   full     : status flag, '1' when the FIFO is completely full (count = FIFO_DEPTH),
--              i.e. the same condition that drives s_axis_tready low. Provided as a
--              direct status bit for logic that wants it without deriving it from
--              the inverse of tready.
--
-- Revision 0.01 - File Created
-- VHDL Version: VHDL-2008
--------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity axis_fifo is
    generic (
        WORD_WIDTH : natural := 16;    -- tdata width in bits
        FIFO_DEPTH : natural := 16     -- number of entries (>= 2)
    );
    port (
        clk   : in std_logic;
        rst_n : in std_logic;

        -- AXI4-Stream slave (write) side
        s_axis_tdata  : in  std_logic_vector(WORD_WIDTH - 1 downto 0);
        s_axis_tvalid : in  std_logic;
        s_axis_tready : out std_logic;
        s_axis_tlast  : in  std_logic;

        -- AXI4-Stream master (read) side
        m_axis_tdata  : out std_logic_vector(WORD_WIDTH - 1 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic;
        m_axis_tlast  : out std_logic;

        -- Status
        full : out std_logic     -- '1' when count = FIFO_DEPTH (s_axis_tready would be '0')
    );
end entity axis_fifo;

architecture rtl of axis_fifo is

    -- one extra bit stored per entry to carry tlast alongside tdata
    type ram_t is array (0 to FIFO_DEPTH - 1) of std_logic_vector(WORD_WIDTH downto 0);
    signal mem : ram_t := (others => (others => '0'));

    signal wr_ptr : natural range 0 to FIFO_DEPTH - 1 := 0;
    signal rd_ptr : natural range 0 to FIFO_DEPTH - 1 := 0;
    signal count  : natural range 0 to FIFO_DEPTH := 0;

    signal wr_en : std_logic;
    signal rd_en : std_logic;

    signal s_axis_tready_r : std_logic := '0';
    signal m_axis_tvalid_r : std_logic := '0';

begin

    -- elaboration-time sanity checks
    assert WORD_WIDTH >= 1
        report "axis_fifo: WORD_WIDTH must be >= 1" severity failure;
    assert FIFO_DEPTH >= 2
        report "axis_fifo: FIFO_DEPTH must be >= 2" severity failure;

    wr_en <= s_axis_tvalid and s_axis_tready_r;
    rd_en <= m_axis_tvalid_r and m_axis_tready;

    s_axis_tready_r <= '1' when count < FIFO_DEPTH else '0';
    m_axis_tvalid_r <= '1' when count > 0 else '0';

    s_axis_tready <= s_axis_tready_r;
    m_axis_tvalid <= m_axis_tvalid_r;

    full <= '1' when count = FIFO_DEPTH else '0';

    -- combinational read of the head entry; qualified by tvalid at the consumer side
    m_axis_tdata <= mem(rd_ptr)(WORD_WIDTH - 1 downto 0);
    m_axis_tlast <= mem(rd_ptr)(WORD_WIDTH);

    fifo_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                wr_ptr <= 0;
                rd_ptr <= 0;
                count  <= 0;
            else
                -- write side
                if wr_en = '1' then
                    mem(wr_ptr) <= s_axis_tlast & s_axis_tdata;
                    if wr_ptr = FIFO_DEPTH - 1 then
                        wr_ptr <= 0;
                    else
                        wr_ptr <= wr_ptr + 1;
                    end if;
                end if;

                -- read side
                if rd_en = '1' then
                    if rd_ptr = FIFO_DEPTH - 1 then
                        rd_ptr <= 0;
                    else
                        rd_ptr <= rd_ptr + 1;
                    end if;
                end if;

                -- occupancy counter: handles simultaneous push+pop (net change = 0)
                if wr_en = '1' and rd_en = '0' then
                    count <= count + 1;
                elsif wr_en = '0' and rd_en = '1' then
                    count <= count - 1;
                end if;
            end if;
        end if;
    end process fifo_p;

end architecture rtl;