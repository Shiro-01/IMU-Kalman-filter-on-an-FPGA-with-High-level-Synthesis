-- //////////////////////////////////////////////////////////////////////////////////
-- // Engineer: Abdelrahman Hewala (Shiro)
-- //
-- // Design Name: fifo_axis
-- // Project Name: Brass Sense project
-- // Target Devices: Xilinx Basys 3
-- // Tool Versions: Vivado 2026.1
-- //
-- // Description:
-- // Generic synchronous FIFO. Plain write port in; AXI-Stream on the
-- // read side out (tvalid asserted whenever count>0, first-word-fall-
-- // through -- no extra read-enable latency, tdata is always the head
-- // element as soon as it's available).
-- //
-- // Count-based full/empty tracking (not a power-of-2 pointer-wrap
-- // trick) since DEPTH isn't required to be a power of 2.
-- //
-- // AXI4-Stream requires TDATA to be a whole number of bytes. Internal
-- // storage stays at DATA_WIDTH (12 bits for the ADCS7476 payload);
-- // only the m_axis_tdata port is padded up to the next byte boundary
-- // with zeros in the unused high bits, computed generically so this
-- // doesn't need revisiting if DATA_WIDTH ever changes (e.g. a 10-bit
-- // ADCS7477).
-- //
-- // Revision 0.02 - Pad m_axis_tdata to a byte-aligned width per
-- //                  AXI4-Stream spec; internal DATA_WIDTH unchanged
-- // VHDL Version: VHDL-2008
-- //////////////////////////////////////////////////////////////////////////////////
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity fifo_axis is
    generic (
        DEPTH      : natural := 1000;
        DATA_WIDTH : natural := 12    -- actual payload width (internal storage)
    );
    port (
        clk   : in std_logic;
        rst_n : in std_logic;

        -- Write side (plain, DATA_WIDTH bits -- unpadded)
        wr_en   : in  std_logic;
        wr_data : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
        full    : out std_logic;

        -- Read side (AXI-Stream, padded up to the next byte boundary)
        m_axis_tdata  : out std_logic_vector((((DATA_WIDTH + 7) / 8) * 8) - 1 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tready : in  std_logic
    );
end entity;

architecture rtl of fifo_axis is

    constant AXIS_TDATA_WIDTH : natural := ((DATA_WIDTH + 7) / 8) * 8;   -- next multiple of 8 >= DATA_WIDTH

    type mem_array_t is array (0 to DEPTH - 1) of std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal mem : mem_array_t := (others => (others => '0'));

    signal wr_ptr : natural range 0 to DEPTH - 1 := 0;
    signal rd_ptr : natural range 0 to DEPTH - 1 := 0;
    signal count  : natural range 0 to DEPTH      := 0;

begin

    full          <= '1' when count = DEPTH else '0';
    m_axis_tvalid <= '1' when count > 0     else '0';
    m_axis_tdata  <= std_logic_vector(resize(unsigned(mem(rd_ptr)), AXIS_TDATA_WIDTH));

    fifo_p : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                wr_ptr <= 0;
                rd_ptr <= 0;
                count  <= 0;
            else
                -- Write (only if there's room; otherwise silently ignored --
                -- caller is expected to check 'full' before asserting wr_en)
                if wr_en = '1' and count < DEPTH then
                    mem(wr_ptr) <= wr_data;
                    if wr_ptr = DEPTH - 1 then
                        wr_ptr <= 0;
                    else
                        wr_ptr <= wr_ptr + 1;
                    end if;
                end if;

                -- Pop (AXI-Stream handshake)
                if count > 0 and m_axis_tready = '1' then
                    if rd_ptr = DEPTH - 1 then
                        rd_ptr <= 0;
                    else
                        rd_ptr <= rd_ptr + 1;
                    end if;
                end if;

                -- count update: handles a simultaneous push+pop correctly
                if (wr_en = '1' and count < DEPTH) and (count > 0 and m_axis_tready = '1') then
                    count <= count;               -- one in, one out: no net change
                elsif (wr_en = '1' and count < DEPTH) then
                    count <= count + 1;
                elsif (count > 0 and m_axis_tready = '1') then
                    count <= count - 1;
                end if;
            end if;
        end if;
    end process fifo_p;

end architecture rtl;