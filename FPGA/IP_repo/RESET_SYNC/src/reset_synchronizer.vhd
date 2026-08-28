-------------------------------------------------------------------------------
-- @file    reset_synchronizer.vhd
--
-- @brief   Takes a raw, asynchronous external reset source (e.g. a button,
--          active-high: pressed = '1') and produces a properly synchronized
--          reset output for downstream logic.
--
--          Pattern: asynchronous assert, synchronous de-assert.
--            - When async_reset_in = '1', the output asserts reset
--              IMMEDIATELY, with no dependency on clk. This guarantees a
--              safe, instant reset regardless of clock activity.
--            - When async_reset_in returns to '0', the output only
--              de-asserts after SYNC_STAGES consecutive rising edges of
--              clk. This gives full metastability protection on release,
--              since de-assertion is the transition that actually risks
--              sampling a signal mid-change.
--
--          Output polarity is configurable (OUTPUT_ACTIVE_LOW), so the
--          same core can drive either rst or rst_n style consumers
--          without needing Vivado's IP-integrator polarity metadata at
--          all -- this is plain RTL, not a typed BD interface pin, so it
--          does not trigger BD 41-238 style checks.
--
-- @note    If your design has MULTIPLE independent clock domains, do NOT
--          fan this single synchronizer's output into all of them. Each
--          clock domain needs its OWN instance of this synchronizer,
--          clocked by that domain's own clock -- reusing one domain's
--          synchronized output as an input to a different clock domain
--          reintroduces the same cross-domain metastability risk this
--          module exists to avoid.
--
-- @author  Abdelrahman Hewala
-- @note    Supervisor: Prof. Lutz Leutelt
-- @date    2026
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

entity reset_synchronizer is
    generic (
        SYNC_STAGES       : integer range 2 to 4 := 2;    -- sync FF stages; 2 is standard, use 3+ for extra margin
        OUTPUT_ACTIVE_LOW : boolean := true                -- true: rst_n style (0=reset). false: rst style (1=reset)
    );
    port (
        clk            : in  std_logic;   -- reference clock (use the slowest/reference clock in the domain this drives)
        async_reset_in : in  std_logic;   -- raw external reset source, ACTIVE HIGH (button pressed = '1')
        sync_resetn     : out std_logic    -- synchronized reset, polarity per OUTPUT_ACTIVE_LOW
    );
end entity reset_synchronizer;

architecture rtl of reset_synchronizer is
    -- Internal convention: '1' = reset asserted, regardless of external output polarity.
    -- Initialised to all '1' so the design starts in reset at power-up, before
    -- the first clock edge has even occurred.
    signal sync_chain : std_logic_vector(SYNC_STAGES-1 downto 0) := (others => '1');
begin

    process(clk, async_reset_in)
    begin
        if async_reset_in = '1' then
            sync_chain <= (others => '1');            -- asynchronous assert, no clk dependency
        elsif rising_edge(clk) then
            sync_chain <= sync_chain(SYNC_STAGES-2 downto 0) & '0';  -- synchronous shift toward de-assert
        end if;
    end process;

    sync_resetn     <= sync_chain(SYNC_STAGES-1) when not OUTPUT_ACTIVE_LOW
                       else not sync_chain(SYNC_STAGES-1);

end architecture rtl;