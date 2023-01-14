-- Copyright (c) 2023 R. FUMERON <fumeron.remi@gmail.com>
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- 1. Redistributions of source code must retain the above copyright notice,
--    this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
-- 3. Neither the name of mosquitto nor the names of its
--    contributors may be used to endorse or promote products derived from
--    this software without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.

library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;

entity tb_lw_uart_tx is
end tb_lw_uart_tx;

architecture tb of tb_lw_uart_tx is

    component lw_uart_tx is
        generic (
            CLK_FREQ    : positive;                  -- Input clock frequency in Hertz
            BAUDRATE    : positive := 9000;          -- UART baudrate (bit/s)
            DATA_BITS   : integer range 5 to 9 := 8; -- Number of data bits
            STOP_BITS   : integer range 1 to 2 := 1; -- Number of stop bits
            PARITY_EN   : boolean := true;           -- Enable the parity bit
            PARITY_TYPE : boolean := true;           -- Parity type, true : odd, false : even
            INPUT_CDC   : boolean := false           -- CDC interface enable
        );
        port (
            clk     : in std_logic;
            rst_n_a : in std_logic;
            -- Interface signals
            data_in  : in  std_logic_vector(DATA_BITS-1 downto 0); -- Input data
            data_vld : in  std_logic;                              -- Input data valid
            data_ack : out std_logic;                              -- Acknowledge the input data
            -- UART signals
            uart_tx : out std_logic; -- UART tx
            uart_en : out std_logic  -- Asserted when the UART interface is outputing data
        );
    end component;

    constant C_CLK_PERIOD : time := 10 ns;

    constant C_CLK_FREQ    : positive := 100000000;     -- Input clock frequency in Hertz
    constant C_BAUDRATE    : positive := 9000;          -- UART baudrate (bit/s)
    constant C_DATA_BITS   : integer range 5 to 9 := 8; -- Number of data bits
    constant C_STOP_BITS   : integer range 1 to 2 := 1; -- Number of stop bits
    constant C_PARITY_EN   : boolean := true;           -- Enable the parity bit
    constant C_PARITY_TYPE : boolean := true;           -- Parity type, true : odd, false : even
    constant C_INPUT_CDC   : boolean := false;           -- CDC interface enable

    signal s_clk     : std_logic;
    signal s_rst_n_a : std_logic;
    -- Interface signals
    signal s_data_in  : std_logic_vector(C_DATA_BITS-1 downto 0); -- Input data
    signal s_data_vld : std_logic;                                -- Input data valid
    signal s_data_ack : std_logic;                                -- Acknowledge the input data
    -- UART signals
    signal s_uart_tx : std_logic; -- UART tx
    signal s_uart_en : std_logic; -- Asserted when the UART interface is outputing data

    signal s_clk_en : boolean := true;

begin

    dut : lw_uart_tx
    generic map (
        CLK_FREQ => C_CLK_FREQ,
        BAUDRATE => C_BAUDRATE,
        DATA_BITS => C_DATA_BITS,
        STOP_BITS => C_STOP_BITS,
        PARITY_EN => C_PARITY_EN,
        PARITY_TYPE => C_PARITY_TYPE,
        INPUT_CDC => C_INPUT_CDC
    )
    port map (
        clk => s_clk,
        rst_n_a => s_rst_n_a,
        data_in => s_data_in,
        data_vld => s_data_vld,
        data_ack => s_data_ack,
        uart_tx => s_uart_tx,
        uart_en => s_uart_en
    );

    -- Clock generation
    p_clk_generation : process
    begin
        s_clk <= '0';
        while s_clk_en loop
            wait for C_CLK_PERIOD / 2;
            s_clk <= '1';
            wait for C_CLK_PERIOD / 2;
            s_clk <= '0';
        end loop;
        wait;
    end process p_clk_generation;

    -- Testbench
    p_testbench : process
    begin
        s_rst_n_a <= '0';

        s_data_in  <= (others => '0');
        s_data_vld <= '0';

        wait for 10*C_CLK_PERIOD;

        -- Rise the reset
        s_rst_n_a <= '1';

        wait for 10*C_CLK_PERIOD;

        s_clk_en <= false;
        wait;
    end process p_testbench;

end tb;