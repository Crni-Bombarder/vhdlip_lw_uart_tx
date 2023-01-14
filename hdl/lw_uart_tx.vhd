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

entity lw_uart_tx is
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
end lw_uart_tx;

architecture rtl of lw_uart_tx is

    constant C_UART_IDLE_VALUE  : std_logic := '1';
    constant C_UART_START_VALUE : std_logic := '0';
    constant C_UART_STOP_VALUE  : std_logic := '1';
    
    constant C_BAUDRATE_CPT_VALUE : integer := CLK_FREQ / BAUDRATE;

    signal s_cpt_baudrate_start : std_logic;
    signal s_cpt_baudrate       : integer; -- Constraint by synthesize
    signal s_top_bit            : std_logic;

    signal s_data_vld : std_logic;
    signal s_data_valid_state : std_logic;
    signal s_data_ack : std_logic;
    signal s_data_uart : std_logic_vector(DATA_BITS-1 downto 0);
    signal s_data_parity : std_logic;
    constant C_CPT_DATA_BIT_SIZE : integer := 4;
    signal s_cpt_data_bit : unsigned(C_CPT_DATA_BIT_SIZE-1 downto 0);

    type state_t is (S_IDLE, S_START_BIT, S_DATA_BITS, S_PARITY_BIT, S_STOP_BITS);
    signal s_fsm_state : state_t;

    function f_parity_computation(data : std_logic_vector) return std_logic is
        variable v_parity : std_logic;
    begin
        if PARITY_TYPE then
            v_parity := '1';
        else
            v_parity := '0';
        end if;
        for I in 0 to DATA_BITS-1 loop
            v_parity := v_parity xor data(I);
        end loop;
        return v_parity;
    end function;

begin

    g_top_bit_cpt_process : if C_BAUDRATE_CPT_VALUE > 1 generate
        p_top_bit_generation : process(clk, rst_n_a) is
        begin
            if rst_n_a = '0' then
                s_cpt_baudrate <= 0;
                s_top_bit <= '0';
            elsif rising_edge(clk) then
                if s_cpt_baudrate_start = '1' then
                    if s_cpt_baudrate = C_BAUDRATE_CPT_VALUE - 1 then
                        s_cpt_baudrate <= 0;
                        s_top_bit <= '1';
                    else
                        s_cpt_baudrate <= s_cpt_baudrate + 1;
                        s_top_bit <= '0';
                    end if;
                else
                    s_cpt_baudrate <= 0;
                    s_top_bit <= '0';
                end if;
            end if; -- Rising_edge
        end process p_top_bit_generation;
    end generate g_top_bit_cpt_process;

    g_top_bit_const : if C_BAUDRATE_CPT_VALUE = 0 generate
        s_top_bit <= '1';
    end generate g_top_bit_const;

    p_fsm_state : process(clk, rst_n_a) is
    begin
        if rst_n_a = '0' then
            s_fsm_state <= S_IDLE;

            s_data_valid_state <= '0';
            s_data_uart <= (others => '0');
            s_data_parity <= '0';
            s_data_ack <= '0';
            s_cpt_baudrate_start <= '0';

            s_cpt_data_bit <= (others => '0');

            uart_tx <= C_UART_IDLE_VALUE;
            uart_en <= '0';
        elsif rising_edge(clk) then
            case s_fsm_state is
                -- IDLE : Waiting for a new data
                when S_IDLE =>
                    if s_data_valid_state = not s_data_vld then
                        -- If new data available, start baudrate cpt and save data_vld state
                        s_data_valid_state   <= not s_data_valid_state;
                        s_data_uart          <= data_in;
                        s_data_parity        <= f_parity_computation(data_in);
                        s_cpt_baudrate_start <= '1';

                        -- Acknowledge the data
                        s_data_ack <= not s_data_ack;

                        -- Output start bit
                        uart_tx <= C_UART_START_VALUE;
                        uart_en <= '1';

                        s_fsm_state <= S_START_BIT;
                    else
                        -- Default output
                        uart_tx <= C_UART_IDLE_VALUE;
                        uart_en <= '0';
                    end if;
                
                -- START_BIT : Sending the start bit
                when S_START_BIT =>
                    if s_top_bit = '1' then
                        -- Data sent MSB first
                        uart_tx                           <= s_data_uart(DATA_BITS-1);
                        s_data_uart(DATA_BITS-1 downto 1) <= s_data_uart(DATA_BITS-2 downto 1);

                        s_cpt_data_bit <= to_unsigned(DATA_BITS-1, C_CPT_DATA_BIT_SIZE);

                        s_fsm_state <= S_DATA_BITS;
                    end if;
                
                -- DATA_BIT : Sending the data bits
                when S_DATA_BITS =>
                    if s_top_bit = '1' then
                        uart_tx <= s_data_uart(DATA_BITS-1);
                        s_data_uart(DATA_BITS-1 downto 1) <= s_data_uart(DATA_BITS-2 downto 1);
                        if s_cpt_data_bit = 0 then
                            if PARITY_EN then
                                s_fsm_state <= S_PARITY_BIT;
                            else
                                s_cpt_data_bit <= to_unsigned(STOP_BITS-1, C_CPT_DATA_BIT_SIZE);

                                s_fsm_state <= S_STOP_BITS;
                            end if;
                        else
                            s_cpt_data_bit <= s_cpt_data_bit - 1;
                            s_fsm_state <= S_DATA_BITS;
                        end if;
                    end if;

                -- PARITY_BIT : Sending the parity bit
                when S_PARITY_BIT =>
                    if s_top_bit = '1' then
                        uart_tx <= s_data_parity;

                        s_cpt_data_bit <= to_unsigned(STOP_BITS-1, C_CPT_DATA_BIT_SIZE);
                        s_fsm_state <= S_STOP_BITS;
                    end if;

                -- STOP_BIT : Sending the stop bit
                when S_STOP_BITS =>
                    if s_top_bit = '1' then
                        uart_tx <= C_UART_STOP_VALUE;
                        if s_cpt_data_bit = 0 then
                            if s_data_valid_state = not s_data_vld then
                                -- Acknowledge the data
                                s_data_ack <= not s_data_ack;

                                -- Output start bit
                                uart_tx <= C_UART_START_VALUE;
                                uart_en <= '1';

                                s_fsm_state <= S_START_BIT;
                            else
                                uart_tx <= C_UART_START_VALUE;
                                uart_en <= '1';
                            end if;
                        else
                            s_cpt_data_bit <= s_cpt_data_bit - 1;
                        end if;
                    end if;
            end case;
        end if; -- Rising_edge
    end process p_fsm_state;

    data_ack <= s_data_ack;

end rtl;
