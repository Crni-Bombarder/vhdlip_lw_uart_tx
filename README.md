# Small size VHDL UART Transmiter IP

This IP implement a standalone UART Transmitter with a fixed configuration in order to reduce the size of the design.

## Generics

| Name        | Type     | Default Value | Description                           |
| ----------- | -------- | ------------- | ------------------------------------- |
| CLK_FREQ    | positive | None          | Input clock frequency in Hertz        |
| BAUDRATE    | positive | 9600          | UART baudrate                         |
| DATA_BITS   | integer  | 8             | Number of data bits, between 5 to 9   |
| STOP_BITS   | integer  | 2             | Number of stop bit, either 1 or 2     |
| PARITY_EN   | boolean  | true          | Enable the parity bit                 |
| PARITY_TYPE | boolean  | false         | Parity type, true : odd, false : even |
| INPUT_CDC   | boolean  | false         | CDC interface enable                  |

---
## Ports

| Name     | Direction | Type                                   | Description                              |
| -------- | --------- | -------------------------------------- | ---------------------------------------- |
| clk      | in        | std_logic                              | Input clock                              |
| rst_n_a  | in        | std_logic                              | Asynchronous active low reset            |
| data_in  | in        | std_logic_vector(DATA_BITS-1 downto 0) | Input data                               |
| data_vld | in        | std_logic                              | Input data valid                         |
| data_ack | out       | std_logic                              | Acknowledge the input data               |
| uart_tx  | out       | std_logic                              | UART Tx output                           |
| uart_en  | out       | std_logic                              | Asserted when the UART interface is used |

---
## Operation

To communicate with this module : 
1. After reset, `data_ack` is low and `data_vld` need to be low.
2. Put the data to be sent in `data_in` and toggle `data_vld`. (Data sent **MSB** first)
3. When `data_ack` change state, the data is currently beeing processed in the module.
   * If the next data is ready, do as step *2.* put the data in `data_in` and toggle `data_vld`.
   * Else, wait until the next data is ready. Then go back to step *2.*

## Clock Domain Change (CDC) requirement

The only condition for a CDC at this module's interface is the following :
$$
    f_{int} > \frac{f_{mod}}{(DATA\_BITS + PARITY\_BIT + STOP\_BITS - 1)}
$$

With :
  * $f_{int}$ the frequency of the interface
  * $f_{mod}$ the frequency of the module

See this [document](CDC_setup.md) for more informations about how to setup the module in case of a CDC.
