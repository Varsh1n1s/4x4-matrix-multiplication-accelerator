## top_matrix_uart.xdc
## Arty A7-100T pin constraints for the matrix multiplication accelerator.
## Clock: 100 MHz  |  UART: 115200 8N1

## ─── Clock ──────────────────────────────────────────────────────────────────
set_property -dict { PACKAGE_PIN E3  IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## ─── Reset (BTN0 – active low) ───────────────────────────────────────────────
set_property -dict { PACKAGE_PIN C2  IOSTANDARD LVCMOS33 } [get_ports rst_n]

## ─── UART (USB-UART via FT2232HQ) ───────────────────────────────────────────
set_property -dict { PACKAGE_PIN A9  IOSTANDARD LVCMOS33 } [get_ports uart_rx]
set_property -dict { PACKAGE_PIN D10 IOSTANDARD LVCMOS33 } [get_ports uart_tx]

## ─── LEDs LD0–LD3 (only green channel used for plain on/off) ────────────────
## LD0 green = H5
set_property -dict { PACKAGE_PIN H5  IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
## LD1 green = J5
set_property -dict { PACKAGE_PIN J5  IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
## LD2 green = T9
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
## LD3 green = T10
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]

## ─── LEDs LD4–LD7 (plain, for result preview bits) ──────────────────────────
set_property -dict { PACKAGE_PIN G6  IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN F6  IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN E1  IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN G3  IOSTANDARD LVCMOS33 } [get_ports {led[7]}]
