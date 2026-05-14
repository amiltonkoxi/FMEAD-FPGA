## ============================================================
## Nexys 4 DDR constraints for mb_edge_wrapper
## Project: FMEAD-FPGA
## FPGA Multi-Modal Edge Anomaly Detector
## ============================================================


## ============================================================
## Clock: 100 MHz
## ============================================================

set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { sys_clock }]
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 5.000} -add [get_ports { sys_clock }]


## ============================================================
## Reset button
## CPU_RESETN on Nexys 4 DDR is C12.
## In this design the wrapper port is named reset.
## ============================================================

set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 } [get_ports { reset }]


## ============================================================
## USB UART
##
## usb_uart_rxd = FPGA receives from USB UART TXD
## usb_uart_txd = FPGA transmits to USB UART RXD
## ============================================================

set_property -dict { PACKAGE_PIN C4 IOSTANDARD LVCMOS33 } [get_ports { usb_uart_rxd }]
set_property -dict { PACKAGE_PIN D4 IOSTANDARD LVCMOS33 } [get_ports { usb_uart_txd }]


## ============================================================
## Switches SW0 - SW15
## Wrapper port: switches_16bits_tri_i[15:0]
## ============================================================

set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[0] }]
set_property -dict { PACKAGE_PIN L16 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[1] }]
set_property -dict { PACKAGE_PIN M13 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[2] }]
set_property -dict { PACKAGE_PIN R15 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[3] }]
set_property -dict { PACKAGE_PIN R17 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[4] }]
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[5] }]
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[6] }]
set_property -dict { PACKAGE_PIN R13 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[7] }]
set_property -dict { PACKAGE_PIN T8  IOSTANDARD LVCMOS18 } [get_ports { switches_16bits_tri_i[8] }]
set_property -dict { PACKAGE_PIN U8  IOSTANDARD LVCMOS18 } [get_ports { switches_16bits_tri_i[9] }]
set_property -dict { PACKAGE_PIN R16 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[10] }]
set_property -dict { PACKAGE_PIN T13 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[11] }]
set_property -dict { PACKAGE_PIN H6  IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[12] }]
set_property -dict { PACKAGE_PIN U12 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[13] }]
set_property -dict { PACKAGE_PIN U11 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[14] }]
set_property -dict { PACKAGE_PIN V10 IOSTANDARD LVCMOS33 } [get_ports { switches_16bits_tri_i[15] }]


## ============================================================
## LEDs LD0 - LD15
## Wrapper port: leds_16bits_tri_io[15:0]
## ============================================================

set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[0] }]
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[1] }]
set_property -dict { PACKAGE_PIN J13 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[2] }]
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[3] }]
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[4] }]
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[5] }]
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[6] }]
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[7] }]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[8] }]
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[9] }]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[10] }]
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[11] }]
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[12] }]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[13] }]
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[14] }]
set_property -dict { PACKAGE_PIN V11 IOSTANDARD LVCMOS33 } [get_ports { leds_16bits_tri_io[15] }]


## ============================================================
## Push buttons
## Wrapper port: buttons_5bits_tri_i[4:0]
##
## [0] = BTNC
## [1] = BTNU
## [2] = BTNL
## [3] = BTNR
## [4] = BTND
## ============================================================

set_property -dict { PACKAGE_PIN N17 IOSTANDARD LVCMOS33 } [get_ports { buttons_5bits_tri_i[0] }]
set_property -dict { PACKAGE_PIN M18 IOSTANDARD LVCMOS33 } [get_ports { buttons_5bits_tri_i[1] }]
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33 } [get_ports { buttons_5bits_tri_i[2] }]
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS33 } [get_ports { buttons_5bits_tri_i[3] }]
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports { buttons_5bits_tri_i[4] }]


## ============================================================
## ADXL362 onboard accelerometer
## Wrapper ports: acl_spi_*
##
## IMPORTANT:
## AXI Quad SPI:
##   io0 = MOSI  -> F14
##   io1 = MISO  -> E15
##   sck = SCLK  -> F15
##   ss  = CS    -> D15
##
## This fixes the previous ADXL ID = 0x00 problem.
## ============================================================

set_property -dict { PACKAGE_PIN F14 IOSTANDARD LVCMOS33 } [get_ports { acl_spi_io0_io }]
set_property -dict { PACKAGE_PIN E15 IOSTANDARD LVCMOS33 } [get_ports { acl_spi_io1_io }]
set_property -dict { PACKAGE_PIN F15 IOSTANDARD LVCMOS33 } [get_ports { acl_spi_sck_io }]
set_property -dict { PACKAGE_PIN D15 IOSTANDARD LVCMOS33 } [get_ports { acl_spi_ss_io }]


## ============================================================
## JA: BME280 I2C
## Wrapper ports:
##   bme280_iic_scl_io
##   bme280_iic_sda_io
##
## Current working mapping:
##   SCL -> G17
##   SDA -> E18
##
## Pull-ups enabled because I2C needs pull-up lines.
## ============================================================

set_property -dict { PACKAGE_PIN G17 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { bme280_iic_scl_io }]
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { bme280_iic_sda_io }]


## ============================================================
## JB: Pmod OLED RGB
## Wrapper ports: jb_pin*_io
## ============================================================

set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS33 } [get_ports { jb_pin1_io }]
set_property -dict { PACKAGE_PIN F16 IOSTANDARD LVCMOS33 } [get_ports { jb_pin2_io }]
set_property -dict { PACKAGE_PIN G16 IOSTANDARD LVCMOS33 } [get_ports { jb_pin3_io }]
set_property -dict { PACKAGE_PIN H14 IOSTANDARD LVCMOS33 } [get_ports { jb_pin4_io }]
set_property -dict { PACKAGE_PIN E16 IOSTANDARD LVCMOS33 } [get_ports { jb_pin7_io }]
set_property -dict { PACKAGE_PIN F13 IOSTANDARD LVCMOS33 } [get_ports { jb_pin8_io }]
set_property -dict { PACKAGE_PIN G13 IOSTANDARD LVCMOS33 } [get_ports { jb_pin9_io }]
set_property -dict { PACKAGE_PIN H16 IOSTANDARD LVCMOS33 } [get_ports { jb_pin10_io }]


## ============================================================
## JC: Pmod ESP32
## Wrapper ports: jc_pin*_io
## ============================================================

set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { jc_pin1_io }]
set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 } [get_ports { jc_pin2_io }]
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports { jc_pin3_io }]
set_property -dict { PACKAGE_PIN G6 IOSTANDARD LVCMOS33 } [get_ports { jc_pin4_io }]
set_property -dict { PACKAGE_PIN E7 IOSTANDARD LVCMOS33 } [get_ports { jc_pin7_io }]
set_property -dict { PACKAGE_PIN J3 IOSTANDARD LVCMOS33 } [get_ports { jc_pin8_io }]
set_property -dict { PACKAGE_PIN J4 IOSTANDARD LVCMOS33 } [get_ports { jc_pin9_io }]
set_property -dict { PACKAGE_PIN E6 IOSTANDARD LVCMOS33 } [get_ports { jc_pin10_io }]


## ============================================================
## JD: Pmod BT2 Bluetooth
## Wrapper ports: jd_pin*_io
##
## PmodBT2 / RN42 pin meaning:
##   JD1  = RTS
##   JD2  = RXD, RN42 input, FPGA TX
##   JD3  = TXD, RN42 output, FPGA RX
##   JD4  = CTS
##   JD7  = STATUS
##   JD8  = RST, active low
##   JD9  = NC
##   JD10 = NC
##
## Keep this block for the current PmodBT2 IP design.
## For direct UARTLite test later, use separate ports:
##   bt_uart_txd -> H1
##   bt_uart_rxd -> G1
##   bt_rst_n    -> G4
## Do not use both mappings at the same time.
## ============================================================

set_property -dict { PACKAGE_PIN H4 IOSTANDARD LVCMOS33 } [get_ports { jd_pin1_io }]
set_property -dict { PACKAGE_PIN H1 IOSTANDARD LVCMOS33 } [get_ports { jd_pin2_io }]
set_property -dict { PACKAGE_PIN G1 IOSTANDARD LVCMOS33 } [get_ports { jd_pin3_io }]
set_property -dict { PACKAGE_PIN G3 IOSTANDARD LVCMOS33 } [get_ports { jd_pin4_io }]
set_property -dict { PACKAGE_PIN H2 IOSTANDARD LVCMOS33 } [get_ports { jd_pin7_io }]
set_property -dict { PACKAGE_PIN G4 IOSTANDARD LVCMOS33 } [get_ports { jd_pin8_io }]
set_property -dict { PACKAGE_PIN G2 IOSTANDARD LVCMOS33 } [get_ports { jd_pin9_io }]
set_property -dict { PACKAGE_PIN F3 IOSTANDARD LVCMOS33 } [get_ports { jd_pin10_io }]


## ============================================================
## 7-segment display
## Nexys 4 DDR
##
## Common-anode, multiplexed, active LOW.
##
## Wrapper ports:
##   sevenseg_seg[7:0]
##   sevenseg_an[7:0]
##
## Segment mapping:
##   sevenseg_seg[0] = CA
##   sevenseg_seg[1] = CB
##   sevenseg_seg[2] = CC
##   sevenseg_seg[3] = CD
##   sevenseg_seg[4] = CE
##   sevenseg_seg[5] = CF
##   sevenseg_seg[6] = CG
##   sevenseg_seg[7] = DP
##
## Anode mapping:
##   sevenseg_an[0] = AN0
##   sevenseg_an[1] = AN1
##   sevenseg_an[2] = AN2
##   sevenseg_an[3] = AN3
##   sevenseg_an[4] = AN4
##   sevenseg_an[5] = AN5
##   sevenseg_an[6] = AN6
##   sevenseg_an[7] = AN7
## ============================================================

## Segments CA, CB, CC, CD, CE, CF, CG, DP
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[0] }]
set_property -dict { PACKAGE_PIN R10 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[1] }]
set_property -dict { PACKAGE_PIN K16 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[2] }]
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[3] }]
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[4] }]
set_property -dict { PACKAGE_PIN T11 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[5] }]
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[6] }]
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_seg[7] }]

## Anodes AN0..AN7
set_property -dict { PACKAGE_PIN J17 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[0] }]
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[1] }]
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[2] }]
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[3] }]
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[4] }]
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[5] }]
set_property -dict { PACKAGE_PIN K2  IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[6] }]
set_property -dict { PACKAGE_PIN U13 IOSTANDARD LVCMOS33 } [get_ports { sevenseg_an[7] }]


## ============================================================
## Reserved for future: ADXL362 interrupts
##
## Do not enable until these ports exist in the wrapper.
## ============================================================

# set_property -dict { PACKAGE_PIN B13 IOSTANDARD LVCMOS33 } [get_ports { adxl_int1 }]
# set_property -dict { PACKAGE_PIN C16 IOSTANDARD LVCMOS33 } [get_ports { adxl_int2 }]


## ============================================================
## Reserved for future: onboard PDM microphone
##
## Do not enable until these ports exist in the wrapper.
## ============================================================

# set_property -dict { PACKAGE_PIN J5 IOSTANDARD LVCMOS33 } [get_ports { mic_clk }]
# set_property -dict { PACKAGE_PIN H5 IOSTANDARD LVCMOS33 } [get_ports { mic_data }]
# set_property -dict { PACKAGE_PIN F5 IOSTANDARD LVCMOS33 } [get_ports { mic_lrsel }]