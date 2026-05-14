## ============================================================
## Nexys 4 DDR onboard PDM microphone
## ADMP421 microphone
##
## mic_clk   -> J5
## mic_data  -> H5
## mic_lrsel -> F5
## ============================================================

set_property -dict { PACKAGE_PIN J5 IOSTANDARD LVCMOS33 } [get_ports { mic_clk }]
set_property -dict { PACKAGE_PIN H5 IOSTANDARD LVCMOS33 } [get_ports { mic_data }]
set_property -dict { PACKAGE_PIN F5 IOSTANDARD LVCMOS33 } [get_ports { mic_lrsel }]

