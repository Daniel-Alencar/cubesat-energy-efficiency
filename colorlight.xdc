set_property -dict {PACKAGE_PIN K4 IOSTANDARD LVCMOS33} [get_ports dev_clk]; 

## UART
# IO_L16P_T2_35 - P5_15
set_property -dict {PACKAGE_PIN M3 IOSTANDARD LVCMOS33} [get_ports rx]; 
# IO_L3P_T0_DQS_34 - P5_16
set_property -dict {PACKAGE_PIN R3 IOSTANDARD LVCMOS33} [get_ports tx]; 

## Header P2
#IO_0_34 - P2_3
set_property -dict {PACKAGE_PIN T3 IOSTANDARD LVCMOS33} [get_ports SDA]; 
#IO_L2P_T0_34 - P2_4
set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports SCL]; 