# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "BAUDRATE" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CLK_FREQ_HZ" -parent ${Page_0}
  ipgui::add_param $IPINST -name "EIGHT_BIT" -parent ${Page_0}
  ipgui::add_param $IPINST -name "OVERSAMPLE" -parent ${Page_0}
  ipgui::add_param $IPINST -name "PARITY_BIT" -parent ${Page_0}
  ipgui::add_param $IPINST -name "TWO_STOP_BITS" -parent ${Page_0}


}

proc update_PARAM_VALUE.BAUDRATE { PARAM_VALUE.BAUDRATE } {
	# Procedure called to update BAUDRATE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BAUDRATE { PARAM_VALUE.BAUDRATE } {
	# Procedure called to validate BAUDRATE
	return true
}

proc update_PARAM_VALUE.CLK_FREQ_HZ { PARAM_VALUE.CLK_FREQ_HZ } {
	# Procedure called to update CLK_FREQ_HZ when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CLK_FREQ_HZ { PARAM_VALUE.CLK_FREQ_HZ } {
	# Procedure called to validate CLK_FREQ_HZ
	return true
}

proc update_PARAM_VALUE.EIGHT_BIT { PARAM_VALUE.EIGHT_BIT } {
	# Procedure called to update EIGHT_BIT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.EIGHT_BIT { PARAM_VALUE.EIGHT_BIT } {
	# Procedure called to validate EIGHT_BIT
	return true
}

proc update_PARAM_VALUE.OVERSAMPLE { PARAM_VALUE.OVERSAMPLE } {
	# Procedure called to update OVERSAMPLE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.OVERSAMPLE { PARAM_VALUE.OVERSAMPLE } {
	# Procedure called to validate OVERSAMPLE
	return true
}

proc update_PARAM_VALUE.PARITY_BIT { PARAM_VALUE.PARITY_BIT } {
	# Procedure called to update PARITY_BIT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.PARITY_BIT { PARAM_VALUE.PARITY_BIT } {
	# Procedure called to validate PARITY_BIT
	return true
}

proc update_PARAM_VALUE.TWO_STOP_BITS { PARAM_VALUE.TWO_STOP_BITS } {
	# Procedure called to update TWO_STOP_BITS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.TWO_STOP_BITS { PARAM_VALUE.TWO_STOP_BITS } {
	# Procedure called to validate TWO_STOP_BITS
	return true
}


proc update_MODELPARAM_VALUE.BAUDRATE { MODELPARAM_VALUE.BAUDRATE PARAM_VALUE.BAUDRATE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BAUDRATE}] ${MODELPARAM_VALUE.BAUDRATE}
}

proc update_MODELPARAM_VALUE.PARITY_BIT { MODELPARAM_VALUE.PARITY_BIT PARAM_VALUE.PARITY_BIT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PARITY_BIT}] ${MODELPARAM_VALUE.PARITY_BIT}
}

proc update_MODELPARAM_VALUE.EIGHT_BIT { MODELPARAM_VALUE.EIGHT_BIT PARAM_VALUE.EIGHT_BIT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.EIGHT_BIT}] ${MODELPARAM_VALUE.EIGHT_BIT}
}

proc update_MODELPARAM_VALUE.TWO_STOP_BITS { MODELPARAM_VALUE.TWO_STOP_BITS PARAM_VALUE.TWO_STOP_BITS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.TWO_STOP_BITS}] ${MODELPARAM_VALUE.TWO_STOP_BITS}
}

proc update_MODELPARAM_VALUE.OVERSAMPLE { MODELPARAM_VALUE.OVERSAMPLE PARAM_VALUE.OVERSAMPLE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OVERSAMPLE}] ${MODELPARAM_VALUE.OVERSAMPLE}
}

proc update_MODELPARAM_VALUE.CLK_FREQ_HZ { MODELPARAM_VALUE.CLK_FREQ_HZ PARAM_VALUE.CLK_FREQ_HZ } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CLK_FREQ_HZ}] ${MODELPARAM_VALUE.CLK_FREQ_HZ}
}

