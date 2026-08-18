# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "BYTES_TO_READ" -parent ${Page_0}
  ipgui::add_param $IPINST -name "GENERAL_WAIT_CYCLES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "RESET_WAIT_CYCLES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "TIMESTAMP_WORDS" -parent ${Page_0}


}

proc update_PARAM_VALUE.BYTES_TO_READ { PARAM_VALUE.BYTES_TO_READ } {
	# Procedure called to update BYTES_TO_READ when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BYTES_TO_READ { PARAM_VALUE.BYTES_TO_READ } {
	# Procedure called to validate BYTES_TO_READ
	return true
}

proc update_PARAM_VALUE.GENERAL_WAIT_CYCLES { PARAM_VALUE.GENERAL_WAIT_CYCLES } {
	# Procedure called to update GENERAL_WAIT_CYCLES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.GENERAL_WAIT_CYCLES { PARAM_VALUE.GENERAL_WAIT_CYCLES } {
	# Procedure called to validate GENERAL_WAIT_CYCLES
	return true
}

proc update_PARAM_VALUE.RESET_WAIT_CYCLES { PARAM_VALUE.RESET_WAIT_CYCLES } {
	# Procedure called to update RESET_WAIT_CYCLES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RESET_WAIT_CYCLES { PARAM_VALUE.RESET_WAIT_CYCLES } {
	# Procedure called to validate RESET_WAIT_CYCLES
	return true
}

proc update_PARAM_VALUE.TIMESTAMP_WORDS { PARAM_VALUE.TIMESTAMP_WORDS } {
	# Procedure called to update TIMESTAMP_WORDS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.TIMESTAMP_WORDS { PARAM_VALUE.TIMESTAMP_WORDS } {
	# Procedure called to validate TIMESTAMP_WORDS
	return true
}


proc update_MODELPARAM_VALUE.BYTES_TO_READ { MODELPARAM_VALUE.BYTES_TO_READ PARAM_VALUE.BYTES_TO_READ } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BYTES_TO_READ}] ${MODELPARAM_VALUE.BYTES_TO_READ}
}

proc update_MODELPARAM_VALUE.TIMESTAMP_WORDS { MODELPARAM_VALUE.TIMESTAMP_WORDS PARAM_VALUE.TIMESTAMP_WORDS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.TIMESTAMP_WORDS}] ${MODELPARAM_VALUE.TIMESTAMP_WORDS}
}

proc update_MODELPARAM_VALUE.RESET_WAIT_CYCLES { MODELPARAM_VALUE.RESET_WAIT_CYCLES PARAM_VALUE.RESET_WAIT_CYCLES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RESET_WAIT_CYCLES}] ${MODELPARAM_VALUE.RESET_WAIT_CYCLES}
}

proc update_MODELPARAM_VALUE.GENERAL_WAIT_CYCLES { MODELPARAM_VALUE.GENERAL_WAIT_CYCLES PARAM_VALUE.GENERAL_WAIT_CYCLES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.GENERAL_WAIT_CYCLES}] ${MODELPARAM_VALUE.GENERAL_WAIT_CYCLES}
}

