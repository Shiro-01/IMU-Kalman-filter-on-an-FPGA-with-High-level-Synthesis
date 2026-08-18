# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "CLK_DIV_HALF" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CS_HOLD_TIME" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SCLK_HIGH_H_TIME" -parent ${Page_0}


}

proc update_PARAM_VALUE.CLK_DIV_HALF { PARAM_VALUE.CLK_DIV_HALF } {
	# Procedure called to update CLK_DIV_HALF when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CLK_DIV_HALF { PARAM_VALUE.CLK_DIV_HALF } {
	# Procedure called to validate CLK_DIV_HALF
	return true
}

proc update_PARAM_VALUE.CS_HOLD_TIME { PARAM_VALUE.CS_HOLD_TIME } {
	# Procedure called to update CS_HOLD_TIME when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CS_HOLD_TIME { PARAM_VALUE.CS_HOLD_TIME } {
	# Procedure called to validate CS_HOLD_TIME
	return true
}

proc update_PARAM_VALUE.SCLK_HIGH_H_TIME { PARAM_VALUE.SCLK_HIGH_H_TIME } {
	# Procedure called to update SCLK_HIGH_H_TIME when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SCLK_HIGH_H_TIME { PARAM_VALUE.SCLK_HIGH_H_TIME } {
	# Procedure called to validate SCLK_HIGH_H_TIME
	return true
}


proc update_MODELPARAM_VALUE.CLK_DIV_HALF { MODELPARAM_VALUE.CLK_DIV_HALF PARAM_VALUE.CLK_DIV_HALF } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CLK_DIV_HALF}] ${MODELPARAM_VALUE.CLK_DIV_HALF}
}

proc update_MODELPARAM_VALUE.SCLK_HIGH_H_TIME { MODELPARAM_VALUE.SCLK_HIGH_H_TIME PARAM_VALUE.SCLK_HIGH_H_TIME } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SCLK_HIGH_H_TIME}] ${MODELPARAM_VALUE.SCLK_HIGH_H_TIME}
}

proc update_MODELPARAM_VALUE.CS_HOLD_TIME { MODELPARAM_VALUE.CS_HOLD_TIME PARAM_VALUE.CS_HOLD_TIME } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CS_HOLD_TIME}] ${MODELPARAM_VALUE.CS_HOLD_TIME}
}

