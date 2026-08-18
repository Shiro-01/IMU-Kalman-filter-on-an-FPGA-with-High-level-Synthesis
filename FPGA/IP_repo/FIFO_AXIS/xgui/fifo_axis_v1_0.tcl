# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  ipgui::add_page $IPINST -name "Page 0"

  set FIFO_DEPTH [ipgui::add_param $IPINST -name "FIFO_DEPTH"]
  set_property tooltip {Fifp Depth in words, >=2} ${FIFO_DEPTH}
  set WORD_WIDTH [ipgui::add_param $IPINST -name "WORD_WIDTH"]
  set_property tooltip {Word Width, must be multiple of 8} ${WORD_WIDTH}

}

proc update_PARAM_VALUE.FIFO_DEPTH { PARAM_VALUE.FIFO_DEPTH } {
	# Procedure called to update FIFO_DEPTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.FIFO_DEPTH { PARAM_VALUE.FIFO_DEPTH } {
	# Procedure called to validate FIFO_DEPTH
	return true
}

proc update_PARAM_VALUE.WORD_WIDTH { PARAM_VALUE.WORD_WIDTH } {
	# Procedure called to update WORD_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.WORD_WIDTH { PARAM_VALUE.WORD_WIDTH } {
	# Procedure called to validate WORD_WIDTH
	return true
}


proc update_MODELPARAM_VALUE.FIFO_DEPTH { MODELPARAM_VALUE.FIFO_DEPTH PARAM_VALUE.FIFO_DEPTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.FIFO_DEPTH}] ${MODELPARAM_VALUE.FIFO_DEPTH}
}

proc update_MODELPARAM_VALUE.WORD_WIDTH { MODELPARAM_VALUE.WORD_WIDTH PARAM_VALUE.WORD_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.WORD_WIDTH}] ${MODELPARAM_VALUE.WORD_WIDTH}
}

