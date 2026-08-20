
################################################################
# This is a generated script based on design: IMU_Streaming
#
# Though there are limitations about the generated script,
# the main purpose of this utility is to make learning
# IP Integrator Tcl commands easier.
################################################################

namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}
variable script_folder
set script_folder [_tcl::get_script_folder]

################################################################
# Check if script is running in correct Vivado version.
################################################################
set scripts_vivado_version 2026.1
set current_vivado_version [version -short]

if { [string first $scripts_vivado_version $current_vivado_version] == -1 } {
   puts ""
   if { [string compare $scripts_vivado_version $current_vivado_version] > 0 } {
      catch {common::send_gid_msg -ssname BD::TCL -id 2042 -severity "ERROR" " This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Sourcing the script failed since it was created with a future version of Vivado."}

   } else {
     catch {common::send_gid_msg -ssname BD::TCL -id 2041 -severity "ERROR" "This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Please run the script in Vivado <$scripts_vivado_version> then open the design in Vivado <$current_vivado_version>. Upgrade the design by running \"Tools => Report => Report IP Status...\", then run write_bd_tcl to create an updated script."}

   }

   return 1
}

################################################################
# START
################################################################

# To test this script, run the following commands from Vivado Tcl console:
# source IMU_Streaming_script.tcl

# If there is no project opened, this script will create a
# project, but make sure you do not have an existing project
# <./myproj/project_1.xpr> in the current working folder.

set list_projs [get_projects -quiet]
if { $list_projs eq "" } {
   create_project project_1 myproj -part xc7a35tcpg236-1
   set_property BOARD_PART digilentinc.com:basys3:part0:1.2 [current_project]
}


# CHANGE DESIGN NAME HERE
variable design_name
set design_name IMU_Streaming

# If you do not already have an existing IP Integrator design open,
# you can create a design using the following command:
#    create_bd_design $design_name

# Creating design if needed
set errMsg ""
set nRet 0

set cur_design [current_bd_design -quiet]
set list_cells [get_bd_cells -quiet]

if { ${design_name} eq "" } {
   # USE CASES:
   #    1) Design_name not set

   set errMsg "Please set the variable <design_name> to a non-empty value."
   set nRet 1

} elseif { ${cur_design} ne "" && ${list_cells} eq "" } {
   # USE CASES:
   #    2): Current design opened AND is empty AND names same.
   #    3): Current design opened AND is empty AND names diff; design_name NOT in project.
   #    4): Current design opened AND is empty AND names diff; design_name exists in project.

   if { $cur_design ne $design_name } {
      common::send_gid_msg -ssname BD::TCL -id 2001 -severity "INFO" "Changing value of <design_name> from <$design_name> to <$cur_design> since current design is empty."
      set design_name [get_property NAME $cur_design]
   }
   common::send_gid_msg -ssname BD::TCL -id 2002 -severity "INFO" "Constructing design in IPI design <$cur_design>..."

} elseif { ${cur_design} ne "" && $list_cells ne "" && $cur_design eq $design_name } {
   # USE CASES:
   #    5) Current design opened AND has components AND same names.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 1
} elseif { [get_files -quiet ${design_name}.bd] ne "" } {
   # USE CASES: 
   #    6) Current opened design, has components, but diff names, design_name exists in project.
   #    7) No opened design, design_name exists in project.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 2

} else {
   # USE CASES:
   #    8) No opened design, design_name not in project.
   #    9) Current opened design, has components, but diff names, design_name not in project.

   common::send_gid_msg -ssname BD::TCL -id 2003 -severity "INFO" "Currently there is no design <$design_name> in project, so creating one..."

   create_bd_design $design_name

   common::send_gid_msg -ssname BD::TCL -id 2004 -severity "INFO" "Making design <$design_name> as current_bd_design."
   current_bd_design $design_name

}

common::send_gid_msg -ssname BD::TCL -id 2005 -severity "INFO" "Currently the variable <design_name> is equal to \"$design_name\"."

if { $nRet != 0 } {
   catch {common::send_gid_msg -ssname BD::TCL -id 2006 -severity "ERROR" $errMsg}
   return $nRet
}

set bCheckIPsPassed 1
##################################################################
# CHECK IPs
##################################################################
set bCheckIPs 1
if { $bCheckIPs == 1 } {
   set list_check_ips "\ 
xilinx.com:user:spi_master:1.0\
Shiro:user:axistream_uart:1.0\
xilinx.com:user:spi_controller:1.0\
xilinx.com:user:timestamping_unit:1.0\
Shirp:user:fifo_axis:1.0\
xilinx.com:user:axis_serializer:1.0\
xilinx.com:ip:ila:6.2\
Shiro:user:ce_pulse_gen:1.0\
xilinx.com:ip:proc_sys_reset:5.0\
xilinx.com:ip:xlconstant:1.1\
xilinx.com:user:UART_CONTROLLER_v1_0:1.0\
"

   set list_ips_missing ""
   common::send_gid_msg -ssname BD::TCL -id 2011 -severity "INFO" "Checking if the following IPs exist in the project's IP catalog: $list_check_ips ."

   foreach ip_vlnv $list_check_ips {
      set ip_obj [get_ipdefs -all $ip_vlnv]
      if { $ip_obj eq "" } {
         lappend list_ips_missing $ip_vlnv
      }
   }

   if { $list_ips_missing ne "" } {
      catch {common::send_gid_msg -ssname BD::TCL -id 2012 -severity "ERROR" "The following IPs are not found in the IP Catalog:\n  $list_ips_missing\n\nResolution: Please add the repository containing the IP(s) to the project." }
      set bCheckIPsPassed 0
   }

}

if { $bCheckIPsPassed != 1 } {
  common::send_gid_msg -ssname BD::TCL -id 2023 -severity "WARNING" "Will not continue with creation of design due to the error(s) above."
  return 3
}

##################################################################
# DESIGN PROCs
##################################################################



# Procedure to create entire design; Provide argument to make
# procedure reusable. If parentCell is "", will use root.
proc create_root_design { parentCell } {

  variable script_folder
  variable design_name

  if { $parentCell eq "" } {
     set parentCell [get_bd_cells /]
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj


  # Create interface ports

  # Create ports
  set spi_mosi [ create_bd_port -dir O spi_mosi ]
  set spi_cs_n [ create_bd_port -dir O spi_cs_n ]
  set spi_sclk [ create_bd_port -dir O spi_sclk ]
  set spi_miso [ create_bd_port -dir I spi_miso ]
  set uart_rxd [ create_bd_port -dir I uart_rxd ]
  set uart_txd [ create_bd_port -dir O uart_txd ]
  set IMU_INT [ create_bd_port -dir I IMU_INT ]
  set clk_in [ create_bd_port -dir I -type clk clk_in ]
  set spi_failure [ create_bd_port -dir O spi_failure ]
  set fifo_full [ create_bd_port -dir O fifo_full ]
  set ext_reset_in [ create_bd_port -dir I -type rst ext_reset_in ]
  set setup_done_dbg [ create_bd_port -dir O setup_done_dbg ]

  # Create instance: spi_master_0, and set properties
  set spi_master_0 [ create_bd_cell -type ip -vlnv xilinx.com:user:spi_master:1.0 spi_master_0 ]

  # Create instance: axistream_uart_0, and set properties
  set axistream_uart_0 [ create_bd_cell -type ip -vlnv Shiro:user:axistream_uart:1.0 axistream_uart_0 ]
  set_property CONFIG.BAUDRATE {1152000} $axistream_uart_0


  # Create instance: spi_controller_0, and set properties
  set spi_controller_0 [ create_bd_cell -type ip -vlnv xilinx.com:user:spi_controller:1.0 spi_controller_0 ]
  set_property CONFIG.GENERAL_WAIT_CYCLES {10000000} $spi_controller_0


  # Create instance: timestamping_unit_0, and set properties
  set timestamping_unit_0 [ create_bd_cell -type ip -vlnv xilinx.com:user:timestamping_unit:1.0 timestamping_unit_0 ]

  # Create instance: fifo_axis_0, and set properties
  set fifo_axis_0 [ create_bd_cell -type ip -vlnv Shirp:user:fifo_axis:1.0 fifo_axis_0 ]
  set_property CONFIG.FIFO_DEPTH {3840} $fifo_axis_0


  # Create instance: axis_serializer_0, and set properties
  set axis_serializer_0 [ create_bd_cell -type ip -vlnv xilinx.com:user:axis_serializer:1.0 axis_serializer_0 ]

  # Create instance: ila_0, and set properties
  set ila_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:ila:6.2 ila_0 ]
  set_property -dict [list \
    CONFIG.ALL_PROBE_SAME_MU_CNT {10} \
    CONFIG.C_DATA_DEPTH {65536} \
    CONFIG.C_MONITOR_TYPE {Native} \
    CONFIG.C_NUM_OF_PROBES {5} \
    CONFIG.C_PROBE0_WIDTH {16} \
    CONFIG.C_PROBE1_WIDTH {1} \
    CONFIG.C_PROBE2_WIDTH {1} \
    CONFIG.C_PROBE3_WIDTH {1} \
    CONFIG.C_PROBE4_WIDTH {8} \
  ] $ila_0


  # Create instance: ce_pulse_gen_0, and set properties
  set ce_pulse_gen_0 [ create_bd_cell -type ip -vlnv Shiro:user:ce_pulse_gen:1.0 ce_pulse_gen_0 ]
  set_property CONFIG.TARGET_HZ {1125} $ce_pulse_gen_0


  # Create instance: proc_sys_reset_0, and set properties
  set proc_sys_reset_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0 ]
  set_property -dict [list \
    CONFIG.C_EXT_RST_WIDTH {4} \
    CONFIG.RESET_BOARD_INTERFACE {Custom} \
  ] $proc_sys_reset_0


  # Create instance: xlconstant_0, and set properties
  set xlconstant_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 xlconstant_0 ]

  # Create instance: UART_CONTROLLER_v1_0_0, and set properties
  set UART_CONTROLLER_v1_0_0 [ create_bd_cell -type ip -vlnv xilinx.com:user:UART_CONTROLLER_v1_0:1.0 UART_CONTROLLER_v1_0_0 ]

  # Create interface connections
  connect_bd_intf_net -intf_net UART_CONTROLLER_v1_0_0_m_axis [get_bd_intf_pins UART_CONTROLLER_v1_0_0/m_axis] [get_bd_intf_pins axis_serializer_0/s_axis]
  connect_bd_intf_net -intf_net axis_serializer_0_m_axis [get_bd_intf_pins axis_serializer_0/m_axis] [get_bd_intf_pins axistream_uart_0/S_AXIS_DIN]
  connect_bd_intf_net -intf_net fifo_axis_0_m_axis [get_bd_intf_pins fifo_axis_0/m_axis] [get_bd_intf_pins UART_CONTROLLER_v1_0_0/s_axis_fifo]

  # Create port connections
  connect_bd_net -net axistream_uart_0_uart_txd  [get_bd_pins axistream_uart_0/uart_txd] \
  [get_bd_ports uart_txd]
  connect_bd_net -net ce_pulse_gen_0_ce  [get_bd_pins ce_pulse_gen_0/ce] \
  [get_bd_pins spi_controller_0/IMU_INT]
  connect_bd_net -net clk_0_1  [get_bd_ports clk_in] \
  [get_bd_pins ila_0/clk] \
  [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
  [get_bd_pins axis_serializer_0/clk] \
  [get_bd_pins axistream_uart_0/clk] \
  [get_bd_pins ce_pulse_gen_0/clk] \
  [get_bd_pins fifo_axis_0/clk] \
  [get_bd_pins spi_master_0/clk] \
  [get_bd_pins timestamping_unit_0/clk] \
  [get_bd_pins UART_CONTROLLER_v1_0_0/clk] \
  [get_bd_pins spi_controller_0/clk]
  connect_bd_net -net ext_reset_in_1  [get_bd_ports ext_reset_in] \
  [get_bd_pins proc_sys_reset_0/ext_reset_in]
  connect_bd_net -net fifo_axis_0_full  [get_bd_pins fifo_axis_0/full] \
  [get_bd_ports fifo_full]
  connect_bd_net -net fifo_axis_0_s_axis_tready  [get_bd_pins fifo_axis_0/s_axis_tready] \
  [get_bd_pins ila_0/probe3] \
  [get_bd_pins spi_controller_0/m_axis_fifo_tready]
  connect_bd_net -net rst_n_0_1  [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
  [get_bd_pins axis_serializer_0/rst_n] \
  [get_bd_pins axistream_uart_0/rst_n] \
  [get_bd_pins ce_pulse_gen_0/rst_n] \
  [get_bd_pins fifo_axis_0/rst_n] \
  [get_bd_pins spi_master_0/rst_n] \
  [get_bd_pins timestamping_unit_0/rst_n] \
  [get_bd_pins UART_CONTROLLER_v1_0_0/rst_n] \
  [get_bd_pins spi_controller_0/rst_n]
  connect_bd_net -net spi_controller_0_m_axis_fifo_tdata  [get_bd_pins spi_controller_0/m_axis_fifo_tdata] \
  [get_bd_pins ila_0/probe0] \
  [get_bd_pins fifo_axis_0/s_axis_tdata]
  connect_bd_net -net spi_controller_0_m_axis_fifo_tlast  [get_bd_pins spi_controller_0/m_axis_fifo_tlast] \
  [get_bd_pins ila_0/probe1] \
  [get_bd_pins fifo_axis_0/s_axis_tlast]
  connect_bd_net -net spi_controller_0_m_axis_fifo_tvalid  [get_bd_pins spi_controller_0/m_axis_fifo_tvalid] \
  [get_bd_pins ila_0/probe2] \
  [get_bd_pins fifo_axis_0/s_axis_tvalid]
  connect_bd_net -net spi_controller_0_m_axis_spi_tdata  [get_bd_pins spi_controller_0/m_axis_spi_tdata] \
  [get_bd_pins spi_master_0/s_axis_tdata]
  connect_bd_net -net spi_controller_0_m_axis_spi_tlast  [get_bd_pins spi_controller_0/m_axis_spi_tlast] \
  [get_bd_pins spi_master_0/s_axis_tlast]
  connect_bd_net -net spi_controller_0_m_axis_spi_tvalid  [get_bd_pins spi_controller_0/m_axis_spi_tvalid] \
  [get_bd_pins spi_master_0/s_axis_tvalid]
  connect_bd_net -net spi_controller_0_setup_done_dbg  [get_bd_pins spi_controller_0/setup_done_dbg] \
  [get_bd_ports setup_done_dbg]
  connect_bd_net -net spi_controller_0_spi_failure  [get_bd_pins spi_controller_0/spi_failure] \
  [get_bd_ports spi_failure]
  connect_bd_net -net spi_master_0_read_byte  [get_bd_pins spi_master_0/read_byte] \
  [get_bd_pins ila_0/probe4] \
  [get_bd_pins spi_controller_0/read_byte]
  connect_bd_net -net spi_master_0_s_axis_tready  [get_bd_pins spi_master_0/s_axis_tready] \
  [get_bd_pins spi_controller_0/m_axis_spi_tready]
  connect_bd_net -net spi_master_0_spi_cs_n  [get_bd_pins spi_master_0/spi_cs_n] \
  [get_bd_ports spi_cs_n]
  connect_bd_net -net spi_master_0_spi_mosi  [get_bd_pins spi_master_0/spi_mosi] \
  [get_bd_ports spi_mosi]
  connect_bd_net -net spi_master_0_spi_sclk  [get_bd_pins spi_master_0/spi_sclk] \
  [get_bd_ports spi_sclk]
  connect_bd_net -net spi_miso_0_1  [get_bd_ports spi_miso] \
  [get_bd_pins spi_master_0/spi_miso]
  connect_bd_net -net timestamping_unit_0_time_stamp  [get_bd_pins timestamping_unit_0/time_stamp] \
  [get_bd_pins spi_controller_0/timestamp]
  connect_bd_net -net uart_rxd_0_1  [get_bd_ports uart_rxd] \
  [get_bd_pins axistream_uart_0/uart_rxd]
  connect_bd_net -net xlconstant_0_dout  [get_bd_pins xlconstant_0/dout] \
  [get_bd_pins proc_sys_reset_0/dcm_locked]

  # Create address segments


  # Restore current instance
  current_bd_instance $oldCurInst

  validate_bd_design
  save_bd_design
}
# End of create_root_design()


##################################################################
# MAIN FLOW
##################################################################

create_root_design ""


