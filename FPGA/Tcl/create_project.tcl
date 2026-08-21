# Recreate Vivado project + BD + custom IP repo for the IMU Kalman Filter FPGA project

# Anchor Vivado's working directory to this script's own location, so
set script_dir [file normalize [file dirname [info script]]]
cd $script_dir

set proj_name "imu_streaming"
set proj_dir  [file normalize "../../../build/$proj_name"]
set part_name "xc7a35tcpg236-1"  ;# Basys 3

create_project $proj_name $proj_dir -part $part_name -force

cd $proj_dir

# Add custom IP repository
# script_dir = <repo>/FPGA/Tcl, so ../IP_repo = <repo>/FPGA/IP_repo
set ip_repo_path [file normalize "$script_dir/../IP_repo"]
set_property ip_repo_paths [list $ip_repo_path] [current_project]
update_ip_catalog

# add constraints (Digilent master XDC + project-specific pin constraints)
add_files -fileset constrs_1 -norecurse [list \
  [file normalize "$script_dir/../Constrain_files/Basys-3-Master.xdc"] \
  [file normalize "$script_dir/../Constrain_files/streaming.xdc"] \
]

# setting the target -- streaming.xdc holds the active project constraints
set_property target_constrs_file [get_files streaming.xdc] [current_fileset -constrset]

# adding other files
#add_files -norecurse [glob -nocomplain ./src/*]

# Recreate block design
source [file normalize "$script_dir/../block_designs/IMU_Streaming.tcl"]

# --- Safety check: fail early if no BD exists ---
set bds [get_files *.bd]
if {[llength $bds] == 0} {
  error "No BD found after sourcing IMU_Streaming.tcl. Check that $script_dir/../block_designs/IMU_Streaming.tcl really creates the BD and that IP repos are set correctly."
}
set bd_file [lindex $bds 0]

open_bd_design $bd_file
validate_bd_design
save_bd_design

# Create wrapper and add it
set wrapper [make_wrapper -files $bd_file -top]
add_files -norecurse $wrapper
update_compile_order -fileset sources_1

# Generate IP output products
generate_target all $bd_file

puts "DONE. Open: $proj_dir/$proj_name.xpr"