# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct D:\FPGA\Labaratory-2026\lab_10\mb_edge_wrapper\platform.tcl
# 
# OR launch xsct and run below command.
# source D:\FPGA\Labaratory-2026\lab_10\mb_edge_wrapper\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {mb_edge_wrapper}\
-hw {D:\FPGA\Labaratory-2026\mb_edge_temp_motion_anomaly_lab13_14\mb_edge_wrapper.xsa}\
-out {D:/FPGA/Labaratory-2026/lab_10}

platform write
domain create -name {standalone_microblaze_0} -display-name {standalone_microblaze_0} -os {standalone} -proc {microblaze_0} -runtime {cpp} -arch {32-bit} -support-app {empty_application}
platform generate -domains 
platform active {mb_edge_wrapper}
platform generate -quick
platform clean
platform clean
platform clean
platform clean
platform clean
platform clean
platform clean
platform clean
platform clean
platform clean
platform clean
platform generate
platform clean
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform config -updatehw {D:/FPGA/Labaratory-2026/mb_edge_temp_motion_anomaly_lab13_14/mb_edge_wrapper.xsa}
platform clean
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform config -updatehw {D:/FPGA/Labaratory-2026/mb_edge_temp_motion_anomaly_lab13_14/mb_edge_wrapper.xsa}
platform generate
platform clean
platform generate
platform config -updatehw {D:/FPGA/Labaratory-2026/mb_edge_temp_motion_anomaly_lab13_14/mb_edge_wrapper.xsa}
platform clean
platform generate
platform clean
platform generate
platform config -updatehw {D:/FPGA/Labaratory-2026/mb_edge_temp_motion_anomaly_lab13_14/mb_edge_wrapper.xsa}
platform clean
platform generate
platform config -updatehw {D:/FPGA/Labaratory-2026/mb_edge_temp_motion_anomaly_lab13_14/mb_edge_wrapper.xsa}
platform generate
platform clean
platform generate
platform clean
platform clean
platform generate
platform clean
platform clean
platform generate
platform clean
platform generate
platform active {mb_edge_wrapper}
platform config -updatehw {D:/FPGA/Labaratory-2026/mb_edge_temp_motion_anomaly_lab13_14/mb_edge_wrapper.xsa}
platform clean
platform generate
platform clean
platform generate
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
platform clean
platform generate
