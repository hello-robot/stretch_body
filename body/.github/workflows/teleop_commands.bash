cd repos/stretch_body;
declare -a TestFiles=("test_arm" "test_dxl_comms" "test_end_of_arm" "test_hello_utils" "test_robot_params" "test_timing_stats" "test_base" "test_dynamixel_hello_XL430" "test_end_of_arm_tools" "test_lift" "test_robot" "test_timing_stats" "test_arm" "test_device" "test_dynamixel_XL430" "test_hello_utils" "test_pimu" "test_steppers" )
 
# Iterate the string array using for loop
for fileName in ${TestFiles[@]}; do
   echo $fileName
done