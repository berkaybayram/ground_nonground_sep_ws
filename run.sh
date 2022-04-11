# bin/bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 &&
echo "BUILD COMPLETED..." &&
#echo &&
echo "running..." &&
source ./install/setup.bash &&
ros2 run ground_sep ground_sep_node # ros2 run <package> <executable>
