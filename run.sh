# bin/bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install&&
echo "BUILD COMPLETED..." &&
#echo &&
echo "running..." &&
source ./install/setup.bash &&
#ros2 run ground_sep ground_sep_node_exe # ros2 run <package> <executable>
ros2 launch ground_sep launch.py

#rviz2 -d ~/projects/ground_nonground_sep_ws/src/ground_sep/config/rviz2_cfg.rviz 
