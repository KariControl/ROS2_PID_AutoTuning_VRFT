source /opt/ros/humble/setup.bash
. install/setup.bash
gnome-terminal -- ros2 run plotjuggler plotjuggler
sleep 10s
gnome-terminal -- ros2 launch vehicle_sim vehicle_run.py
gnome-terminal -- ros2 launch velocity_control control_run.py
ros2 bag record /accel /vehicle_velocity