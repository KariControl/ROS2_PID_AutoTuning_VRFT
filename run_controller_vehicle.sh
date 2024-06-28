source /opt/ros/humble/setup.bash
. install/setup.bash
gnome-terminal -- ros2 run plotjuggler plotjuggler
sleep 30s
gnome-terminal -- ros2 launch vehicle_sim vehicle_run.py
# cd ~/Vehicle_PID_sim
# source /opt/ros/humble/setup.bash
# . install/setup.bash
gnome-terminal -- ros2 launch control_sim control_run.py
ros2 bag record /steering /vehicle_state