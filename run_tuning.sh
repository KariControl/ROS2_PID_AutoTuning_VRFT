source /opt/ros/humble/setup.bash
. install/setup.bash
# gnome-terminal -- ros2 run plotjuggler plotjuggler
# sleep 10s
gnome-terminal -- ros2 launch time_sync time_sync.py
sleep 1s
gnome-terminal -- ros2 bag play rosbag2_2024_06_30-08_13_17/
ros2 launch pid_tuner pid_tuner.py
sleep 1s