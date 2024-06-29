source /opt/ros/humble/setup.bash
rm -r install/ build/
colcon build  --packages-select controller_msgs
colcon build  --packages-select time_sync