original_cwd=$(pwd)
cd /root/dev/turtlesim/turtlesim_ws
colcon build --symlink-install --executor sequential
source install/setup.bash
cd "$original_cwd"