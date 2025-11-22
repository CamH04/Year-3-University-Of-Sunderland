colcon build --symlink-install
source install/setup.bash
ros2 launch my_diffbot bringup.launch.py
