# 运行命令（在GAZEBO目录下）
colcon build
source install/setup.sh
ros2 launch find_box test.launch.py
# 打开另一个终端（在GAZEBO目录下）
ros2 run turtlesim turtle_teleop_key

# 切换目标方块
ros2 topic pub /color std_msgs/msg/Int32 'data: 0'
- data值共四种0、1、2、3,分别代表四个不同颜色（黄色、紫色、蓝色、墨蓝色）的方块

# 切换操控模式
ros2 param set /find_box is_key 0
- is_key设置为0时代表使用键盘操控，设置为1时代表自动寻找目标方块
