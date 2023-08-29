# 运行命令（在GAZEBO目录下）
colcon build  
source install/setup.sh  
ros2 launch find_box test.launch.py  

# 切换目标方块
ros2 topic pub /color std_msgs/msg/Int32 'data: 0'
- data值共四种0、1、2、3,分别代表四个不同颜色（黄色、紫色、蓝色、墨蓝色）的方块

# 注意
如果重复launch，可能会导致小车回不到初始位置，可以把pose等置0再退出
