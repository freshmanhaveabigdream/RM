# 启动命令
colcon build  
source install/setup.sh  
ros2 launch golden_miner test.launch.py  

# 修改矿石数
在ROS/src/golden_miner/launch/test.launch.py中修改第17行的数字
