# 启动命令
colcon build
source install/setup.sh
ros2 launch golden_miner test.launch.py
# 修改矿石数
在ROS/src/golden_miner/config/mine_number.yaml中修改第3行的数字

![图片走丢了](pictures/打印位置和已有矿石信息.png)
![图片走丢了](pictures/topic echo接收矿石信息.png)
