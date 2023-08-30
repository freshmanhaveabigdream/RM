from launch import LaunchDescription
from launch_ros.actions import Node
import os

current_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))) # 获取工作空间的绝对路径
rviz_dir = os.path.join(current_dir, "src/golden_miner/src/path_and_ball.rviz")   # 定位到预设rivz文件
gold_mine_dir = os.path.join(current_dir, "src/golden_miner/config/mine_number.yaml")
def generate_launch_description():
    return LaunchDescription([
        Node(   # 启动矿界节点
            package='golden_miner',
            executable='gold_mine',
            name='gold_mine',
            parameters=[gold_mine_dir] # 设置初始矿石数
        ),
        Node(   # 启动可视化节点
            package='golden_miner',
            executable='visualization',
            name='visualization'
        ),
        Node(   # 启动rviz2
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", rviz_dir]
        ),
        Node(   # 启动采矿人节点
            package='golden_miner',
            executable='miner',
            name='miner'
        ),
    ])
