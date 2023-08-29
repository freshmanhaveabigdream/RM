from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

current_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))) # 获取工作空间的绝对路径
dir = os.path.join(current_dir, "src/golden_miner/src/test.rviz")   # 定位到预设rivz文件
print(current_dir)
def generate_launch_description():
    return LaunchDescription([
        Node(   # 启动矿界节点
            package='golden_miner',
            executable='gold_mine',
            name='gold_mine',
            parameters=[{
                'ore_number' : 7   # 设置初始矿石数
            }]
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
            arguments=["-d", dir]
        ),
        Node(   # 启动采矿人节点
            package='golden_miner',
            executable='miner',
            name='miner'
        ),
    ])
