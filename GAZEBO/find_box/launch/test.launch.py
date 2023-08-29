from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign gazebo',
                 '/home/jjgong/learning/GAZEBO/find_box/src/myrobot.sdf',
            ],
            output='screen',
            shell=True
        ),
        # 创建ros与ign的桥接节点
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                       '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                       '/lida@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            output='screen',
            name='message_bridge'
        ),
        # 创建ign和ros之间图像传输的节点
        Node(
            package='ros_ign_image',
            executable='image_bridge',
            arguments=['camera'],
            output='screen',
            name='camera'
        ),
        # 创建可执行文件的节点
        Node(
            package='find_box',
            executable='find_box',
            output='screen',
            name='find_box'
        )
    ])
