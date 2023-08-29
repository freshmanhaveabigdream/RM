# 运行命令（在tf-urdf目录下）
## 打开一个终端，进入tf-urdf目录  
- colcon build  
- source install/setup.sh  
- ros2 launch hero_description view_model.lauch.py  
## 再开启一个终端，进入tf-urdf目录  
- source install/setup.sh  
- ros2 run hero_description FrameListener  