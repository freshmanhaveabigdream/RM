# 修改内容
1、在ultralytics/ultralytics/cfg/datasets文件夹下打开coco128.yaml，修改11行path、12行train、13行val、17行names  
path:/root/dataset_  
train:images/train  
val:images/val  
names: ['blue_r', 'blue_not_hitted', 'blue_hitted', 'red_r', 'red_not_hitted', 'red_hitted']  
2.在ultralytics/ultralytics/cfg/models/v8下打开yolov8.yaml，修改第五行nc为6  
3.训练命令：  
yolo task=detect mode=train model=yolov8s.yaml data=coco128.yaml epochs=50 batch=16 imgsz=640  
# 注意事项
在处理数据时，由于yolov8的detect任务数据格式要求严格，所以单独写了一个程序，将图像信息（.txt）中的前五列保留。
