# 修改内容
1、在ultralytics/ultralytics/cfg/datasets文件夹下打开coco-pose.yaml，修改11行path、12行train、13行val、17行kpt_shape、21行names，注释掉18行flip_idx  
path:/root/dataset_  
train:images/train  
val:images/val  
kpt_shape: [5, 2]  
names: ['blue_r', 'blue_not_hitted', 'blue_hitted', 'red_r', 'red_not_hitted', 'red_hitted']  
2.在ultralytics/ultralytics/cfg/models/v8下打开yolov8-pose.yaml，修改第五行nc为6，修改第6行kpt_shape为[5, 2]  
# 训练命令：  
yolo task=pose mode=train model=yolov8s-pose.yaml data=coco-pose.yaml epochs=50 batch=16 imgsz=640  
# 预测命令：  
yolo task=pose mode=predict model=ultralytics/runs/pose/train25/weights/best.pt source=test_video.mp4