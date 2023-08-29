# 将文件夹转为yolov8训练格式
import os
import random
import shutil

# 定义所有下载下来的文件保存的文件夹路径和目标文件夹路径
input_folder = "E:\\大符数据\\source"
images_folder = "dataset_\\images"
labels_folder = "dataset_\\labels"

if not os.path.exists(images_folder):
    os.makedirs(os.path.join(images_folder, "train"))
    os.makedirs(os.path.join(images_folder, "val"))

if not os.path.exists(labels_folder):
    os.makedirs(os.path.join(labels_folder, "train"))
    os.makedirs(os.path.join(labels_folder, "val"))

# 获取文件夹中所有的jpg文件名（不包括扩展名）
jpg_filenames = [filename.split(".")[0] for filename in os.listdir(input_folder) if filename.lower().endswith(".jpg")]

# 计算80%和20%的文件数量
total_files = len(jpg_filenames)
num_train_files = int(0.8 * total_files)
num_val_files = total_files - num_train_files

# 随机选择80%的文件作为train文件
selected_files = random.sample(jpg_filenames, num_train_files)

# 将文件分配到images的train和val、labels的train和val文件夹中
for filename in jpg_filenames:
    src_jpg_path = os.path.join(input_folder, f"{filename}.jpg")
    src_txt_path = os.path.join(input_folder, f"{filename}.txt")

    if filename in selected_files:
        dst_jpg_path = os.path.join(images_folder, "train", f"{filename}.jpg")
        dst_txt_path = os.path.join(labels_folder, "train", f"{filename}.txt")
    else:
        dst_jpg_path = os.path.join(images_folder, "val", f"{filename}.jpg")
        dst_txt_path = os.path.join(labels_folder, "val", f"{filename}.txt")

    shutil.copy(src_jpg_path, dst_jpg_path)
    shutil.copy(src_txt_path, dst_txt_path)

print("文件分配完成！")