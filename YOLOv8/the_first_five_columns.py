# 保留txt文件的前五列
import os

def process_txt_file(file_path):
    lines_to_keep = []
    with open(file_path, 'r') as file:
        for line in file:
            columns = line.strip().split()[:5]  # 保留前五列
            new_line = ' '.join(columns) + '\n'
            lines_to_keep.append(new_line)

    with open(file_path, 'w') as file:
        file.writelines(lines_to_keep)

def process_txt_files_in_folder(folder_path):
    for filename in os.listdir(folder_path):
        if filename.endswith('.txt'):
            file_path = os.path.join(folder_path, filename)
            process_txt_file(file_path)

if __name__ == "__main__":
    target_folder = "E:\大符数据\source"  # 下载下来的文件夹路径
    process_txt_files_in_folder(target_folder)
    print("处理完成！")