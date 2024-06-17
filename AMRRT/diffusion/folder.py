import os
import glob

# 指定要搜索的文件夹路径
folder_path = 'diffusion_matrix'  # 替换成你的文件夹路径

# 使用glob.glob()来获取匹配的文件名
csv_files = glob.glob(os.path.join(folder_path, '*.csv'))

# 打印所有匹配的CSV文件名
for file in csv_files:
    print(type(file),file)
