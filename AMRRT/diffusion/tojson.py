import os
import glob
import csv
import json

# 指定要搜索的文件夹路径
folder_path = 'index_to_pos'  # 替换成你的文件夹路径

# 使用glob.glob()来获取匹配的文件名
csv_files = glob.glob(os.path.join(folder_path, '*.csv'))

# 打印所有匹配的CSV文件名
for file_name in csv_files:
    # 打开CSV文件
    with open(file_name, mode='r') as file:
        reader = csv.reader(file)  # 创建CSV读取器
        first_row = next(reader)
        print(file_name,first_row)
        data = [[int(float(D)) for D in row[:-1]] for row in reader]
        json_data = json.dumps(data)
        with open(file_name[:-3]+'json', mode='w') as json_file:
            json_file.write(json_data)
