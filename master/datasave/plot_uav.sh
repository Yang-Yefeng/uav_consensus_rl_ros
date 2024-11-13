#!/bin/bash

# 获取当前工作目录路径
folder_path="$PWD"

# 统计.zip文件个数
zip_count=$(find "$folder_path" -maxdepth 1 -type f -name "*.zip" | wc -l)

# 输出.zip文件个数
# echo "在文件夹 $folder_path 中有 $zip_count 个.zip文件："

# 输出所有.zip文件名称
zip_files=$(find "$folder_path" -maxdepth 1 -type f -name "*.zip")

for file in $zip_files; do
    # echo "解压缩文件: $file"
    unzip -qjo "$file" -d "${file%.zip}"
done

python3 plot_consensus.py

