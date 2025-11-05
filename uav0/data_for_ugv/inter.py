import pandas as pd
import numpy as np

df = pd.read_csv(
    'filtered_map_vel01.csv',
    header=None,          # 声明无表头
    names=['t', 'x', 'y'],  # 手动命名列（第一列时间，第二x，第三y）
    parse_dates=['t']  # 解析第一列（time）为 datetime 格式
)

start_t = df['t'].min()
end_t = df['t'].max()

# print(start_t, end_t)

target_t = np.arange(start_t, end_t + 0.01, 0.01)

df_resampled = pd.DataFrame({
    't': target_t,
    'x': np.interp(target_t, df['t'], df['x']),  # 对x插值
    'y': np.interp(target_t, df['t'], df['y'])   # 对y插值
})

# 4. 保存结果（可选）
df_resampled.to_csv('int_filtered_map_vel01.csv', index=False)  # 不保存索引

# 打印结果查看
print("10ms间隔插值后的数据（前20行）：")
print(df_resampled.head(20))