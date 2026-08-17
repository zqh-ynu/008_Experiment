import pandas as pd
import plotly.express as px

# 1. 读取数据
try:
    df = pd.read_csv('./data/Autonomous_Medical_Aid_Dataset.csv')
except FileNotFoundError:
    print("错误：未找到文件。")
    exit()

# ---------------- 数据清洗与处理 ----------------

# A. 转换时间格式
df['Timestamp'] = pd.to_datetime(df['Timestamp'])

# B. 筛选 2019年10月 的数据
df_oct_2019 = df[
    (df['Timestamp'].dt.year == 2019) &
    (df['Timestamp'].dt.month == 10)
    ].copy()

# C. 创建用于分组的“日期”列 (字符串格式，作为滑块的标签)
# 格式为 "YYYY-MM-DD"
df_oct_2019['Date_Str'] = df_oct_2019['Timestamp'].dt.strftime('%Y-%m-%d')

# D. 极其重要：按时间排序
# 动画帧必须按顺序排列，否则滑块会乱跳
df_oct_2019 = df_oct_2019.sort_values(by='Timestamp')

# E. 处理气泡大小 (避免0值)
df_oct_2019['Size_Scaled'] = df_oct_2019['Number_of_Individuals_Detected'] + 1

# ---------------- 可视化配置 ----------------

print(f"正在生成地图... 共处理 {len(df_oct_2019)} 条记录。")

fig = px.scatter_mapbox(
    df_oct_2019,
    lat="Latitude",
    lon="Longitude",
    color="Rescue_Priority_Score",
    size="Size_Scaled",

    # === 关键：启用按日查看动画 ===
    animation_frame="Date_Str",
    # ==========================

    hover_data={
        "Timestamp": True,
        "Number_of_Individuals_Detected": True,
        "Rescue_Priority_Score": True,
        "Size_Scaled": False,
        "Date_Str": False
    },
    # 颜色设置：红-黄-绿 反转
    color_continuous_scale=px.colors.diverging.RdYlGn[::-1],
    size_max=20,
    zoom=9,

    # === 关键：设置英语地图 ===
    # carto-positron 是一个纯净的、英文标注的底图风格
    mapbox_style="carto-positron",

    title="Daily Drone Operations: October 2019 (English Map)"
)

# ---------------- 布局优化 ----------------

fig.update_layout(
    margin={"r": 0, "t": 50, "l": 0, "b": 0},
    coloraxis_colorbar=dict(title="Priority Score"),
    # 优化下方的滑块样式
    sliders=[{
        "currentvalue": {"prefix": "Date: "},  # 显示当前日期前缀
        "pad": {"t": 50}  # 增加滑块与地图的间距
    }]
)

# ---------------- 保存结果 ----------------
output_file = 'drone_map_daily_animation.html'
fig.write_html(output_file)

print(f"\n成功！交互式动画地图已保存为: {output_file}")
print("请在浏览器中打开。点击左下角的 'Play' 按钮即可自动播放每一天的数据变化。")