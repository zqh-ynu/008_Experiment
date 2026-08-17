此文件夹中的配置文件主要存储一些关于系统基础设置的参数，例如无人机高度、信号频段、平均阴影噪声等。

# 1. 环境参数常量

| 参数类别        | 参数名称           | 参数名          | 设定值/范围                       | 设置合理性与依据                                  |
| :---------- | :------------- | :----------- | :--------------------------- | :---------------------------------------- |
| **环境参数常量**  | 视距损失参数(dB)     | los_loss_db  | $\eta_{\text{LoS}}=1$ dB<br> | 城市环境参数                                    |
|             | 非视距损失参数(dB)    | nlos_loss_db | $\eta_{\text{NLoS}}=20$ dB   | 城市环境参数                                    |
|             | 白噪声            | noise_dbm    | -105 dB                      |                                           |
|             | UAV-user视距概率参数 | param_a      | $a = 9.611725$<br>           | [\[1\]](al-houraniOptimalLAPAltitude2014) |
|             | UAV-user视距概率参数 | param_b      | $b = 0.158062$               | [\[1\]](al-houraniOptimalLAPAltitude2014) |
|             | 光速             | speed_light  | 299,792,458 m/s              |                                           |
| **无人机参数常量** | 无人机高度          | uav_alt      | 300m                         | [\[1\]](al-houraniOptimalLAPAltitude2014) |
|             | 载波频率           | freq_hz      | $2.4$ GHz                    |                                           |
|             | 发射功率           | trans_power  | 10 W                         |                                           |
|             | 天线增益           | gain_uav_db  | 5 dB                         |                                           |
