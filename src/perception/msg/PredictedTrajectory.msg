# 单个预测轨迹点
float64 time_relative           # 相对当前时间(s)
geometry_msgs/Pose pose         # 预测位置和朝向
float64 speed                   # 预测速度(m/s)
float64 confidence              # 置信度[0,1]