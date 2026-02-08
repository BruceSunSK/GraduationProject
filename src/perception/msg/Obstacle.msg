# 障碍物类型枚举
uint8 UNKNOWN=0
uint8 VEHICLE=1
uint8 PEDESTRIAN=2
uint8 BICYCLE=3

Header header
uint8 id                         # 障碍物唯一ID
uint8 type                       # 障碍物类型
geometry_msgs/Pose pose         # 当前位置和朝向
geometry_msgs/Vector3 dimension # 尺寸：长、宽、高
float64 speed                   # 当前速度(m/s)

# 预测轨迹，固定10个点，时间间隔0.5s
PredictedTrajectory[] predicted_trajectory