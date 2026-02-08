#include "local_planning/obstacles/obstacle.h"


namespace Obstacle
{
// 获取障碍物在参考线上的投影（并计算相关信息）
void Obstacle::CalculateProjection(const Path::ReferencePath::Ptr & reference_path,
        const Path::PathNode & ego_position)
{
    // 将障碍物中心投影到参考线
    auto [proj_point, proj_idx] = reference_path->GetProjection(
        { perception_obstacle_.pose.position.x, perception_obstacle_.pose.position.y });

    // 计算SL坐标
    Path::PointSL sl = Path::Utils::XYtoSL(
        { perception_obstacle_.pose.position.x, perception_obstacle_.pose.position.y },
        { proj_point.x, proj_point.y },
        proj_point.s,
        proj_point.theta);

    projection_.s = sl.s;
    projection_.l = sl.l;

    // 估算投影尺寸
    double obstacle_yaw = tf2::getYaw(perception_obstacle_.pose.orientation);
    double ref_yaw = proj_point.theta;
    double angle_diff = std::abs(obstacle_yaw - ref_yaw);

    projection_.length = dimension_.x * std::cos(angle_diff) +
        dimension_.y * std::sin(angle_diff);
    projection_.width = dimension_.x * std::sin(angle_diff) +
        dimension_.y * std::cos(angle_diff);

    // 判断是否在自车前方
    projection_.is_ahead = (projection_.s > ego_position.s);

    // 计算欧氏距离
    double dx = perception_obstacle_.pose.position.x - ego_position.x;
    double dy = perception_obstacle_.pose.position.y - ego_position.y;
    projection_.distance = std::sqrt(dx * dx + dy * dy);

    // 计算相对速度（需要自车速度，这里只设置占位符，实际由外部计算）
    projection_.relative_speed = 0.0;
    projection_.time_to_collision = 999.0;
}

// 更新碰撞时间信息
void Obstacle::UpdateCollisionTime(double ego_speed, double ego_acceleration)
{
    if (!projection_.is_ahead)
    {
        projection_.time_to_collision = 999.0;
        return;
    }

    projection_.relative_speed = ego_speed - speed_;

    if (std::abs(projection_.relative_speed) > 0.1)
    {
        double distance_to_obstacle = std::max(0.0, projection_.s - projection_.length / 2.0);
        projection_.time_to_collision = distance_to_obstacle / std::max(0.1, std::abs(projection_.relative_speed));
    }
    else
    {
        projection_.time_to_collision = 999.0;
    }
}


} // namespace Obstacle

