// obstacle.h
#pragma once
#include <vector>
#include <memory>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/utils.h>

#include "global_planning/path/utils.h"
#include "global_planning/path/reference_path.h"
#include "local_planning/decision/data_type.h"
#include "perception/Obstacle.h"
#include "perception/PredictedObstacles.h"


namespace Obstacle
{

// 障碍物包装类（添加决策信息）
class Obstacle
{
public:
    using Ptr = std::shared_ptr<Obstacle>;
    using List = std::vector<Ptr>;

public:
    Obstacle() = default;

    explicit Obstacle(const perception::Obstacle & perception_obstacle)
        : perception_obstacle_(perception_obstacle)
    {
        id_ = perception_obstacle.id;
        type_ = static_cast<int>(perception_obstacle.type);
        speed_ = perception_obstacle.speed;
        dimension_ = perception_obstacle.dimension;
        pose_ = perception_obstacle.pose;
    }

    // Getters
    int GetId() const { return id_; }
    int GetType() const { return type_; }
    double GetSpeed() const { return speed_; }
    const geometry_msgs::Pose & GetPose() const { return pose_; }
    const geometry_msgs::Vector3 & GetDimension() const { return dimension_; }
    const Decision::DecisionInfo & GetDecision() const { return decision_; }
    const Decision::ObstacleProjection & GetProjection() const { return projection_; }
    const perception::Obstacle & GetPerceptionObstacle() const { return perception_obstacle_; }

    // Setters
    void SetDecision(const Decision::DecisionInfo & decision) { decision_ = decision; }
    void SetProjection(const Decision::ObstacleProjection & projection) { projection_ = projection; }
    void SetDecision(Decision::DecisionType type, double confidence, double target_speed = 0.0)
    {
        decision_.type = type;
        decision_.decision_confidence = confidence;
        decision_.target_speed = target_speed;
    }

    // 获取障碍物在参考线上的投影（并计算相关信息）
    void CalculateProjection(const Path::ReferencePath::Ptr & reference_path,
        const Path::PathNode & ego_position);

    // 更新碰撞时间信息
    void UpdateCollisionTime(double ego_speed, double ego_acceleration = 0.0);

    std::string ToString() const
    {
        std::stringstream ss;
        ss << "Obstacle{id=" << id_
            << ", type=" << type_
            << ", speed=" << speed_
            << ", projection=" << projection_.ToString()
            << ", decision=" << decision_.ToString() << "}";
        return ss.str();
    }

private:
    int id_ = -1;
    int type_ = 0;
    double speed_ = 0.0;
    geometry_msgs::Pose pose_;
    geometry_msgs::Vector3 dimension_;
    perception::Obstacle perception_obstacle_;

    Decision::DecisionInfo decision_;
    Decision::ObstacleProjection projection_;
};

} // namespace Obstacle