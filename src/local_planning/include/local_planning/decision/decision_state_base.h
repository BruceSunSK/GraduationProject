// decision_state_base.h
#pragma once
#include <memory>
#include <string>

#include "global_planning/path/utils.h"
#include "global_planning/path/reference_path.h"
#include "global_planning/map/distance_map.h"
#include "local_planning/decision/data_type.h"


namespace Decision
{
struct DecisionContext
{
    Path::PathNode ego_position;
    double ego_speed;
    double ego_acceleration;

    int obstacle_id;
    int obstacle_type;
    double obstacle_speed;
    geometry_msgs::Vector3 obstacle_dimension;
    ObstacleProjection projection;

    double curvature;
    double cost_value;

    std::vector<perception::PredictedTrajectory> predicted_trajectory;

    double lane_width;
    double road_width;

    DecisionType last_state;
    int state_count;
    double state_time;

    double timestamp;
};

class DecisionState
{
public:
    virtual ~DecisionState() = default;
    virtual DecisionType Evaluate(DecisionContext & context) = 0;
    virtual std::string GetName() const = 0;
    virtual DecisionType GetType() const = 0;
};

} // namespace Decision