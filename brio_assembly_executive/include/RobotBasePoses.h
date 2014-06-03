#include <iostream>
#include <geometry_msgs/PoseStamped.h>

#ifndef ROBOT_BASE_POSES_H
#define ROBOT_BASE_POSES_H

class RobotBasePoses{
public:
    std::string frame_id_string;
     geometry_msgs::PoseStamped away;
     geometry_msgs::PoseStamped assembly;
     geometry_msgs::PoseStamped vision;
public:
    RobotBasePoses();
};

#endif
