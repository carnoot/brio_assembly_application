#include <RobotBasePoses.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/action_definition.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#ifndef MOVE_H
#define MOVE_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveClient;

class Move{

public:
    MoveClient* move_client;
    RobotBasePoses poses;
public:
    Move();
    void move(geometry_msgs::PoseStamped);

};

#endif
