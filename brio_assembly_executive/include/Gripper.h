#include <actionlib/action_definition.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

#ifndef GRIPPER_H
#define GRIPPER_H

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{

private:
    GripperClient* left_gripper;
    GripperClient* right_gripper;

public:
    Gripper();
    void open(GripperClient*);
    void close(GripperClient*);
    GripperClient* getLeftGripper();
    GripperClient* getRightGripper();
    ~Gripper();

};

#endif
