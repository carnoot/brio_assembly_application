#include <Gripper.h>

Gripper::Gripper(){

    this->left_gripper = new GripperClient("l_gripper_controller/gripper_action", true);
    this->right_gripper = new GripperClient("r_gripper_controller/gripper_action", true);

    while(!this->right_gripper->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }

    while(!this->left_gripper->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
    }
}

Gripper::~Gripper(){
    delete this->left_gripper;
    delete this->right_gripper;
}

GripperClient* Gripper::getLeftGripper(){
    return this->left_gripper;
}

GripperClient* Gripper::getRightGripper(){
    return this->right_gripper;
}

void Gripper::open(GripperClient* gripper){

    pr2_controllers_msgs::Pr2GripperCommandGoal open;

    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");

    gripper->sendGoal(open);
    gripper->waitForResult();
    if(gripper->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("The gripper opened!");
    }
    else
    {
        ROS_INFO("The gripper failed to open.");
    }
}

void Gripper::close(GripperClient* gripper){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0; // DE CALCULAT CAT ESTE NECESAR
    squeeze.command.max_effort = 50.0;  // Close gently

    ROS_INFO("Sending squeeze goal");
    gripper->sendGoal(squeeze);
    gripper->waitForResult();
    if(gripper->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("The gripper closed!");
    }
    else
    {
        ROS_INFO("The gripper failed to close.");
    }
}
