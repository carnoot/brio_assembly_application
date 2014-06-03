#include <Gripper.h>

Gripper::Gripper(){

    this->gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);

    while(!this->gripper_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
    }
}

Gripper::~Gripper(){
    delete gripper_client_;
}

void Gripper::open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");

    this->gripper_client_->sendGoal(open);
    this->gripper_client_->waitForResult();
    if(this->gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The gripper opened!");
    else
        ROS_INFO("The gripper failed to open.");
}

void Gripper::close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0; // DE CALCULAT CAT ESTE NECESAR
    squeeze.command.max_effort = 50.0;  // Close gently

    ROS_INFO("Sending squeeze goal");
    this->gripper_client_->sendGoal(squeeze);
    this->gripper_client_->waitForResult();
    if(this->gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The gripper closed!");
    else
        ROS_INFO("The gripper failed to close.");
}
