#include <Move.h>

Move::Move(){

    this->move_client = new MoveClient("/nav_pcontroller/move_base", true);

    while (!this->move_client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the /nav_pcontroller/move_base action server to come up!");

    }
}

void Move::move(geometry_msgs::PoseStamped pose){

    move_base_msgs::MoveBaseAction my_action;

    ROS_INFO("Sending MOVE Command!");

    pose.header.stamp = ros::Time::now();
    my_action.action_goal.goal.target_pose = pose;

    this->move_client->sendGoal(my_action.action_goal.goal);
    this->move_client->waitForResult();
    if (this->move_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Moved into position!");
    }
    else{
        ROS_INFO("Move failed!");
    }

}
