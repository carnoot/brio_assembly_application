#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <stdio.h>
#include <mutex>
#include <ros/ros.h>
#include <brio_assembly_vision/TrasformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <two_hand_ik_trajectory_executor/ExecuteLeftArmCartesianIKTrajectory.h>
#include <two_hand_ik_trajectory_executor/ExecuteRightArmCartesianIKTrajectory.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveClient;

class PieceFinalPoses{

public:
    void createPrePosition();
};

class RobotBasePoses{
private:
    std::string frame_id_string;
     geometry_msgs::PoseStamped pre_assembly;
     geometry_msgs::PoseStamped assembly;
     geometry_msgs::PoseStamped pre_vision;
     geometry_msgs::PoseStamped vision;
public:
    RobotBasePoses();
};

RobotBasePoses::RobotBasePoses(){
    this->frame_id_string = "/map";

    this->pre_assembly.header.frame_id = this->frame_id_string;

    this->pre_assembly.pose.position.x = -0.20;
    this->pre_assembly.pose.position.y = -0.45;
    this->pre_assembly.pose.position.z = 0.0;

    this->pre_assembly.pose.orientation.x = 0.0;
    this->pre_assembly.pose.orientation.y = 0.0;
    this->pre_assembly.pose.orientation.z = 1.0;
    this->pre_assembly.pose.orientation.w = 0.0;

    this->assembly.header.frame_id = this->frame_id_string;

    this->assembly.pose.position.x = -0.55;
    this->assembly.pose.position.y = -0.45;
    this->assembly.pose.position.z = 0.0;

    this->assembly.pose.orientation.x = 0.0;
    this->assembly.pose.orientation.y = 0.0;
    this->assembly.pose.orientation.z = 1.0;
    this->assembly.pose.orientation.w = 0.0;

    this->pre_vision.header.frame_id = this->frame_id_string;

    this->pre_vision.pose.position.x = -0.20;
    this->pre_vision.pose.position.y = 1.295;
    this->pre_vision.pose.position.z = 0.0;

    this->pre_vision.pose.orientation.x = 0.0;
    this->pre_vision.pose.orientation.y = 0.0;
    this->pre_vision.pose.orientation.z = 1.0;
    this->pre_vision.pose.orientation.w = 0.0;

    this->vision.header.frame_id = this->frame_id_string;

    this->vision.pose.position.x = -0.375;
    this->vision.pose.position.x = 1.295;
    this->vision.pose.position.x = 0.0;

    this->vision.pose.orientation.x = 0.0;
    this->vision.pose.orientation.y = 0.0;
    this->vision.pose.orientation.z = 1.0;
    this->vision.pose.orientation.w = 0.0;

}

class Move{

private:
    MoveClient* move_client;
public:
    Move();
    ~Move();
    void move(geometry_msgs::PoseStamped);

};

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

class Gripper{

private:
    GripperClient* gripper_client_;

public:
    Gripper();
    void open();
    void close();
    ~Gripper();

};

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

class TfPublisher {

public:
    std::string child_frame;
    std::string parent_frame;
    std::string pre_string;
    tf::StampedTransform transform;
    tf::StampedTransform pre_transform;
    tf::TransformBroadcaster br;
    int shared_variable;
    bool can_publish;

public:

    boost::thread *my_thread;
    std::mutex mut;
    TfPublisher();
    ~TfPublisher();
    void publishTransform();
    void calculate_pre_transform();

};

TfPublisher::TfPublisher() {
    std::cout << "Constructor of thread class!" << std::endl;
    this->pre_string = "pre_";
    this->my_thread = new boost::thread(
                boost::bind(&TfPublisher::publishTransform, this));
    this->can_publish = false;
}

TfPublisher::~TfPublisher() {
    std::cout << "Destructor of thread class!" << std::endl;
}

void TfPublisher::calculate_pre_transform(){

    this->pre_transform.stamp_ = ros::Time::now();

    this->pre_transform.setOrigin(tf::Vector3(this->transform.getOrigin().getX(),
                                              this->transform.getOrigin().getY(),
                                              this->transform.getOrigin().getZ()));
    this->pre_transform.setRotation(tf::Quaternion(this->transform.getRotation().getX(),
                                                   this->transform.getRotation().getY(),
                                                   this->transform.getRotation().getZ(),
                                                   this->transform.getRotation().getW()));
    this->pre_transform.frame_id_ = this->transform.frame_id_;

    this->pre_transform.child_frame_id_ = this->pre_string + this->transform.child_frame_id_;

}

void TfPublisher::publishTransform(){
    std::cout << "Publishing TRANSFORM" << std::endl;
    ros::Rate r(30);

    while(ros::ok()){
        this->mut.lock();
        this->transform.stamp_ = ros::Time::now();
        if (this->can_publish){
            this->calculate_pre_transform();
            this->br.sendTransform(this->transform);
            this->br.sendTransform(this->pre_transform);
        }

        this->mut.unlock();
        r.sleep();
    }
}

class Control{
public:

    RobotBasePoses robot_poses;

    std::string serviceName;
    std::string tfPieceFrameName;
    std::string tfBaseLinkFrameName;
    std::mutex mutex;

    ros::NodeHandle n;
    ros::ServiceClient vision_client;
    ros::ServiceClient move_left_arm_client;
    ros::ServiceClient move_right_arm_client;

    tf::StampedTransform basePieceTransform;
    tf::TransformListener tf_listener;
    brio_assembly_vision::TrasformStamped stampedTransform;

    //#################### NOT INCLUDED ###################
    two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory left_arm;
    two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory right_arm;
    //#################### NOT INCLUDED ###################

    bool move_into_position;
    bool can_call_vision_service;
    bool can_call_move_service;
    bool can_call_move_away_service;

    TfPublisher thread;

    Control();
    ~Control();

    void callVisionService();
    void copyTransformIntoThread();
    void WaitForTfTransform(std::string);
    void callMoveArmService();
};


Control::Control(){

    std::cout << "Setting up Service Client!" << std::endl;

    this->vision_client = this->n.serviceClient<brio_assembly_vision::TrasformStamped>(
                "/brio_assembly_vision");

        this->move_left_arm_client = this->n.serviceClient<two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory>(
                    "/execute_left_arm_cartesian_ik_trajectory"); //FOR LEFT ARM

    this->move_into_position = false;
    this->can_call_vision_service = true;
    this->can_call_move_service = false;
    this->can_call_move_away_service = false;

    this->serviceName = "Vision";
    this->tfBaseLinkFrameName = "base_link";
}

Control::~Control(){

}

void Control::callVisionService(){

    this->stampedTransform.request.serviceName = this->serviceName;
    if (this->vision_client.call(this->stampedTransform)) {
        std::cout << "Successfully returned from VISION Service call!" << std::endl;

        this->thread.mut.lock();

        this->copyTransformIntoThread();
        this->can_call_move_service = true;
        this->thread.can_publish = true;

        this->thread.mut.unlock();
    }
    else{
        std::cout << "VISION Service call FAILED!" << std::endl;
    }

}

void Control::callMoveArmService(){

    double offset_lh_x = -0.22;
    double offset_lh_y = 0.0;
    double offset_lh_z = 0.0;
    double rotation_lh_x = 0.0;
    double rotation_lh_y = 0.0;
    double rotation_lh_z = 0.0;

    tf::Transform transform_lh;

    transform_lh.setOrigin( tf::Vector3(offset_lh_x, offset_lh_y, offset_lh_z));
    transform_lh.setRotation( tf::Quaternion (0.0, 0.0, 0.0, 1.0 ) );

    std::cout << "Callig Move arm Service!" << std::endl;

    // this->left_arm.request.header.seq = 0;
    // this->left_arm.request.header.stamp = ros::Time::now();
    this->left_arm.request.header.frame_id = "base_link";

    tf::Transform brio_piece_transform;

    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;

    position.x = this->basePieceTransform.getOrigin().getX();
    position.y = this->basePieceTransform.getOrigin().getY();
    position.z = this->basePieceTransform.getOrigin().getZ();

    orientation.x = this->basePieceTransform.getRotation().getX();
    orientation.y = this->basePieceTransform.getRotation().getY();
    orientation.z = this->basePieceTransform.getRotation().getZ();
    orientation.w = this->basePieceTransform.getRotation().getW();

    brio_piece_transform.setOrigin( tf::Vector3(position.x, position.y, position.z) );
    brio_piece_transform.setRotation( tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w) );

    tf::Transform brio_piece_transform_ready = brio_piece_transform * transform_lh;

    position.x = brio_piece_transform_ready.getOrigin().getX();
    position.y = brio_piece_transform_ready.getOrigin().getY();
    position.z = brio_piece_transform_ready.getOrigin().getZ();

    orientation.x = brio_piece_transform_ready.getRotation().getX();
    orientation.y = brio_piece_transform_ready.getRotation().getY();
    orientation.z = brio_piece_transform_ready.getRotation().getZ();
    orientation.w = brio_piece_transform_ready.getRotation().getW();

/*
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;
*/

    geometry_msgs::Pose pose;
    pose.position = position;
    pose.orientation = orientation;

    this->left_arm.request.poses.push_back( pose );

/*
    this->left_arm.request.poses[0].position.x = this->basePieceTransform.getOrigin().getX();
    this->left_arm.request.poses[0].position.y = this->basePieceTransform.getOrigin().getY();
    this->left_arm.request.poses[0].position.z = this->basePieceTransform.getOrigin().getZ();

    this->left_arm.request.poses[0].orientation.x = this->basePieceTransform.getRotation().getX();
    this->left_arm.request.poses[0].orientation.y = this->basePieceTransform.getRotation().getY();
    this->left_arm.request.poses[0].orientation.z = this->basePieceTransform.getRotation().getZ();
    this->left_arm.request.poses[0].orientation.w = this->basePieceTransform.getRotation().getW();
*/
    std::cout << this->move_left_arm_client.getService() << std::endl;

    if (this->move_left_arm_client.call(this->left_arm)){
        if (this->left_arm.response.success == 1){
            std::cout << "SUCCESS in MOVING the ARM" << std::endl;
            this->can_call_move_away_service = true;
        }
        else
        {
            std::cout << "FAILURE in MOVING the ARM" << std::endl;
        }
    }

}

void Control::WaitForTfTransform(std::string child_frame){

    std::cout << "Waiting for transform!" << std::endl;

    try {
        this->tf_listener.waitForTransform("base_link", child_frame, ros::Time(0), ros::Duration(5));

        this->tf_listener.lookupTransform("base_link", child_frame, ros::Time(0), this->basePieceTransform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }

}

void Control::copyTransformIntoThread(){

    this->thread.transform.setOrigin(tf::Vector3(this->stampedTransform.response.msg.transform.translation.x,
                                                 this->stampedTransform.response.msg.transform.translation.y,
                                                 this->stampedTransform.response.msg.transform.translation.z));

    this->thread.transform.setRotation(tf::Quaternion(this->stampedTransform.response.msg.transform.rotation.x,
                                                      this->stampedTransform.response.msg.transform.rotation.y,
                                                      this->stampedTransform.response.msg.transform.rotation.z,
                                                      this->stampedTransform.response.msg.transform.rotation.w));
    /*
    this->thread.child_frame = this->stampedTransform.response.msg.child_frame_id;

    std::cout<< "child_frame " << this->thread.child_frame << std::endl;

    this->thread.parent_frame = this->stampedTransform.response.msg.header.frame_id;

    std::cout<< "parent_frame " << this->thread.parent_frame << std::endl;
*/

    this->thread.transform.frame_id_ =
            this->stampedTransform.response.msg.header.frame_id; //this->thread.parent_frame;

    this->thread.transform.child_frame_id_ = //this->thread.child_frame;
            this->stampedTransform.response.msg.child_frame_id;

    this->tfPieceFrameName = this->thread.transform.child_frame_id_;

    std::cout<< "child_frame  " << this->thread.transform.child_frame_id_ <<std::endl;
    std::cout<< "parent_frame " << this->thread.transform.frame_id_ << std::endl;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "CONTROL");
    Control contr;
    while (1){

        if (contr.can_call_vision_service){
            contr.callVisionService();
        }

        if (contr.can_call_move_service == true){
            contr.WaitForTfTransform(contr.thread.pre_transform.child_frame_id_);
            contr.callMoveArmService();
            contr.WaitForTfTransform(contr.thread.transform.child_frame_id_);
            contr.callMoveArmService();
        }

    }

}
