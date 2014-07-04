//#include <boost/thread.hpp>
//#include <boost/bind.hpp>
//#include <iostream>
//#include <stdio.h>
//#include <mutex>
//#include <ros/ros.h>
//#include <brio_assembly_vision/TrasformStamped.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>

//#include <PieceFinalPoses.h>
//#include <RobotBasePoses.h>
//#include <TfPublisher.h>
//#include <Move.h>
//#include <Gripper.h>

<<<<<<< Updated upstream
#include <two_hand_ik_trajectory_executor/ExecuteLeftArmCartesianIKTrajectory.h>
#include <two_hand_ik_trajectory_executor/ExecuteRightArmCartesianIKTrajectory.h>
#include <two_hand_ik_trajectory_executor/ExecuteBothArmsCartesianIKTrajectory.h>
=======
////#include <two_hand_ik_trajectory_executor/ExecuteLeftArmCartesianIKTrajectory.h>
////#include <two_hand_ik_trajectory_executor/ExecuteRightArmCartesianIKTrajectory.h>
>>>>>>> Stashed changes

//class Control{
//public:

//    RobotBasePoses robot_poses;

//    std::string serviceName;
//    std::string tfPieceFrameName;
//    std::string tfBaseLinkFrameName;
//    std::mutex mutex;

<<<<<<< Updated upstream
    ros::NodeHandle n;
    ros::ServiceClient vision_client;
    ros::ServiceClient move_left_arm_client;
    ros::ServiceClient move_right_arm_client;
    ros::ServiceClient move_away_arms;
=======
//    ros::NodeHandle n;
//    ros::ServiceClient vision_client;
//    ros::ServiceClient move_left_arm_client;
//    ros::ServiceClient move_right_arm_client;
>>>>>>> Stashed changes

//    tf::StampedTransform basePieceTransform;
//    tf::TransformListener tf_listener;
//    brio_assembly_vision::TrasformStamped stampedTransform;

<<<<<<< Updated upstream
    //#################### NOT INCLUDED ###################
    two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory left_arm;
    two_hand_ik_trajectory_executor::ExecuteRightArmCartesianIKTrajectory right_arm;
    two_hand_ik_trajectory_executor::ExecuteBothArmsCartesianIKTrajectory both_arm;
    //#################### NOT INCLUDED ###################
=======
//    //#################### NOT INCLUDED ###################
//    //two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory left_arm;
//    //two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory right_arm;
//    //#################### NOT INCLUDED ###################
>>>>>>> Stashed changes

//    bool move_into_position;
//    bool can_call_vision_service;
//    bool can_call_move_service;
//    bool can_call_move_away_service;

//    TfPublisher thread;

//    Control();
//    ~Control();

<<<<<<< Updated upstream
    void callVisionService();
    void copyTransformIntoThread();
    void WaitForTfTransform(std::string &, int);
    void CallMoveLeftArmService(ros::ServiceClient&,
                            two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory&);
    void CallMoveRightArmService(ros::ServiceClient&,
                                 two_hand_ik_trajectory_executor::ExecuteRightArmCartesianIKTrajectory&);

    void MoveAwayBothArms(ros::ServiceClient&,
                          two_hand_ik_trajectory_executor::ExecuteBothArmsCartesianIKTrajectory&);
};
=======
//    //void callVisionService();
//    //void copyTransformIntoThread();
//    //void WaitForTfTransform(std::string);
//    //void CallMoveArmService(ros::ServiceClient&,
//                            //two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory&);
//};
>>>>>>> Stashed changes


//Control::Control(){

//    std::cout << "Setting up Service Client!" << std::endl;

//    this->vision_client = this->n.serviceClient<brio_assembly_vision::TrasformStamped>(
//                "/brio_assembly_vision");

//    this->move_left_arm_client = this->n.serviceClient<two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory>(
//                "/execute_left_arm_cartesian_ik_trajectory");

//    this->move_right_arm_client = this->n.serviceClient<two_hand_ik_trajectory_executor::ExecuteRightArmCartesianIKTrajectory>(
//                "/execute_right_arm_cartesian_ik_trajectory");

<<<<<<< Updated upstream
    this->move_away_arms = this->n.serviceClient<two_hand_ik_trajectory_executor::ExecuteBothArmsCartesianIKTrajectory>(
                "/execute_both_arms_cartesian_ik_trajectory");

    this->move_into_position = false;
    this->can_call_vision_service = true;
    this->can_call_move_service = false;
    this->can_call_move_away_service = false;
=======
//    this->move_into_position = false;
//    this->can_call_vision_service = true;
//    this->can_call_move_service = false;
//    this->can_call_move_away_service = false;
>>>>>>> Stashed changes

//    this->serviceName = "Vision";
//    this->tfBaseLinkFrameName = "base_link";
//}

//Control::~Control(){

//}

//void Control::callVisionService(){

//    this->stampedTransform.request.serviceName = this->serviceName;
//    if (this->vision_client.call(this->stampedTransform)) {
//        std::cout << "Successfully returned from VISION Service call!" << std::endl;

//        this->thread.mut.lock();

//        this->copyTransformIntoThread();

<<<<<<< Updated upstream
        this->thread.can_publish = true;
=======
////      this->can_call_move_service = true;

//        this->thread.can_publish = true;
>>>>>>> Stashed changes

//        this->thread.mut.unlock();
//    }
//    else{
//        std::cout << "VISION Service call FAILED!" << std::endl;
//    }

//}

<<<<<<< Updated upstream
void Control::CallMoveLeftArmService(ros::ServiceClient& client,
                                 two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory& arm){

    arm.request.poses.clear();

    double offset_lh_x = -0.22;
    double offset_lh_y = 0.0;
    double offset_lh_z = 0.0;
    double rotation_lh_x = 0.0;
    double rotation_lh_y = 0.0;
    double rotation_lh_z = 0.0;
    double grasp_offset = 0.05;
=======
//void Control::CallMoveArmService(ros::ServiceClient& client,
//                                 two_hand_ik_trajectory_executor::ExecuteLeftArmCartesianIKTrajectory& arm){

//    double offset_lh_x = -0.22;
//    double offset_lh_y = 0.0;
//    double offset_lh_z = 0.0;
//    double rotation_lh_x = 0.0;
//    double rotation_lh_y = 0.0;
//    double rotation_lh_z = 0.0;
//    double grasp_offset = 0.05;
>>>>>>> Stashed changes

//    tf::Transform transform_lh;

//    transform_lh.setOrigin( tf::Vector3(offset_lh_x, offset_lh_y, offset_lh_z));
//    transform_lh.setRotation( tf::Quaternion (0.0, 0.0, 0.0, 1.0 ) );

//    std::cout << "Callig Move arm Service!" << std::endl;

//    // this->left_arm.request.header.seq = 0;
//    // this->left_arm.request.header.stamp = ros::Time::now();
//    this->left_arm.request.header.frame_id = "base_link";

//    tf::Transform brio_piece_transform;

//    geometry_msgs::Point position;
//    geometry_msgs::Quaternion orientation;

//    position.x = this->basePieceTransform.getOrigin().getX();
//    position.y = this->basePieceTransform.getOrigin().getY();
//    position.z = this->basePieceTransform.getOrigin().getZ(); // +- GRASP_OFFSET $####################

//    orientation.x = this->basePieceTransform.getRotation().getX();
//    orientation.y = this->basePieceTransform.getRotation().getY();
//    orientation.z = this->basePieceTransform.getRotation().getZ();
//    orientation.w = this->basePieceTransform.getRotation().getW();

//    brio_piece_transform.setOrigin( tf::Vector3(position.x, position.y, position.z) );
//    brio_piece_transform.setRotation( tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w) );

//    tf::Transform brio_piece_transform_ready = brio_piece_transform * transform_lh;

//    position.x = brio_piece_transform_ready.getOrigin().getX();
//    position.y = brio_piece_transform_ready.getOrigin().getY();
//    position.z = brio_piece_transform_ready.getOrigin().getZ();

//    orientation.x = brio_piece_transform_ready.getRotation().getX();
//    orientation.y = brio_piece_transform_ready.getRotation().getY();
//    orientation.z = brio_piece_transform_ready.getRotation().getZ();
//    orientation.w = brio_piece_transform_ready.getRotation().getW();

//    geometry_msgs::Pose pose;
//    pose.position = position;
//    pose.orientation = orientation;

//    this->left_arm.request.poses.push_back( pose );

//    std::cout << client.getService() << std::endl;

<<<<<<< Updated upstream
    if (client.call(arm)){
        if (arm.response.success == 1){
            std::cout << "SUCCESS in MOVING the LEFT ARM!" << std::endl;
            this->can_call_move_away_service = true;
        }
        else
        {
            std::cout << "FAILURE in MOVING the LEFT ARM!" << std::endl;
        }
    }

}

void Control::MoveAwayBothArms(ros::ServiceClient& client,
                               two_hand_ik_trajectory_executor::ExecuteBothArmsCartesianIKTrajectory& arms){

    arms.request.lh_poses.clear();
    arms.request.rh_poses.clear();
    arms.request.stop_at_pose.clear();

    geometry_msgs::Pose left_pose;
    geometry_msgs::Pose right_pose;

    left_pose.position.x = 0.40;
    left_pose.position.y = 0.70;
    left_pose.position.z = 1.20;

    left_pose.orientation.x = 0.0;
    left_pose.orientation.y = 0.0;
    left_pose.orientation.z = 0.0;
    left_pose.orientation.w = 1.0;

    right_pose.position.x = 0.40;
    right_pose.position.y = -0.70;
    right_pose.position.z = 1.20;

    right_pose.orientation.x = 0.0;
    right_pose.orientation.y = 0.0;
    right_pose.orientation.z = 0.0;
    right_pose.orientation.w = 1.0;

//    arms.request.lh_poses.resize(1);
    arms.request.lh_poses.push_back(left_pose);

//    arms.request.rh_poses.resize(1);
    arms.request.rh_poses.push_back(right_pose);

    arms.request.header.frame_id = "base_link";

    arms.request.stop_at_pose.push_back(1);

    std::cout << client.getService() << std::endl;

    if (client.call(arms)){
        if (arms.response.success == 1){
            std::cout << "SUCCESS in MOVING BOTH ARMS!" << std::endl;
            this->can_call_move_away_service = true;
        }
        else
        {
            std::cout << "FAILURE in MOVING BOTH ARMS!" << std::endl;
        }
    }

}

void Control::CallMoveRightArmService(ros::ServiceClient& client,
                                 two_hand_ik_trajectory_executor::ExecuteRightArmCartesianIKTrajectory& arm){

    arm.request.poses.clear();

    double offset_lh_x = -0.22;
    double offset_lh_y = 0.0;
    double offset_lh_z = 0.0;
    double rotation_lh_x = 0.0;
    double rotation_lh_y = 0.0;
    double rotation_lh_z = 0.0;
    double grasp_offset = 0.05;

    tf::Transform transform_lh;

    transform_lh.setOrigin( tf::Vector3(offset_lh_x, offset_lh_y, offset_lh_z));
    transform_lh.setRotation( tf::Quaternion (0.0, 0.0, 0.0, 1.0 ) );

    std::cout << "Callig Move arm Service!" << std::endl;

    arm.request.header.frame_id = "base_link";

    tf::Transform brio_piece_transform;

    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;

    position.x = this->basePieceTransform.getOrigin().getX();
    position.y = this->basePieceTransform.getOrigin().getY();
    position.z = this->basePieceTransform.getOrigin().getZ(); // +- GRASP_OFFSET $####################

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

    geometry_msgs::Pose pose;
    pose.position = position;
    pose.orientation = orientation;

    arm.request.poses.push_back( pose );

    std::cout << client.getService() << std::endl;

    if (client.call(arm)){
        if (arm.response.success == 1){
            std::cout << "SUCCESS in MOVING the RIGHT ARM!" << std::endl;
            this->can_call_move_away_service = true;
        }
        else
        {
            std::cout << "FAILURE in MOVING the RIGHT ARM!" << std::endl;
        }
    }
=======
//    if (client.call(arm)){
//        if (this->left_arm.response.success == 1){
//            std::cout << "SUCCESS in MOVING the ARM" << std::endl;
//            this->can_call_move_away_service = true;
//        }
//        else
//        {
//            std::cout << "FAILURE in MOVING the ARM" << std::endl;
//        }
//    }
>>>>>>> Stashed changes

//}

<<<<<<< Updated upstream
void Control::WaitForTfTransform(std::string& child_frame, int poz){
=======
//void Control::WaitForTfTransform(std::string child_frame){
>>>>>>> Stashed changes

//    std::cout << "Waiting for transform!" << std::endl;

<<<<<<< Updated upstream
    try {
        this->tf_listener.waitForTransform("base_link", child_frame, ros::Time(0), ros::Duration(10));

        this->tf_listener.lookupTransform("base_link", child_frame, ros::Time(0), this->basePieceTransform);

        if (poz == 0){
        this->basePieceTransform.setOrigin(tf::Vector3(this->basePieceTransform.getOrigin().getX(),
                                                       this->basePieceTransform.getOrigin().getY(),
                                                       this->basePieceTransform.getOrigin().getZ() + 0.20));
        }
        if (poz == 1){
            this->basePieceTransform.setOrigin(tf::Vector3(this->basePieceTransform.getOrigin().getX(),
                                                           this->basePieceTransform.getOrigin().getY(),
                                                           this->basePieceTransform.getOrigin().getZ() - 0.07));
        }
        if (poz == 2){
            this->basePieceTransform.setOrigin(tf::Vector3(this->basePieceTransform.getOrigin().getX(),
                                                           this->basePieceTransform.getOrigin().getY(),
                                                           this->basePieceTransform.getOrigin().getZ())); // pozitia hardcodata de destinatie, nu mai trebuie adaugata nimic!!
        }
    } catch (tf::TransformException &ex) {
        std::cout << "INSTEAD of ROS ERROR in WaitForTfTransform!"<< std::endl;
//        ROS_ERROR("%s", ex.what());
    }
=======
//    try {
//        this->tf_listener.waitForTransform("base_link", child_frame, ros::Time(0), ros::Duration(5));

//        this->tf_listener.lookupTransform("base_link", child_frame, ros::Time(0), this->basePieceTransform);
//    } catch (tf::TransformException &ex) {
//        ROS_ERROR("%s", ex.what());
//    }
>>>>>>> Stashed changes

//}

//void Control::copyTransformIntoThread(){

//    this->thread.transform.setOrigin(tf::Vector3(this->stampedTransform.response.msg.transform.translation.x,
//                                                 this->stampedTransform.response.msg.transform.translation.y,
//                                                 this->stampedTransform.response.msg.transform.translation.z));

//    this->thread.transform.setRotation(tf::Quaternion(this->stampedTransform.response.msg.transform.rotation.x,
//                                                      this->stampedTransform.response.msg.transform.rotation.y,
//                                                      this->stampedTransform.response.msg.transform.rotation.z,
//                                                      this->stampedTransform.response.msg.transform.rotation.w));
//    /*
//    this->thread.child_frame = this->stampedTransform.response.msg.child_frame_id;

//    std::cout<< "child_frame " << this->thread.child_frame << std::endl;

//    this->thread.parent_frame = this->stampedTransform.response.msg.header.frame_id;

//    std::cout<< "parent_frame " << this->thread.parent_frame << std::endl;
//*/

//    this->thread.transform.frame_id_ =
//            this->stampedTransform.response.msg.header.frame_id; //this->thread.parent_frame;

//    this->thread.transform.child_frame_id_ = //this->thread.child_frame;
//            this->stampedTransform.response.msg.child_frame_id;

//    this->tfPieceFrameName = this->thread.transform.child_frame_id_;

//    std::cout<< "child_frame  " << this->thread.transform.child_frame_id_ <<std::endl;
//    std::cout<< "parent_frame " << this->thread.transform.frame_id_ << std::endl;
//}

int main(int argc, char** argv) {

//    ros::init(argc, argv, "CONTROL");
//    Control contr;
//    Move move;
//    Gripper gripper;
//    while (1){

<<<<<<< Updated upstream
        move.move(move.poses.away);

        contr.MoveAwayBothArms(contr.move_away_arms, contr.both_arm);

        gripper.open(gripper.getRightGripper());
        gripper.open(gripper.getLeftGripper());

        ROS_INFO("MOVING TO AWAY");
=======
//        ROS_INFO("MOVING TO AWAY");
>>>>>>> Stashed changes

//        move.move(move.poses.away);

//        ROS_INFO("MOVING TO VISION");

//        move.move(move.poses.vision);

//        sleep(5);

//        contr.callVisionService();

<<<<<<< Updated upstream
        sleep(3);

        std::cout << "this->pre_transform.frame_id IN MAIN: " << contr.thread.pre_transform.frame_id_ << std::endl;

        contr.WaitForTfTransform(contr.thread.pre_transform.child_frame_id_, 0);
        contr.CallMoveRightArmService(contr.move_right_arm_client, contr.right_arm);

        gripper.open(gripper.getRightGripper());

        contr.WaitForTfTransform(contr.thread.transform.child_frame_id_, 1);
        contr.CallMoveRightArmService(contr.move_right_arm_client, contr.right_arm);

        gripper.close(gripper.getRightGripper());

        contr.MoveAwayBothArms(contr.move_away_arms, contr.both_arm);
=======
//        contr.WaitForTfTransform(contr.thread.pre_transform.child_frame_id_);
//        contr.CallMoveArmService(contr.move_left_arm_client, contr.left_arm);

//        gripper.open();

//        contr.WaitForTfTransform(contr.thread.transform.child_frame_id_);
//        contr.CallMoveArmService(contr.move_left_arm_client, contr.left_arm);

//        gripper.close();
>>>>>>> Stashed changes

//        move.move(move.poses.away);

//        move.move(move.poses.assembly);

<<<<<<< Updated upstream
        contr.WaitForTfTransform(contr.thread.final_poses.round_piece_pre_transform.child_frame_id_,0);
        contr.CallMoveRightArmService(contr.move_right_arm_client, contr.right_arm);

//        sleep(3);

//        contr.WaitForTfTransform(contr.thread.final_poses.round_piece_transform.child_frame_id_,2);
//        contr.CallMoveRightArmService(contr.move_right_arm_client, contr.right_arm);

    }
=======
//        contr.WaitForTfTransform(contr.thread.final_poses.child_frame_round);
//        contr.CallMoveArmService(contr.move_left_arm_client, contr.left_arm);
//    }
>>>>>>> Stashed changes

}
