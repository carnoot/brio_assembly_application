#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <stdio.h>
#include <mutex>
#include <ros/ros.h>
#include <brio_assembly_vision/TrasformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <two_hand_ik_trajectory_executor/ExecuteLeftArmCartesianIKTrajectory.h>
#include <two_hand_ik_trajectory_executor/ExecuteRightArmCartesianIKTrajectory.h>

class thread_class {

public:
    std::string child_frame;
    std::string parent_frame;
    tf::StampedTransform transform;
    tf::TransformBroadcaster br;
    int shared_variable;
    bool can_publish;

public:

    boost::thread *my_thread;
    std::mutex mut;
    thread_class();
    ~thread_class();
    void publishTransform();

};

thread_class::thread_class() {
    std::cout << "Constructor of thread class!" << std::endl;
    this->my_thread = new boost::thread(
                boost::bind(&thread_class::publishTransform, this));
    this->can_publish = false;
}

thread_class::~thread_class() {
    std::cout << "Destructor of thread class!" << std::endl;
}

void thread_class::publishTransform(){
    std::cout << "Publishing TRANSFORM" << std::endl;
    ros::Rate r(30);

    while(ros::ok()){
        this->mut.lock();
        this->transform.stamp_ = ros::Time::now();
	if (this->can_publish)
        this->br.sendTransform(this->transform);
        //    this->br.sendTransform(this->transform, ros::Time::now(), this->parent_frame, this->child_frame);
        this->mut.unlock();
        r.sleep();
    }
}

class Control{
public:
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

    thread_class thread;

    Control();
    ~Control();

    void callVisionService();
    void copyTransformIntoThread();
    void WaitForTfTransform();
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
    this->tfBaseLinkFrameName = "base_link"; //TF FRAME NAME OF PR2 BASE_LINK ??
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

void Control::WaitForTfTransform(){

	std::cout << "Waiting for transform!" << std::endl;

    try {
        this->tf_listener.waitForTransform("base_link",        //"brio_piece_frame", //this->tfPieceFrameName,
                                           "brio_piece_frame", //"base_link",        //this->tfBaseLinkFrameName,
                                           ros::Time(0), ros::Duration(5));

        this->tf_listener.lookupTransform("base_link",        //"brio_piece_frame", //this->tfPieceFrameName,
                                          "brio_piece_frame", //"base_link",        //this->tfBaseLinkFrameName,
                                          ros::Time(0), this->basePieceTransform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
   
    std::cout << "base: " << this->tfBaseLinkFrameName<< std::endl;

    std::cout << "piece: " << this->tfPieceFrameName << std::endl;
   
    std::cout << "parent: " << this->basePieceTransform.frame_id_ << std::endl;

    std::cout << "child: " << this->basePieceTransform.child_frame_id_ << std::endl;

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
            //contr.can_call_vision_service = false;
        }

        sleep(1);

        // ROS_INFO("Done with one loop ...");


        if (contr.can_call_move_service == true){
            contr.WaitForTfTransform();
            contr.callMoveArmService();
        }

/*
        if (contr.can_call_move_away_service == true){

            contr.callMoveArmService();

        }
*/
    }

}

