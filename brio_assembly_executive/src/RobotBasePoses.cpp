#include <RobotBasePoses.h>

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
