#include <RobotBasePoses.h>

RobotBasePoses::RobotBasePoses(){
    this->frame_id_string = "/map";

    this->away.header.frame_id = this->frame_id_string;

    this->away.pose.position.x = 0.000;
    this->away.pose.position.y = 3.600;
    this->away.pose.position.z = 0.000;

    this->away.pose.orientation.x = 0.00;
    this->away.pose.orientation.y = 0.00;
    this->away.pose.orientation.z = -0.703;
    this->away.pose.orientation.w = 0.711;

    this->assembly.header.frame_id = this->frame_id_string;

    this->assembly.pose.position.x = 1.120;
    this->assembly.pose.position.y = 2.985;
    this->assembly.pose.position.z = 0.000;

    this->assembly.pose.orientation.x = 0.0;
    this->assembly.pose.orientation.y = 0.0;
    this->assembly.pose.orientation.z = -0.706;
    this->assembly.pose.orientation.w = 0.708;

    this->vision.header.frame_id = this->frame_id_string;

    this->vision.pose.position.x = -0.300;
    this->vision.pose.position.y = 3.064;
    this->vision.pose.position.z = 0.0;

    this->vision.pose.orientation.x = 0.0;
    this->vision.pose.orientation.y = 0.0;
    this->vision.pose.orientation.z = -0.703;
    this->vision.pose.orientation.w = 0.711;

}
