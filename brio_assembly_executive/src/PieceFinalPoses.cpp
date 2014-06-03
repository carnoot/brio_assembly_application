#include <PieceFinalPoses.h>

PieceFinalPoses::PieceFinalPoses(){

    z_offset = 0.15;

    this->frame_id = "base_link";
    this->pre_frame_string = "pre_";
    this->child_frame_straight = "straight_piece_dest";
    this->child_frame_round = "round_piece_dest";

    this->round_piece_transform.child_frame_id_ = this->child_frame_round;
    this->round_piece_transform.frame_id_ = this->frame_id;
    this->round_piece_transform.setOrigin(tf::Vector3(0.549, -0.057, 0.748));
    this->round_piece_transform.setRotation(tf::Quaternion(-0.295, 0.645, 0.298, 0.639).normalize());

    this->round_piece_pre_transform.child_frame_id_ = this->pre_frame_string + this->child_frame_round;
    this->round_piece_pre_transform.frame_id_ = this->frame_id;
    this->round_piece_pre_transform.setOrigin(tf::Vector3(this->round_piece_transform.getOrigin().getX(),
                                                          this->round_piece_transform.getOrigin().getY(),
                                                          this->round_piece_transform.getOrigin().getZ() + z_offset));
    this->round_piece_pre_transform.setRotation(tf::Quaternion(this->round_piece_transform.getRotation().getX(),
                                                               this->round_piece_transform.getRotation().getY(),
                                                               this->round_piece_transform.getRotation().getZ(),
                                                               this->round_piece_transform.getRotation().getW()).normalize());

    this->straight_piece_transform.child_frame_id_ = this->child_frame_straight;
    this->straight_piece_transform.frame_id_ = this->frame_id;
    this->straight_piece_transform.setOrigin(tf::Vector3(0.570, 0.120, 0.697));
    this->straight_piece_transform.setRotation(tf::Quaternion(0.512, -0.505, -0.489, -0.493).normalize());

    this->round_piece_pre_transform.child_frame_id_ = this->pre_frame_string + this->child_frame_straight;
    this->round_piece_pre_transform.frame_id_ = this->frame_id;
    this->round_piece_pre_transform.setOrigin(tf::Vector3(this->straight_piece_transform.getOrigin().getX(),
                                                          this->straight_piece_transform.getOrigin().getY(),
                                                          this->straight_piece_transform.getOrigin().getZ() + z_offset));
    this->round_piece_pre_transform.setRotation(tf::Quaternion(this->straight_piece_transform.getRotation().getX(),
                                                               this->straight_piece_transform.getRotation().getY(),
                                                               this->straight_piece_transform.getRotation().getZ(),
                                                               this->straight_piece_transform.getRotation().getW()).normalize());

    this->tf_transform_vector.resize(4);

    this->tf_transform_vector[0] = this->round_piece_pre_transform;
    this->tf_transform_vector[1] = this->round_piece_transform;
    this->tf_transform_vector[2] = this->straight_piece_pre_transform;
    this->tf_transform_vector[3] = this->straight_piece_transform;

}
