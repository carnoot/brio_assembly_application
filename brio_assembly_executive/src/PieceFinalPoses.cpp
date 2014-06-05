#include <PieceFinalPoses.h>

PieceFinalPoses::PieceFinalPoses(){

    z_offset = 0.15;

    this->frame_id = "base_link";
    this->pre_round_child_frame_string = "pre_round_piece_dest";
    this->pre_straight_child_frame_string = "pre_straight_piece_dest";
    this->child_frame_straight = "straight_piece_dest";
    this->child_frame_round = "round_piece_dest";

    double x,y,z,x1,y1,z1;
    double q_x, q_y, q_z, q_w, q_x1, q_y1, q_z1, q_w1;

    x = 0.677;
    y = -0.097;
    z = 0.827;

    q_x = 0.460;
    q_y = 0.545;
    q_z = -0.416;
    q_w = 0.564;

    x1 = 0.701;
    y1 = -0.267;
    z1 = 0.828;

    q_x1 = 0.561;
    q_y1 = 0.452;
    q_z1 = -0.475;
    q_w1 = 0.505;

    this->round_piece_transform.child_frame_id_ = this->child_frame_round;
    this->round_piece_transform.frame_id_ = this->frame_id;
    this->round_piece_transform.setOrigin(tf::Vector3(x1, y1, x1));
    this->round_piece_transform.setRotation(tf::Quaternion(q_x1, q_y1, q_z1, q_w1).normalize());

    this->round_piece_pre_transform.child_frame_id_ = this->pre_round_child_frame_string;//this->pre_frame_string + this->child_frame_round;
    this->round_piece_pre_transform.frame_id_ = this->frame_id;
    this->round_piece_pre_transform.setOrigin(tf::Vector3(this->round_piece_transform.getOrigin().getX(),
                                                          this->round_piece_transform.getOrigin().getY(),
                                                          this->round_piece_transform.getOrigin().getZ()));

    this->round_piece_pre_transform.setRotation(tf::Quaternion(this->round_piece_transform.getRotation().getX(),
                                                               this->round_piece_transform.getRotation().getY(),
                                                               this->round_piece_transform.getRotation().getZ(),
                                                               this->round_piece_transform.getRotation().getW()).normalize());

    this->straight_piece_transform.child_frame_id_ = this->child_frame_straight;
    this->straight_piece_transform.frame_id_ = this->frame_id;
    this->straight_piece_transform.setOrigin(tf::Vector3(x, y, z));
    this->straight_piece_transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w).normalize());

    this->straight_piece_pre_transform.child_frame_id_ = this->pre_straight_child_frame_string;//this->pre_frame_string + this->child_frame_straight;
    this->straight_piece_pre_transform.frame_id_ = this->frame_id;
    this->straight_piece_pre_transform.setOrigin(tf::Vector3(this->straight_piece_transform.getOrigin().getX(),
                                                          this->straight_piece_transform.getOrigin().getY(),
                                                          this->straight_piece_transform.getOrigin().getZ()));

    this->straight_piece_pre_transform.setRotation(tf::Quaternion(this->straight_piece_transform.getRotation().getX(),
                                                               this->straight_piece_transform.getRotation().getY(),
                                                               this->straight_piece_transform.getRotation().getZ(),
                                                               this->straight_piece_transform.getRotation().getW()).normalize());

    this->tf_transform_vector.resize(4);

    this->tf_transform_vector[0] = this->round_piece_pre_transform;
    this->tf_transform_vector[1] = this->round_piece_transform;
    this->tf_transform_vector[2] = this->straight_piece_pre_transform;
    this->tf_transform_vector[3] = this->straight_piece_transform;

}
