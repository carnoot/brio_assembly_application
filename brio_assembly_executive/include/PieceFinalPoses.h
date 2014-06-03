#include <iostream>
#include <tf/tf.h>

#ifndef PIECE_FINAL_POSES_H
#define PIECE_FINAL_POSES_H

class PieceFinalPoses{

public:

    std::string child_frame_round;
    std::string child_frame_straight;
    std::string frame_id;
    std::string pre_frame_string;

    tf::StampedTransform round_piece_pre_transform;
    tf::StampedTransform round_piece_transform;
    tf::StampedTransform straight_piece_pre_transform;
    tf::StampedTransform straight_piece_transform;

    std::vector<tf::StampedTransform> tf_transform_vector;

    tfScalar z_offset;

     PieceFinalPoses();
};

#endif
