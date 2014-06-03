#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <PieceFinalPoses.h>
#include <boost/thread.hpp>
#include <mutex>

#ifndef Tf_Publisher_H
#define Tf_Publisher_H

class TfPublisher {

public:
    std::string child_frame;
    std::string parent_frame;
    std::string pre_string;
    tf::StampedTransform transform;
    tf::StampedTransform pre_transform;
    tf::TransformBroadcaster br;
    tf::TransformBroadcaster br2;
    PieceFinalPoses final_poses;
    int shared_variable;
    bool can_publish;

public:

    boost::thread *my_thread;
    boost::thread *my_thread2;

    std::mutex mut;
    TfPublisher();
    ~TfPublisher();
    void publishTransform();
    void publishFinalTransform();
    void calculate_pre_transform();

};

#endif
