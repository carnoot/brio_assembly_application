#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <stdio.h>
#include <mutex>
#include <ros/ros.h>
#include <brio_assembly_vision/TrasformStamped.h>
#include <tf/transform_broadcaster.h>

class thread_class {

public:
    std::string child_frame;
    std::string parent_frame;
    tf::StampedTransform transform;
    tf::TransformBroadcaster br;
    int shared_variable;

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
}

thread_class::~thread_class() {
    std::cout << "Destructor of thread class!" << std::endl;
}

void thread_class::publishTransform(){
    std::cout << "Publishing TRANSFORM" << std::endl;
    ros::Rate r(30);

    while(ros::ok()){
    this->mut.lock();
        this->br.sendTransform(this->transform);
//    this->br.sendTransform(this->transform, ros::Time::now(), this->parent_frame, this->child_frame);
    this->mut.unlock();
        r.sleep();
}
}

class Control{
public:
    std::mutex mutex;
    ros::NodeHandle n;
    ros::ServiceClient client;
    std::string serviceName;
    brio_assembly_vision::TrasformStamped stampedTransform;
    thread_class thread;
    Control();
    ~Control();
    void callVisionService();
    void copyTransformIntoThread();
};


Control::Control(){

    std::cout << "Setting up Service Client!" << std::endl;
    this->client = this->n.serviceClient<brio_assembly_vision::TrasformStamped>(
                "/brio_assembly_vision");
    this->serviceName = "Vision";
}

Control::~Control(){

}

void Control::callVisionService(){

    this->stampedTransform.request.serviceName = this->serviceName;
    if (this->client.call(this->stampedTransform)) {
        std::cout << "Successfully returned from VISION Service call!" << std::endl;
        this->thread.mut.lock();
        this->copyTransformIntoThread();
        this->thread.mut.unlock();
    }
    else{
        std::cout << "VISION Service call FAILED!" << std::endl;
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

    this->thread.child_frame = this->stampedTransform.response.msg.child_frame_id;

    this->thread.parent_frame = this->stampedTransform.response.msg.header.frame_id;

    this->thread.transform.frame_id_ = this->thread.parent_frame;

    this->thread.transform.child_frame_id_ = this->thread.child_frame;

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "CONTROL");
    Control contr;
    while (1){
        contr.callVisionService();
    }


}

