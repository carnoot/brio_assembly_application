#include <TfPublisher.h>

TfPublisher::TfPublisher() {
    std::cout << "Constructor of thread class!" << std::endl;
    this->pre_string = "pre_";
    this->my_thread = new boost::thread(
                boost::bind(&TfPublisher::publishTransform, this));
    this->my_thread2 = new boost::thread(boost::bind(&TfPublisher::publishFinalTransform, this));
    this->can_publish = false;
}

TfPublisher::~TfPublisher() {
    std::cout << "Destructor of thread class!" << std::endl;
}

void TfPublisher::calculate_pre_transform(){

    tfScalar z_offset = 0.15;

    this->pre_transform.stamp_ = ros::Time::now();

    this->pre_transform.setOrigin(tf::Vector3(this->transform.getOrigin().getX(),
                                              this->transform.getOrigin().getY(),
                                              this->transform.getOrigin().getZ() + z_offset));

    this->pre_transform.setRotation(tf::Quaternion(this->transform.getRotation().getX(),
                                                   this->transform.getRotation().getY(),
                                                   this->transform.getRotation().getZ(),
                                                   this->transform.getRotation().getW()));
    this->pre_transform.frame_id_ = this->transform.frame_id_;

    this->pre_transform.child_frame_id_ = this->pre_string + this->transform.child_frame_id_;

}

void TfPublisher::publishTransform(){
    std::cout << "Publishing TRANSFORMS" << std::endl;
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

void TfPublisher::publishFinalTransform(){
    std::cout << "Publishing FINAL PIECE TRANSFORM" << std::endl;
    ros::Rate r(30);

    while(ros::ok()){

        for (int a = 0; a < this->final_poses.tf_transform_vector.size(); a++){
            tf::StampedTransform transform;
            transform = this->final_poses.tf_transform_vector[a];
            transform.stamp_ = ros::Time::now();
            this->br2.sendTransform(this->transform);
}
        r.sleep();
    }
}
