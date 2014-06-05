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

    std::cerr << "CALCULATE PRE TRANSFORM" << std::endl;

    tfScalar z_offset = -0.25;

    this->pre_transform.stamp_ = ros::Time::now();

    this->pre_transform.setOrigin(tf::Vector3(this->transform.getOrigin().getX(),
                                              this->transform.getOrigin().getY(),
                                              this->transform.getOrigin().getZ()));

    this->pre_transform.setRotation(tf::Quaternion(this->transform.getRotation().getX(),
                                                   this->transform.getRotation().getY(),
                                                   this->transform.getRotation().getZ(),
                                                   this->transform.getRotation().getW()).normalize());

    std::cout << "this->transform.frame_id: " << this->transform.frame_id_ << std::endl;
    std::cout << "this->transform.child_frame_id: " << this->transform.child_frame_id_ << std::endl;

    this->pre_transform.frame_id_ = this->transform.frame_id_;

    this->pre_transform.child_frame_id_ = this->pre_string + this->transform.child_frame_id_;

}

void TfPublisher::publishTransform(){
    std::cout << "Publishing TRANSFORMS" << std::endl;
    ros::Rate r(30);

    while(ros::ok()){
        this->mut.lock();

//        std::cerr << "CHILD FRAME ID:" << this->pre_transform.child_frame_id_ << std::endl;
//        std::cerr << "FRAME ID:" << this->pre_transform.frame_id_ << std::endl;

        this->transform.stamp_ = ros::Time::now();
        if (this->can_publish){

            std::cerr << "CHILD FRAME ID:" << this->pre_transform.child_frame_id_ << std::endl;
            std::cerr << "FRAME ID:" << this->pre_transform.frame_id_ << std::endl;

            this->calculate_pre_transform();

            std::cerr << "CHILD FRAME ID AFTER CALCULATE:" << this->pre_transform.child_frame_id_ << std::endl;
            std::cerr << "FRAME ID AFTER CALCULATE:" << this->pre_transform.frame_id_ << std::endl;

            if (this->transform.child_frame_id_ != "" && this->transform.frame_id_ != ""){
                ROS_INFO("ACUM PUBLIC!!!!!!!!!!!!!!!!!!");
            this->br.sendTransform(this->transform);
//            this->br.sendTransform(this->pre_transform);
        }
            if (this->pre_transform.child_frame_id_ != "" && this->pre_transform.frame_id_ != ""){
                this->br.sendTransform(this->pre_transform);
            }
        }

        this->mut.unlock();
        r.sleep();
    }
}

void TfPublisher::publishFinalTransform(){
    ros::Rate r(30);

    while(ros::ok()){

//        std::cout << "Publishing FINAL PIECE TRANSFORM" << std::endl;

        for (int a = 0; a < this->final_poses.tf_transform_vector.size(); a++){
            tf::StampedTransform transform;
            transform = this->final_poses.tf_transform_vector[a];
            transform.stamp_ = ros::Time::now();
//            std::cerr << transform.child_frame_id_ << std::endl;
            if (transform.child_frame_id_ != "" && transform.frame_id_ != "")
            this->br2.sendTransform(transform);
}
        r.sleep();
    }
}
