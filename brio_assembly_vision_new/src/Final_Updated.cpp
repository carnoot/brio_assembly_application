#include<iostream>
#include <vector>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen3/Eigen/src/Core/MatrixBase.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

#include <brio_assembly_vision_new/TrasformStamped.h>
#include </home/stefan/ros_ws/src/brio_assembly_application/brio_assembly_vision_new/include/brio_assembly_vision_new/DepthImageProjection.h>
#include <brio_assembly_vision_new/Is_This_Easier.h>

#define TOPIC_DEPTH  "/kinect_head/depth_registered/image_raw"
#define TOPIC_COLOR "/kinect_head/rgb/image_color"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr cloud (new PointCloud);
ros::ServiceClient client ;
brio_assembly_vision_new::Container * client_call();
ros::Publisher pub;
bool request_a_new_cluster=true,final=false;
bool find_cloud_from_kinect_head =false,color_find=false,depth_find=false,first_time=true;
int initial_cluster_size=0;

Eigen::Matrix4d transformata_finala;
Eigen::Quaternionf createQuaternion();
Eigen::Vector3d estimate_plane_normals(PointCloud::Ptr cloud_f);
Eigen::Matrix4d calculate_transformation(Eigen::Vector3d first_vector, Eigen::Vector3d second_vector,Eigen::Vector4d);
std::vector<std::string> model;
int model_step=0;
cv::Mat color,depth;

void image_cb (const sensor_msgs::Image::ConstPtr  pRosColor)
{
    if(request_a_new_cluster==true){
        cv::Mat color_stream;
        cv_bridge::CvImageConstPtr pCvColor = cv_bridge::toCvShare(pRosColor, sensor_msgs::image_encodings::BGR8);
        pCvColor->image.copyTo(color_stream);
        color=color_stream;
        color_find=true;
    }
}

void cloud_cb (const sensor_msgs::Image::ConstPtr  pRosDepth)
{
    if(request_a_new_cluster==true)
    {
        cv::Mat  depth_stream;
        cv_bridge::CvImageConstPtr pCvDepth = cv_bridge::toCvShare(pRosDepth, sensor_msgs::image_encodings::TYPE_16UC1);
        pCvDepth->image.copyTo(depth_stream);
        cv::Size size(1280, 1024);
        int offset = 0;
        cv::Mat tmp = cv::Mat::zeros(size, CV_16U), roi = tmp(cv::Rect(0, offset, 1280, 960));
        cv::resize(depth_stream, roi, cv::Size(1280, 960), 0, 0, cv::INTER_NEAREST);
        depth_stream = tmp;
        depth=depth_stream;
        depth_find=true;
    }
}

void write_cloud(const sensor_msgs::Image::ConstPtr  pRosColor)
{
    if(request_a_new_cluster==true)
    {
        if(color_find==true  && depth_find==true && find_cloud_from_kinect_head==false){
            ROS_INFO("CALCULATE TRANSFORMATION FOR A NEW CLUSTER");
            iai_rs::rs_pcl::DepthImageProjection projection;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud;
            pCloud = projection.project(depth,color);
            if(pCloud->size()>0){
                //pcl::io::savePCDFileASCII ("test_pcd.pcd", *pCloud);
                *cloud=*pCloud;
                find_cloud_from_kinect_head=true;
                request_a_new_cluster=false;
                color_find=false;
                depth_find=false;
            }
        }
    }
}

Eigen::Vector3d estimate_plane_normals(PointCloud::Ptr cloud_f)
{
    PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_f_aux (new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud::Ptr cloud_filtered_while (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_f);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.1);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (2);
    sor.filter (*cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    cloud_filtered_while =cloud_filtered;
    int i = 0, nr_points = (int) cloud_filtered_while->points.size ();
    // While 30% of the original cloud is still there
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    while (cloud_filtered_while->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered_while);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the planar inliers from the input cloud
        extract.setInputCloud (cloud_filtered_while);
        extract.setIndices (inliers);
        // Remove the planar inliers, extract the rest
        extract.setNegative (false);
        extract.filter (*cloud_plane);
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f_aux);
        cloud_filtered_while.swap (cloud_f_aux);
        i++;
    }

    Eigen::Vector3d plane_normal_vector ;
    for(int i=0;i<3;i++)
        plane_normal_vector(i) = coefficients->values[i];

    return plane_normal_vector;
}

Eigen::Vector3d obj_axe_2_points_next(int i_head,int i_center){
    pcl::PointXYZRGB center,head;
    center = cloud->at(i_center); //x1 y1 z1
    head = cloud->at(i_head); //x2 y2 z2
    Eigen::Vector3d vector_l;
    vector_l(0)=5*(head.x - center.x);
    vector_l(1)=5*(head.y - center.y);
    vector_l(2)=5*(head.z - center.z);

    return vector_l;
}


Eigen::Matrix4d calculate_transformation(Eigen::Vector3d x_or_y_axe_vector, Eigen::Vector3d z_axe_vector, Eigen::Vector4d xyz_centroid)
{
    transformata_finala=Eigen::MatrixXd::Identity(4,4);
    Eigen::Vector3d axa3;
    axa3(0)=x_or_y_axe_vector(1)*z_axe_vector(2)-z_axe_vector(1)*x_or_y_axe_vector(2);//  a_nou = b1*c2 - b2*c1;
    axa3(1)=z_axe_vector(0)*x_or_y_axe_vector(2)-x_or_y_axe_vector(0)*z_axe_vector(2);//  b_nou = a2*c1 - a1*c2;
    axa3(2)=x_or_y_axe_vector(0)*z_axe_vector(1)-x_or_y_axe_vector(1)*z_axe_vector(0);//  c_nou = a1*b2 - b1*a2;
    for(int i=0;i<3;i++)
    {
        transformata_finala(i,0) =z_axe_vector(i);
        transformata_finala(i,1) =axa3(i);
        transformata_finala(i,2) =x_or_y_axe_vector(i);
    }

    for(int i=0;i<3;i++)
        transformata_finala(i,3)=xyz_centroid(i);

    std::cout<<std::endl<<"Transformation Matrix:"<<std::endl<<transformata_finala<<std::endl;

    float x0,y0,z0;
    x0=xyz_centroid[0];
    y0=xyz_centroid[1];
    z0=xyz_centroid[2];

    float l = 0.005;
    pcl::PointXYZRGB p_x,p_y,p_z;
    p_x.r=255;
    p_x.g=0;
    p_x.b=0;

    p_y.r=0;
    p_y.g=255;
    p_y.b=0;

    p_z.r=0;
    p_z.g=0;
    p_z.b=255;

    PointCloud::Ptr axe (new PointCloud);
    for(int i=0;i<100;i++)
    {

        //axa x red
        p_x.x =x0 + transformata_finala(0,0)*l;
        p_x.y =y0 + transformata_finala(1,0)*l;
        p_x.z =z0 + transformata_finala(2,0)*l;

        //axa y green
        p_y.x =x0 + transformata_finala(0,1)*l;
        p_y.y =y0 + transformata_finala(1,1)*l;
        p_y.z =z0 + transformata_finala(1,1)*l;

        //axa z blue
        p_z.x =x0 + transformata_finala(0,2)*l;
        p_z.y =y0 + transformata_finala(1,2)*l;
        p_z.z =z0 + transformata_finala(2,2)*l;

        axe->push_back(p_x);
        axe->push_back(p_y);
        axe->push_back(p_z);

        l+=0.005;
    }

    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    viewer.addPointCloud (cloud, "plane",0);
    viewer.addPointCloud (axe, "a",0);
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }

}

Eigen::Quaternionf createQuaternion(){
    Eigen::Matrix3f rotation;
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++)
            rotation(a,b) = transformata_finala(a,b);
    Eigen::Quaternionf quat (rotation);
    return quat;
}

brio_assembly_vision_new::Container * client_call(){
    brio_assembly_vision_new::Is_This_Easier srv;
    brio_assembly_vision_new::Container * cont = new brio_assembly_vision_new::Container();
    if (client.call(srv)) //call the service
    {
        int size=srv.response.get_message.date_container.size();
        if(size!=0){
            //use data
            for(int i=0;i<size;i++)
            {
                (*cont).date_container.push_back(srv.response.get_message.date_container.at(i));
            }

            ROS_INFO("Receved data. Number of clusters : %d",size);
            return cont;
        }
        //empty resonse
        ROS_ERROR("Failed to call OpenCV service , reason : RESPONSE IS EMPTY , retrying");
        return NULL;
    }
    else
    {
        ROS_ERROR("Failed to call OpenCV service , retrying");
        return NULL;
    }
}

bool send2(brio_assembly_vision_new::TrasformStampedRequest  &req, brio_assembly_vision_new::TrasformStampedResponse &res)
{
    request_a_new_cluster =true;
    if(find_cloud_from_kinect_head==true && final ==false)
    {
        brio_assembly_vision_new::Container * cont = new brio_assembly_vision_new::Container();
        while((cont=client_call())==NULL)
        {
            std::this_thread::sleep_for (std::chrono::seconds(1)); //wait for a good response
        }

        if(first_time==true)
        {
            initial_cluster_size=cont->date_container.size();
        }
        first_time=false;

        if(cont->date_container.size()>0) //after moving objects size is decreased
        {
            int i=0; //start with the lowest index
            std::string object_with_shape_requested = model[model_step];
            while(cont->date_container[i].piece_type!=object_with_shape_requested && i<cont->date_container.size()-1) //
            {
                i++;
            }
            //end while when cluster_vector[i] has the requested shape or when the search index is bigger then cluster_vector
            if(i<=cont->date_container.size()-1) // if i<=cluster_vector.size() then we did find requested object at indices i
            {
                Eigen::Vector3d z_axe,x_or_y_axe;
                Eigen::Vector4d translation;
                z_axe=estimate_plane_normals(cloud);
                int i_center = cont->date_container[i].center_index;
                int i_head = cont->date_container[i].head_conn_index;
                x_or_y_axe= obj_axe_2_points_next(i_head,i_center);
                pcl::PointXYZRGB center = cloud->at(cont->date_container[i].center_index);
                translation[0] = center.x;
                translation[1] = center.y;
                translation[2] = center.z;

                calculate_transformation(x_or_y_axe,z_axe ,translation);

                ROS_INFO("Send TransformedPose");
                Eigen::Quaternionf quat;
                quat = createQuaternion();
                res.msg.child_frame_id = "brio_piece_frame";
                res.msg.header.frame_id = "head_mount_kinect_rgb_optical_frame";
                res.msg.transform.translation.x = transformata_finala(0,3);
                res.msg.transform.translation.y = transformata_finala(1,3);
                res.msg.transform.translation.z = transformata_finala(2,3);

                res.msg.transform.rotation.w = (double)quat.w();
                res.msg.transform.rotation.x = (double)quat.x();
                res.msg.transform.rotation.y = (double)quat.y();
                res.msg.transform.rotation.z = (double)quat.z();

                if(model_step<initial_cluster_size-1)
                    model_step++;
                else
                {   final=true;
                    //return false;
                }

                return true;
            }
            else
                return false;
        }
        else return false;
    }
    else
        return false;

}

int main (int argc, char** argv)
{
    model.push_back("piesa3");
    model.push_back("piesa2");
    model.push_back("piesa1");
    model.push_back("piesa4");
    model.push_back("piesa5");
    // Initialize ROS
    ros::init (argc, argv, "pcl_node");
    ros::NodeHandle nh;
    ROS_INFO("Hello world");
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_color = nh.subscribe (TOPIC_COLOR, 1, image_cb);
    ros::Subscriber sub_depth = nh.subscribe (TOPIC_DEPTH, 1, cloud_cb);
    ros::Subscriber sub_write = nh.subscribe (TOPIC_DEPTH, 1, write_cloud);

    ros::ServiceServer service1 = nh.advertiseService("/brio_assembly_vision", send2);
    ros::ServiceClient client_p = nh.serviceClient <brio_assembly_vision_new::Is_This_Easier>("/brio_assembly_vision_container");
    client=client_p;

    ros::spin();

    return 0;
}
