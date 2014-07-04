#include<iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h> //hydro
#include <sensor_msgs/PointCloud2.h> //hydro
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/filesystem.hpp>
#include <pcl/filters/filter.h>
#include <eigen3/Eigen/src/Core/MatrixBase.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include<pcl/common/centroid.h>
#include <brio_assembly_vision/TrasformStamped.h>
#include<brio_assembly_vision/Finish_Movement.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include<pcl/sample_consensus/sac_model_line.h>
#include<pcl/sample_consensus/sac_model_circle.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <complex>      // std::complex, std::real
#include <pcl/filters/radius_outlier_removal.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
<<<<<<< Updated upstream
#include <brio_assembly_vision/DepthImageProjection.h>

ros::Publisher pub;
bool request_a_new_cluster=true,final=false;
bool find_cloud_from_kinect_head =false,color_find=false,depth_find=false,first_time=true;
int initial_cluster_size=0;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr cloud (new PointCloud);
std::string ransac_detect(PointCloud cluster_to_detect);

Eigen::Matrix4d transformata_finala;
=======
#include </home/stefan/ros_ws/src/brio_assembly_application/brio_assembly_vision/include/brio_assembly_vision/DepthImageProjection.h>
#include <brio_assembly_vision/Get_Info_From_OpenCV.h>
#include <brio_assembly_vision/Is_This_Easier.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
>>>>>>> Stashed changes


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
Eigen::Vector4d calculate_centroid(PointCloud cluster_cloud);
class Clusters
{
public:
    PointCloud cluster_point_cloud;
    std::string type;
    Eigen::Vector4d centroid_xyz;
    int index_centroid,index_head,index_back;
    //Clusters();

    void compute_centroid()
    {
        this->centroid_xyz = calculate_centroid(cluster_point_cloud);

    }
};

PointCloud::Ptr cloud (new PointCloud);
ros::ServiceClient client ;
brio_assembly_vision::Container * client_call();
ros::Publisher pub;
bool request_a_new_cluster=true,final=false;
bool find_cloud_from_kinect_head =false,color_find=false,depth_find=false,first_time=true;
int initial_cluster_size=0;

std::string ransac_detect(PointCloud cluster_to_detect);

Eigen::Matrix4d transformata_finala;

Eigen::Quaternionf createQuaternion();
//std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_extraction(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);

Eigen::Vector3d calculate_one_object_axe (PointCloud cloud_cluster);
Eigen::Vector3d estimate_plane_normals(PointCloud::Ptr cloud_f);
Eigen::Matrix4d calculate_transformation(Eigen::Vector3d first_vector, Eigen::Vector3d second_vector,Eigen::Vector4d);

int find_point_index(Eigen::Vector4d centroid);
std::vector<Clusters> cluster_extraction(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_to_extract);
std::vector<std::string> model;
int model_step=0;
//Clusters cluster_extraction(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);
cv::Mat color,depth;


void image_cb (const sensor_msgs::Image::ConstPtr  pRosColor)
{
    if(request_a_new_cluster==true){
        cv::Mat color_stream;
        cv_bridge::CvImageConstPtr pCvColor = cv_bridge::toCvShare(pRosColor, sensor_msgs::image_encodings::BGR8);
        pCvColor->image.copyTo(color_stream);
        color=color_stream;
        //        std::vector<int>
        //                params;
        //        params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        //        params.push_back(9);
        //cv::imwrite("image_color.png", color);
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

        //        std::vector<int>
        //                params;
        //        params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        //        params.push_back(9);
        //        cv::imwrite("img_depth.png", depth, params);

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
            pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
            pCloud = projection.project(depth,color);
            cloud_filtered = *pCloud;

            if(cloud_filtered.size()>0){
                for(int i=0;i<10;i++)
                {
                    cloud_filtered.at(622411+i*1280).r=255; //red
                    cloud_filtered.at(622411+i*1280).g=0;
                    cloud_filtered.at(622411+i*1280).b=0;

                    cloud_filtered.at(631484+i*1280).r=0; //green
                    cloud_filtered.at(631484+i*1280).g=255;
                    cloud_filtered.at(631484+i*1280).b=0;

                    cloud_filtered.at(602259+i*1280).r=0; //blue
                    cloud_filtered.at(602259+i*1280).g=0;
                    cloud_filtered.at(602259+i*1280).b=255;

                    cloud_filtered.at(624149+i*1280).r=255; //yellow
                    cloud_filtered.at(624149+i*1280).g=255;
                    cloud_filtered.at(624149+i*1280).b=0;

                    cloud_filtered.at(620084+i*1280).r=0; //azur
                    cloud_filtered.at(620084+i*1280).g=255;
                    cloud_filtered.at(620084+i*1280).b=255;
                }
                pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud_filtered);
                *cloud=cloud_filtered;
                find_cloud_from_kinect_head=true;
                request_a_new_cluster=false;
                color_find=false;
                depth_find=false;
            }
        }
    }

}


//void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
//{
//    //ROS_INFO("I HAVE IMAGE ON TOPIC FROM KINECT");

//    if(request_a_new_cluster==true)
//    {
//        ROS_INFO("CALCULATE TRANSFORMATION FOR CLUSTER");
//        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
//        pcl::fromROSMsg(*input, cloud_filtered);

//        if(cloud_filtered.size()>0){
//            pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud_filtered);

//            *cloud=cloud_filtered;
//            find_cloud_from_kinect_head=true;
//            /*
//                        std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_vector;
//                        cluster_vector=cluster_extraction(cloud);
//                        PointCloud current_cluster = cluster_vector[0];
//                        Eigen::Vector3d z_axe,x_or_y_axe;
//                        Eigen::Vector4d translation;
//                        z_axe=estimate_plane_normals(cloud);
//                        x_or_y_axe=calculate_one_object_axe(current_cluster);
//                        translation =calculate_centroid(current_cluster);
//                        calculate_transformation(x_or_y_axe,z_axe ,translation);
//                        ransac_detect(current_cluster);
//            */
//            request_a_new_cluster=false;
//        }
//    }
//    //pub.publish(input);
//}


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



    //    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    //    vg.setInputCloud (cloud_filtered);
    //    vg.setLeafSize (0.002f, 0.002f, 0.002f);
    //    vg.filter (*cloud_filtered);

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
    //seg.setInputCloud (cloud_filtered);
    //seg.segment (*inliers, *coefficients);
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
        //    extract.setNegative (false);
        //    // Get the points associated with the planar surface
        //    extract.filter (*cloud_plane);
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        // Remove the planar inliers, extract the rest
        extract.setNegative (false);
        extract.filter (*cloud_plane);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f_aux);
        cloud_filtered_while.swap (cloud_f_aux);
        i++;
    }


    /*
    Eigen::Vector4d xyz_centroid;
    // Estimate the XYZ centroid
    pcl::compute3DCentroid(*cloud_plane, xyz_centroid);

    float x0,y0,z0;
    x0=xyz_centroid[0];
    y0=xyz_centroid[1];
    z0=xyz_centroid[2];

    float l = 0.01;
    pcl::PointXYZRGB p_x,p_y,p_z;
    p_x.r=255;
    p_x.g=0;
    p_x.b=0;
    PointCloud::Ptr axe (new PointCloud);
    for(int i=0;i<100;i++)
    {
        p_x.x =x0 + coefficients->values[0]*l;
        p_x.y =y0 + coefficients->values[1]*l;
        p_x.z =z0 + coefficients->values[2]*l;

        l+=0.01;
        axe->push_back(p_x);
    }

    std::cout<<"axe size"<<axe->size()<<std::endl;

    pcl::visualization::PCLVisualizer viewer ("Plane Normal Vector");
    int v1(0); int v2(1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    //viewer.addCoordinateSystem(0.1,0);
    viewer.addPointCloud (cloud_f, "cloud",v1);
    viewer.addPointCloud (cloud_plane, "plane",v2);
    viewer.addPointCloud (axe, "a",v2);
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }

*/
    //std::cout<<"Plane Coefficients:"<<std::endl<<*coefficients<<std::endl;
    Eigen::Vector3d plane_normal_vector ;
    for(int i=0;i<3;i++)
        plane_normal_vector(i) = coefficients->values[i];
    //std::cout<<"Plane Coefficients:"<<std::endl<<plane_normal_vector<<std::endl;

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


//        float l = 0.001;
//        pcl::PointXYZRGB p_x ;
//        p_x.r=255;
//        p_x.g=0;
//        p_x.b=0;
//        PointCloud::Ptr axe (new PointCloud);
//        for(int i=0;i<100;i++)
//        {
//            p_x.x =center.x  + vector_l(0)*l;
//            p_x.y =center.y  + vector_l(1)*l;
//            p_x.z =center.z  + vector_l(2)*l;
//            l+=0.005;
//            axe->push_back(p_x);

//        }

//        pcl::visualization::PCLVisualizer viewer ("Object CLuster Axe");
//        viewer.addPointCloud (cloud, "cloud",0);
//        viewer.addPointCloud (axe, "a",0);
//        while (!viewer.wasStopped ()) {
//            viewer.spinOnce ();
//        }
return vector_l;
}

Eigen::Vector3d obj_axe_2_points(Clusters cluster_curent){
        pcl::PointXYZRGB center,head;
        center = cloud->at(cluster_curent.index_centroid); //x1 y1 z1
        head = cloud->at(cluster_curent.index_head); //x2 y2 z2
     Eigen::Vector3d vector_l;
        vector_l(0)=5*(head.x - center.x);
        vector_l(1)=5*(head.y - center.y);
        vector_l(2)=5*(head.z - center.z);


        float l = 0.001;
        pcl::PointXYZRGB p_x ;
        p_x.r=255;
        p_x.g=0;
        p_x.b=0;
        PointCloud::Ptr axe (new PointCloud);
        for(int i=0;i<100;i++)
        {
            p_x.x =center.x  + vector_l(0)*l;
            p_x.y =center.y  + vector_l(1)*l;
            p_x.z =center.z  + vector_l(2)*l;
            l+=0.005;
            axe->push_back(p_x);

        }

//        pcl::visualization::PCLVisualizer viewer ("Object CLuster Axe");
//        viewer.addPointCloud (cloud, "cloud",0);
//        viewer.addPointCloud (axe, "a",0);
//        while (!viewer.wasStopped ()) {
//            viewer.spinOnce ();
//        }
return vector_l;
}



Eigen::Vector3d calculate_one_object_axe (PointCloud cloud_cluster)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_aux=cloud_cluster;

    // Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3d covariance_matrix,result;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4d xyz_centroid;
    // Estimate the XYZ centroid
    pcl::compute3DCentroid(*cloud_aux, xyz_centroid);

    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix(*cloud_aux, xyz_centroid, covariance_matrix);

    //std::cout<<std::endl<<"Centroid"<<std::endl << xyz_centroid << std::endl;

    //std::cout<<std::endl<<"Covariance Matrix"<<std::endl << covariance_matrix << std::endl;

    Eigen::EigenSolver <Eigen::Matrix3d> es(covariance_matrix);
    //std::cout<<std::endl<<"Eigen vectors:"<<std::endl<<es.pseudoEigenvectors()<<std::endl;
    result=es.pseudoEigenvectors();
    //    std::cout<<std::endl<<"Eigen values:"<<std::endl<<es.eigenvalues()<<std::endl;
    Eigen::VectorXcd eivals = es.eigenvalues();
    //std::cout<<std::endl<<"Eigen values:"<<std::endl<<eivals<<std::endl;

    int j=0;
    double max;
    //max = eivals(0,0);
    max = eivals(0).real();
    for(int i=1;i<3;i++)
    {
        if(max < eivals(i).real())
            j=i;
    }
    Eigen::Vector3d vector_1 ;
    for(int i=0;i<3;i++)
        vector_1(i)=result(i,j);


    float x0,y0,z0;
    x0=xyz_centroid[0];
    y0=xyz_centroid[1];
    z0=xyz_centroid[2];

    float l = 0.001;
    pcl::PointXYZRGB p_x ,p_x_m;
    p_x.r=255;
    p_x.g=0;
    p_x.b=0;

    p_x_m.r=0;
    p_x_m.g=255;
    p_x_m.b=0;
    PointCloud::Ptr axe (new PointCloud);
    for(int i=0;i<100;i++)
    {
        p_x.x =x0 + vector_1(0)*l;
        p_x.y =y0 + vector_1(1)*l;
        p_x.z =z0 + vector_1(2)*l;

        p_x_m.x =x0 - vector_1(0)*l;
        p_x_m.y =y0 - vector_1(1)*l;
        p_x_m.z =z0 - vector_1(2)*l;

        l+=0.001;
        axe->push_back(p_x);
        axe->push_back(p_x_m);

        if(i==99)

        {
            std::cerr<<"red xyz: "<<p_x.x<<" "<<p_x.y<<" "<<p_x.z<<std::endl;
            std::cerr<<"green xyz: "<<p_x_m.x<<" "<<p_x_m.y<<" "<<p_x_m.z<<std::endl;

            if(p_x_m.y<p_x.y)
                vector_1*=-1;
        }

    }

//    //std::cout<<"axe size"<<axe->size()<<std::endl;


//    pcl::visualization::PCLVisualizer viewer ("Object CLuster Axe");
//    int v1(0); int v2(1);
//    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//   viewer.addCoordinateSystem(0.9,0);
//    viewer.addPointCloud (cloud_aux, "cloud",v1);
//    viewer.addPointCloud (cloud, "cluster",v2);
//    viewer.addPointCloud (axe, "a",v2);
//    while (!viewer.wasStopped ()) {
//        viewer.spinOnce ();
//    }


    return vector_1;

}


Eigen::Vector4d calculate_centroid(PointCloud cluster_cloud)
{
    Eigen::Vector4d xyz_centroid;
    // Estimate the XYZ centroid
    pcl::compute3DCentroid(cluster_cloud, xyz_centroid);
    return xyz_centroid;
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
    int v1(0); int v2(1);
    //    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    //    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    //viewer.addCoordinateSystem(0.1,0);
    // viewer.addPointCloud (cloud_f, "cloud",v1);
    viewer.addPointCloud (cloud, "plane",0);
    viewer.addPointCloud (axe, "a",0);
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }

}
//std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_extraction(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud)
std::vector<Clusters> cluster_extraction(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB> );
    PointCloud::Ptr cloud_filtered (new PointCloud);
    //    pcl::PCDWriter writer;
    //    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZRGB>);
    //Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.0015f, 0.0015f, 0.0015f); //0.0015f toate cele 3
    vg.filter (*cloud_filtered);
    //*cloud_filtered=*cloud;

    //   pcl::PassThrough<pcl::PointXYZRGB> pass1;
    //   pass1.setInputCloud(cloud);
    //   pass1.setFilterFieldName("x");
    //   pass1.setFilterLimits(0.0, 0.2);
    //   pass1.filter(*cloud_filtered);
    //   *cloud=*cloud_filtered;

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    //    pass.setInputCloud (cloud);
    //    pass.setFilterFieldName ("x");
    //    pass.setFilterLimits (-0.9, 0.9);
    //    //pass.setFilterLimitsNegative (true);
    //    pass.filter (*cloud_filtered);
    //*cloud_filtered=*cloud;

    //pcl::io::savePCDFileASCII ("x_fil.pcd", *cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.1);
    pass.filter (*indices);
    //pass.filter(*cloud_filtered);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud_filtered);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (30); //40
    //    reg.setPointColorThreshold (7); //color change , mic=>simte multe  schimbari de culoare
    //    reg.setRegionColorThreshold (30);
    //    reg.setPointColorThreshold (15); //color change , mic=>simte multe  schimbari de culoare
    //    reg.setRegionColorThreshold (30);
    reg.setPointColorThreshold (15); //color change , mic=>simte multe  schimbari de culoare //23

    reg.setRegionColorThreshold (0.1); //0.5

    //std::cout<<"Am crapat!!!!!!!!!!!!!!!!!!!!!!"<<std::endl<<std::endl<<std::endl;

<<<<<<< Updated upstream
    reg.setMinClusterSize (1600); //2000
    reg.setMaxClusterSize(6000);

=======
    reg.setMinClusterSize (1900);
    reg.setMaxClusterSize(15000);
>>>>>>> Stashed changes
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    //std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > pcd_vector;
    //Clusters cluster_vector;
    std::vector<Clusters> cluster_vector ;
    pcl::PCDWriter writer;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
    {
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        Clusters cloud_cluster;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster.cluster_point_cloud.points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster.cluster_point_cloud.width = cloud_cluster.cluster_point_cloud.points.size ();
        cloud_cluster.cluster_point_cloud.height = 1;
        cloud_cluster.cluster_point_cloud.is_dense = true;
        cloud_cluster.centroid_xyz=calculate_centroid(cloud_cluster.cluster_point_cloud);
        int indice = find_point_index(cloud_cluster.centroid_xyz);
        cloud_cluster.index_centroid=indice;
        std::cout<<"Centroidul pentru clusterul"<<j<<" este la indicele::"<<indice<<std::endl;
        std::cout<<"pct-ul are proprietatile:"<<cloud->at(indice)<<std::endl;
        //to do : match with opencv center indices

        cluster_vector.push_back(cloud_cluster);
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
        std::cerr<<"cluster"<<j<<" size"<< cloud_cluster.cluster_point_cloud.size()<<std::endl;
        j++;
    }

    // std::cout<<"Nmb of cluseters find:"<<pcd_vector.size()<<std::endl;
    std::cout<<"Nmb of cluseters find:"<<cluster_vector.size()<<std::endl;

//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//    viewer.showCloud (colored_cloud);
//    while (!viewer.wasStopped ())
//    {
//        boost::this_thread::sleep (boost::posix_time::microseconds (100));
//    }
    //return pcd_vector;
    return cluster_vector;
}

int find_point_index(Eigen::Vector4d point_coordinates)
{
    int ind=0;
    for(int i=0;i<cloud->size();i++)
        if( ( cloud->at(i).x-point_coordinates[0]<0.0015 && cloud->at(i).x-point_coordinates[0]>-0.0015 ) &&
            ( cloud->at(i).y-point_coordinates[1]<0.0015 && cloud->at(i).y-point_coordinates[1]>-0.0015 ) &&
            ( cloud->at(i).z-point_coordinates[2]<0.0015 && cloud->at(i).z-point_coordinates[2]>-0.0015 ) )
          {  ind = i; //this is the index for the cluster cinter in the big point cloud
                    return ind;
          }

      if(ind == 0) //if previous search dind't a match relax search parameters
          for(int i=0;i<cloud->size();i++)
              if( ( cloud->at(i).x-point_coordinates[0]<0.0025 && cloud->at(i).x-point_coordinates[0]>-0.0025 ) &&
                  ( cloud->at(i).y-point_coordinates[1]<0.0025 && cloud->at(i).y-point_coordinates[1]>-0.0025 ) &&
                  ( cloud->at(i).z-point_coordinates[2]<0.0025 && cloud->at(i).z-point_coordinates[2]>-0.0025 ) )
                {  ind = i; //this is the index for the cluster cinter in the big point cloud
                          return ind;
                }

    return ind;
}



std::string ransac_detect(PointCloud cluster_to_detect){
    if(cluster_to_detect.size()>4000)
    {
        std::cerr<<"Am gasit Mare"<<std::endl;
        return "Mare";
    }
    if(cluster_to_detect.size()<2500)
    {
        std::cerr<<"Am gasit mic"<<std::endl;
        return "Mic";
    }


        // initialize PointClouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloud_cluster=cluster_to_detect;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls ;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;


        mls.setComputeNormals (false);
        // Set parameters
        mls.setInputCloud (cloud_cluster);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree_mls);
        mls.setSearchRadius (0.1);
        mls.setPolynomialOrder(2);
        mls.setSqrGaussParam(0.0025);
        // Reconstruct
        mls.process (*cloud_filtered);


        sor2.setInputCloud (cloud_filtered);
        sor2.setMeanK (100); //300
        sor2.setStddevMulThresh (2);
        sor2.filter (*cloud_cluster);
        *cloud_cluster = * cloud_filtered;

        mls.setInputCloud (cloud_cluster);
        mls.setPolynomialFit (false);
        mls.setSearchMethod (tree_mls);
        mls.setSearchRadius (0.1);
        mls.setPolynomialOrder(2);
        mls.setSqrGaussParam(0.0025);
        // Reconstruct
        mls.process (*cloud_filtered);
        cloud_cluster=cloud_filtered;


        std::vector<int> inliers,in_circle;

        pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model_line (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud_cluster));
        pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB>::Ptr model_circle (new pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB> (cloud_cluster));

        //Eigen::VectorXf coef;
        model_circle->setRadiusLimits(0.001,0.09); //0.09


        //model_circle->
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_line);
        //    ransac.setDistanceThreshold (0.0005); // here +- depends on quality of clouds
        ransac.setDistanceThreshold (0.008); // here +- depends on quality of clouds
        ransac.computeModel();
        ransac.getInliers(inliers);


        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac2 (model_circle);
        //    ransac2.setDistanceThreshold (0.0005); //here +- depends on quality of clouds
        ransac2.setDistanceThreshold (0.008); // here +- depends on quality of clouds
        ransac2.computeModel();
        ransac2.getInliers(in_circle);

        //model_circle->computeModelCoefficients(in_circle,coef);

        std::cout<<"inliers line size="<<inliers.size()<<std::endl;
        std::cout<<"inliers circle size="<<in_circle.size()<<std::endl;

//        pcl::visualization::PCLVisualizer viewer ("ICP demo");
//        int v1(0); int v2(1);
//        viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//        viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

        if(inliers.size()>=in_circle.size())
        { // copies all inliers of the model computed to another PointCloud
            pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_cluster, inliers, *final);
            std::cout<<"Am gasit linie"<<std::endl;

//            viewer.addPointCloud (cloud_cluster, "cloud",v1);
//            viewer.addPointCloud (final, "final",v2);
//            while (!viewer.wasStopped ())
//            {
//                viewer.spinOnce ();
//            }

            return "Linie";
        }
        else
        {
            pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_cluster, in_circle, *final);
            std::cout<<"Am gasit semicerc"<<std::endl;

//            viewer.addPointCloud (cloud_cluster, "cloud",v1);
//            viewer.addPointCloud (final, "final",v2);

//            while (!viewer.wasStopped ())
//            {
//                viewer.spinOnce ();
//            }

            return "Semicerc";
        }
    }



void sort_by_center_index(std::vector<Clusters>& cluster_vector){
    Clusters clust_aux ;
     for(int i = 0; i< cluster_vector.size();i++)
     {
         for (int j=0;j<cluster_vector.size()-1;j++)
             if(cluster_vector[j].index_centroid > cluster_vector[j+1].index_centroid)
                {
                     clust_aux=cluster_vector[j+1];
                     cluster_vector[j+1]=cluster_vector[j];
                     cluster_vector[j]=clust_aux;
                     //cluster_vector[i].type = (*cont).date_container[j].piece_type;
                }
     }
}

void sort_by_center_index_opencv(brio_assembly_vision::Container * container){
    brio_assembly_vision::Data_Type aux;
     for(int i = 0; i< (*container).date_container.size();i++)
     {
         for (int j=0;j<(*container).date_container.size()-1;j++)
             if((*container).date_container[j].center_index > (*container).date_container[j+1].center_index)
                {
                     aux=(*container).date_container[j+1];
                     (*container).date_container[j+1]=(*container).date_container[j];
                     (*container).date_container[j]=aux;
                }
     }
}

bool cluster_center_match(std::vector<Clusters> &cluster_vector){
    //this function is called when we do cluster extraction
    //we check if the current cluster center can match with the ones get from opencv

    //get info from opencv
    brio_assembly_vision::Container * cont = new brio_assembly_vision::Container();
    while((cont=client_call())==NULL)
    {
        std::this_thread::sleep_for (std::chrono::seconds(1)); //wait for a good response
    }

    if(cluster_vector.size()!=cont->date_container.size())
        cluster_center_match(cluster_vector); // recall this function maybe we find a matching between 2 vectors
        // !!!!!!! TO DO : call only once then call again cluster extraction in order not to block the program !!

      //now we have opencv information, let's sort our clusters;
    sort_by_center_index(cluster_vector);
    sort_by_center_index_opencv(cont);

    std::cout<<std::endl<<"---------------------After---------------------"<<std::endl;

    for(int i = 0; i< cluster_vector.size();i++)
    {
        cluster_vector[i].type=(*cont).date_container[i].piece_type;
        cluster_vector[i].index_centroid=(*cont).date_container[i].center_index;
        cluster_vector[i].index_head=(*cont).date_container[i].head_conn_index;
        cluster_vector[i].index_back=(*cont).date_container[i].back_conn_index;
    //std::cout<<cluster_vector[i].centroid_xyz<<std::endl;
    std::cout<<"At index: "<<cluster_vector[i].index_centroid<<"----- OpenCV index: "<<(*cont).date_container[i].center_index<<std::endl;
    std::cout<<"Type: "<<cluster_vector[i].type<<std::endl;
    }

    return true;
}


    bool send(brio_assembly_vision::TrasformStampedRequest  &req, brio_assembly_vision::TrasformStampedResponse &res)
    {
         //TO_DO: string de la serviciu cu forma obiectului pe care o dorim
        request_a_new_cluster =true;
        if(find_cloud_from_kinect_head==true && final ==false)
        {
            //std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_vector;
            std::vector<Clusters> cluster_vector;
            cluster_vector=cluster_extraction(cloud); //extract clusters
            cluster_center_match(cluster_vector);

            if(first_time==true)
            {
                initial_cluster_size=cluster_vector.size();
            }
            first_time=false;

            if(cluster_vector.size()>0) //after moving objects size is decreased
            {
                PointCloud current_cluster;
                int i=0; //start with the lowest index

                std::string object_with_shape_requested = model[model_step];

                //object detection -- here we should use opencv
//                while(ransac_detect(cluster_vector[i].cluster_point_cloud)!=object_with_shape_requested && i<cluster_vector.size()-1) //
//                {
//                    i++;
//                }
                while(cluster_vector[i].type!=object_with_shape_requested && i<cluster_vector.size()-1) //
                {
                    i++;
                }

                //end while when cluster_vector[i] has the requested shape or when the search index is bigger then cluster_vector

                if(i<=cluster_vector.size()-1) // if i<=cluster_vector.size() then we did find requested object at indices i
                {
                    //std::cerr<<cloud->points.at(307199)<<std::endl;//
                    current_cluster=cluster_vector[i].cluster_point_cloud;

                    std::cerr<<"Cluster"<<i<<" points:"<<std::endl;
                    std::cerr<<cluster_vector[i].centroid_xyz<<std::endl;
                    //                for(int i=0;i<current_cluster.size();i++)
                    //                {
                    //                    std::cerr<<current_cluster.points[i].x<<" "<<current_cluster.points[i].y<<" "<<current_cluster.points[i].z<<std::endl;
                    //                }

                    Eigen::Vector3d z_axe,x_or_y_axe;
                    Eigen::Vector4d translation;
                    z_axe=estimate_plane_normals(cloud);
                    //x_or_y_axe = calculate_one_object_axe(current_cluster);
                    Clusters cluster_curent = cluster_vector[i];
                    x_or_y_axe = obj_axe_2_points(cluster_curent); // to do : see if indices are ok form opencv then use points direct somehow

                    translation = calculate_centroid(current_cluster);
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

    bool send2(brio_assembly_vision::TrasformStampedRequest  &req, brio_assembly_vision::TrasformStampedResponse &res)
    {
         //TO_DO: string de la serviciu cu forma obiectului pe care o dorim
        request_a_new_cluster =true;
        if(find_cloud_from_kinect_head==true && final ==false)
        {
            //std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_vector;

            brio_assembly_vision::Container * cont = new brio_assembly_vision::Container();
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

                    //translation = calculate_centroid(current_cluster);
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

    //bool finish_movement(brio_assembly_vision::Finish_MovementRequest  &req, brio_assembly_vision::Finish_MovementResponse &res)
    //{

    //    ROS_INFO("Finish Movement triggerd");
    //    finish_mv=true;

    //    return true;

    //}

    Eigen::Quaternionf createQuaternion(){
        Eigen::Matrix3f rotation;

        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
                rotation(a,b) = transformata_finala(a,b);

        Eigen::Quaternionf quat (rotation);

        return quat;
    }

//    bool get_info_from_opencv(brio_assembly_vision::Get_Info_From_OpenCVRequest  &req, brio_assembly_vision::Get_Info_From_OpenCVResponse &res)
//    {

//        ROS_INFO("Get info from Paul ");

//        //use data
//        std::vector<int> center_index = req.get_message.center_index;
//        std::cout<<"Receved centers: "<<std::endl;
//        //std::string type = req.msg.piece_type.at(0);
//        for(int i=0;i<req.get_message.center_index.size();i++)
//            {
//                std::cout<<center_index.at(i)<<std::endl;
//                std::cout<<req.get_message.piece_type.at(i)<<std::endl;
//            }
//        //send response to client
//        res.response_status="OK, boss. Merci de informatii:)";

//        //print on our screen
//         ROS_INFO("Sending back response to client !");
//        return true;

//    }

    brio_assembly_vision::Container * client_call(){
        brio_assembly_vision::Is_This_Easier srv;
        brio_assembly_vision::Container * cont = new brio_assembly_vision::Container();
        if (client.call(srv)) //call the service
        {
            if(srv.response.get_message.date_container.size()!=0){
                //use data
                int x = srv.response.get_message.date_container.at(0).center_index;
                std::cout<<"Receved centers: "<<std::endl;
                std::cout<<x<<std::endl;
                //send response to client
                //cont = srv.response.get_message.date_container;
                for(int i=0;i<srv.response.get_message.date_container.size();i++)
                {
                    (*cont).date_container.push_back(srv.response.get_message.date_container.at(i));
                }
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


    int main (int argc, char** argv)
    {
<<<<<<< Updated upstream
//         model.push_back("Linie");
        model.push_back("Semicerc");
        model.push_back("Mare");
        model.push_back("Mic");
        model.push_back("Semicerc");
        model.push_back("Semicerc");
        model.push_back("Semicerc");
=======
         model.push_back("piesa3");
        model.push_back("piesa2");
        model.push_back("piesa1");
        model.push_back("piesa4");
        model.push_back("piesa5");
        //model.push_back("Semicerc");
        //model.push_back("Semicerc");
>>>>>>> Stashed changes
        // Initialize ROS
        ros::init (argc, argv, "pcl_node");
        ros::NodeHandle nh;

#define TOPIC_DEPTH  "/kinect_head/depth_registered/image_raw"
#define TOPIC_COLOR "/kinect_head/rgb/image_color"
        // Create a ROS subscriber for the input point cloud
        //ros::Subscriber sub = nh.subscribe ("/kinect_head/depth_registered/points", 1, cloud_cb);
        ros::Subscriber sub_color = nh.subscribe ("/kinect_head/rgb/image_color", 1, image_cb);
        ros::Subscriber sub_depth = nh.subscribe ("/kinect_head/depth_registered/image_raw", 1, cloud_cb);
        ros::Subscriber sub_write = nh.subscribe ("/kinect_head/depth_registered/image_raw", 1, write_cloud);
        // Create a ROS publisher for the output point cloud
        //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

        // Spin
        //  while(!find_cloud_from_kinect_head)
        //  {
        //    ros::spinOnce();
        //  }

        //  std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_vector;
        //  cluster_vector=cluster_extraction(cloud);
        //  // GLOBAL TRANSFORMATA FINALA //
        // //  Eigen::Matrix4d transformata_finala;
        //  // GLOBAL TRANSFORMATA FINALA //
        //  transformata_finala=calculate_transformation(cluster_vector[0]);
        //  ransac_detect(cluster_vector[0]);

//        ros::ServiceServer service1 = nh.advertiseService("/brio_assembly_vision", send);
        ros::ServiceServer service1 = nh.advertiseService("/brio_assembly_vision", send2);
        ros::ServiceClient client_p = nh.serviceClient <brio_assembly_vision::Is_This_Easier>("/brio_assembly_vision_container");
        client=client_p;
        //ros::ServiceServer service2 = nh.advertiseService("/brio_assembly_vision_opencv_get",get_info_from_opencv);
        //ros::ServiceServer service3 = nh.advertiseService("/brio_assembly_vision_container",get_info_form_opencv_container);


        ros::spin();

        return 0;
    }
