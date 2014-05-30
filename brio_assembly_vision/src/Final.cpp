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


ros::Publisher pub;
bool find_cloud_from_kinect_head =false,finish_mv=true;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr cloud (new PointCloud);
void ransac_detect(PointCloud cluster_to_detect);

Eigen::Matrix4d transformata_finala;

Eigen::Quaternionf createQuaternion();
std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_extraction(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);
Eigen::Matrix4d calculate_transformation (PointCloud cloud_cluster);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
   //ROS_INFO("I HAVE IMAGE ON TOPIC FROM KINECT");
if(finish_mv==true)
   {
    ROS_INFO("CALCULATE TRANSFORMATION FOR CLUSTER");
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
  pcl::fromROSMsg(*input, cloud_filtered);

  if(cloud_filtered.size()>0){
    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud_filtered);

    *cloud=cloud_filtered;
    find_cloud_from_kinect_head=true;

    std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_vector;
    cluster_vector=cluster_extraction(cloud);
    // GLOBAL TRANSFORMATA FINALA //
  //  Eigen::Matrix4d transformata_finala;
    // GLOBAL TRANSFORMATA FINALA //
    transformata_finala=calculate_transformation(cluster_vector[0]);
    ransac_detect(cluster_vector[0]);
    finish_mv=false;
    }
  }
  //pub.publish(input);
}

std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > cluster_extraction(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB> );
    pcl::PCDWriter writer;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZRGB>);
     //Create the filtering object: downsample the dataset using a leaf size of 1cm
     pcl::VoxelGrid<pcl::PointXYZRGB> vg;
     vg.setInputCloud (cloud);
     vg.setLeafSize (0.0015f, 0.0015f, 0.0015f);
     //vg.filter (*cloud_filtered);
     //*cloud=*cloud_filtered;

  //   pcl::PassThrough<pcl::PointXYZRGB> pass1;
  //   pass1.setInputCloud(cloud);
  //   pass1.setFilterFieldName("x");
  //   pass1.setFilterLimits(0.0, 0.2);
  //   pass1.filter(*cloud_filtered);
  //   *cloud=*cloud_filtered;

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.1);
    pass.filter (*indices);
    //pass.filter(*cloud_filtered);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (7); //color change , mic=>simte multe  schimbari de culoare
    reg.setRegionColorThreshold (30);
  //  reg.setDistanceThreshold (10);
  //  reg.setPointColorThreshold (15); //color change , mic=>simte multe  schimbari de culoare
  //  reg.setRegionColorThreshold (30);
    reg.setMinClusterSize (800);
    reg.setMaxClusterSize(4900);
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > pcd_vector;

    int j = 0;
      for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

//        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//        std::stringstream ss;
//        ss << "cluster_" << j << ".pcd";
//        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    pcd_vector.push_back(*cloud_cluster);
        j++;
      }
//      for(int i=0;i<pcd_vector.size();i++)
//      {
//                  std::stringstream ss;
//                  ss << "cloud_" << i<< ".pcd";
//                  writer.write<pcl::PointXYZRGB> (ss.str (), pcd_vector[i], false); //*
//      }
std::cout<<"Nmb of cluseters find:"<<pcd_vector.size()<<std::endl;

return pcd_vector;

}

Eigen::Matrix4d calculate_transformation (PointCloud cloud_cluster)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  *cloud=cloud_cluster;

    // Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3f covariance_matrix,result;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;
    // Estimate the XYZ centroid
    pcl::compute3DCentroid(*cloud, xyz_centroid);

    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix(*cloud, xyz_centroid, covariance_matrix);

    //std::cout<<"Centroid"<<std::endl << xyz_centroid << std::endl;

    //std::cout<<"Covariance Matrix"<<std::endl << covariance_matrix << std::endl;

    Eigen::EigenSolver <Eigen::Matrix3f> es(covariance_matrix);
//    std::cout<<std::endl<<"Eigen vectors:"<<std::endl<<es.pseudoEigenvectors()<<std::endl;
    result=es.pseudoEigenvectors();
//    std::cout<<std::endl<<"Eigen values:"<<std::endl<<es.eigenvalues()<<std::endl;

    Eigen::Matrix4d transformata_finala=Eigen::MatrixXd::Identity(4,4);
    for(int i=0;i<3;i++)
       for(int j=0;j<3;j++)
           transformata_finala(i,j)=result(i,j);
    for(int i=0;i<3;i++)
        transformata_finala(i,3)=xyz_centroid(i);
    std::cout<<"Final transform"<<std::endl<<transformata_finala<<std::endl;

    return transformata_finala;

}

void ransac_detect(PointCloud cluster_to_detect)
{
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud=cluster_to_detect;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls ;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;

/*
    mls.setComputeNormals (false);
        // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree_mls);
    mls.setSearchRadius (1);
    mls.setPolynomialOrder(2);
    mls.setSqrGaussParam(0.0025);
        // Reconstruct
    mls.process (*cloud_filtered);


    sor2.setInputCloud (cloud_filtered);
    sor2.setMeanK (300);
    sor2.setStddevMulThresh (2);
    sor2.filter (*cloud);

    mls.setInputCloud (cloud);
    mls.setPolynomialFit (false);
    mls.setSearchMethod (tree_mls);
    mls.setSearchRadius (1);
    mls.setPolynomialOrder(2);
    mls.setSqrGaussParam(0.0025);
        // Reconstruct
    mls.process (*cloud_filtered);
    cloud=cloud_filtered;
*/
    std::vector<int> inliers,in_circle;

    pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model_line (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud));
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB>::Ptr model_circle (new pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_line);
    ransac.setDistanceThreshold (0.0005); // here +- depends on quality of clouds
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac2 (model_circle);
    ransac2.setDistanceThreshold (0.0005); //here +- depends on quality of clouds
    ransac2.computeModel();
    ransac2.getInliers(in_circle);

  std::cout<<"inliers line size="<<inliers.size()<<std::endl;
  std::cout<<"inliers circle size="<<in_circle.size()<<std::endl;
  if(inliers.size()>=in_circle.size())
   { // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *final);
    std::cout<<"Am gasit linie"<<std::endl;
  }
  else
   {
      pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, in_circle, *final);
      std::cout<<"Am gasit semicerc"<<std::endl;
   }

//  pcl::visualization::PCLVisualizer viewer ("ICP demo");
//  int v1(0); int v2(1);
//  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//  //viewer.addCoordinateSystem(0.1,0);
//  viewer.addPointCloud (cloud, "cloud",v1);
//    viewer.addPointCloud (final, "final",v2);
//    while (!viewer.wasStopped ()) {
//        viewer.spinOnce ();
//    }
}

bool send(brio_assembly_vision::TrasformStampedRequest  &req, brio_assembly_vision::TrasformStampedResponse &res)
{

    ROS_INFO("Send TransformedPose");

    Eigen::Quaternionf quat;
    quat = createQuaternion();

      res.msg.transform.translation.x = transformata_finala(0,3);
      res.msg.transform.translation.y = transformata_finala(1,3);
      res.msg.transform.translation.z = transformata_finala(2,3);

      res.msg.transform.rotation.w = (double)quat.w();
      res.msg.transform.rotation.x = (double)quat.x();
      res.msg.transform.rotation.y = (double)quat.y();
      res.msg.transform.rotation.z = (double)quat.z();

  return true;

}
bool finish_movement(brio_assembly_vision::Finish_MovementRequest  &req, brio_assembly_vision::Finish_MovementResponse &res)
{

    ROS_INFO("Finish Movement triggerd");
    finish_mv=true;

  return true;

}

Eigen::Quaternionf createQuaternion(){
      Eigen::Matrix3f rotation;

      for (int a = 0; a < 3; a++)
          for (int b = 0; b < 3; b++)
              rotation(a,b) = transformata_finala(a,b);

      Eigen::Quaternionf quat (rotation);

      return quat;
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/kinect_head/depth_registered/points", 1, cloud_cb);

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

  ros::ServiceServer service1 = nh.advertiseService("/brio_assembly_vision", send);
  ros::ServiceServer service2 = nh.advertiseService("/brio_finish_movement", finish_movement);
  ros::spin();

    return 0;
}
