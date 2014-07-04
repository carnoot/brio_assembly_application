/*
 * depth_image_projection.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: blodow
 */

#include </home/stefan/ros_ws/src/brio_assembly_application/brio_assembly_vision_new/include/brio_assembly_vision_new/DepthImageProjection.h>

namespace iai_rs
{
namespace rs_pcl
{

DepthImageProjection::DepthImageProjection()
{
  // TODO Auto-generated constructor stub
}

DepthImageProjection::~DepthImageProjection()
{
  // TODO Auto-generated destructor stub
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr DepthImageProjection::project(const cv::Mat &depth_image_in, const cv::Mat &rgb_image)
{
  cv::Mat depth_image;
  if (depth_image_in.type() == CV_16U)
    depth_image_in.convertTo(depth_image, CV_32F, 0.001, 0.0);
  else
    depth_image = depth_image_in;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // TODO cloud->header.stamp = time;
  cloud->height = depth_image.rows;
  cloud->width = depth_image.cols;
  cloud->is_dense = false;
  cloud->points.resize(cloud->height * cloud->width);
  register const float
      constant = 1.0f / (0.8203125 * cloud->width),
      bad_point = std::numeric_limits<float>::quiet_NaN();
  register const int
      centerX = (cloud->width >> 1),
      centerY = (cloud->height >> 1);

#pragma omp parallel for
  for (int y = 0; y < depth_image.rows; ++y)
  {
    register pcl::PointXYZRGB *pPt = &cloud->points[y * depth_image.cols];
    register const float *pDepth = depth_image.ptr<float>(y), v = (y - centerY) * constant;
    register const cv::Vec3b *pBGR = rgb_image.ptr<cv::Vec3b>(y);

    for (register int u = -centerX; u < centerX; ++u, ++pPt, ++pDepth, ++pBGR)
    {
      pPt->r = pBGR->val[2];
      pPt->g = pBGR->val[1];
      pPt->b = pBGR->val[0];
      //pPt->a = 255;

      register const float depth = *pDepth;
      // Check for invalid measurements
      if (isnan(depth) || depth == 0)
      {
        // not valid
        pPt->x = pPt->y = pPt->z = bad_point;
        continue;
      }
      pPt->z = depth;
      pPt->x = u * depth * constant;
      pPt->y = v * depth;
    }
  }
  return cloud;
}

} // end namespace
} // end namespace
