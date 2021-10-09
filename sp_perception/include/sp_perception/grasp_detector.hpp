#ifndef GRASP_POINTS_DETECTOR_H
#define GRASP_POINTS_DETECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/make_shared.hpp>

namespace sp_perception
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    class GraspDetector
    {
    public:
        GraspDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
        ~GraspDetector();

    private:
        // ROS related
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber cloud_subscriber_;
        ros::ServiceServer detection_service_;

        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_broadcaster_;

        // parameters
        std::string cloud_topic_;

        void cloudCallback(const PointCloudT::ConstPtr &cloud_msg);
    }
}
#endif