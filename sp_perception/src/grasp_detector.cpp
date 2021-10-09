#include "sp_perception/grasp_detector.hpp"

namespace sp_perception
{
    GraspDetector::GraspDetector(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private)
    {
        // params
        if (!nh_private_.getParam("cloud_topic", cloud_topic_))
        {
            cloud_topic_ = "/camera/depth/color/points";
        }

        // subscribe & services
        cloud_subscriber_ = nh_.subscribe(cloud_topic_, 10, &GraspDetector::cloudCallback, this);
    }
    GraspDetector::~GraspDetector()
    {
    }

    void GraspDetector::cloudCallback(const PointCloudT::ConstPtr &cloud_msg)
    {
    }
}