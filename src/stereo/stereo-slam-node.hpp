#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <cv_bridge/cv_bridge.h>
#include "pcl_conversions/pcl_conversions.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using StrMsg = std_msgs::msg::String;
using ImageMsg = sensor_msgs::msg::Image;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PosMsg = geometry_msgs::msg::PoseStamped;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;



class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual);
    ~StereoSlamNode();

private:
    void GetCommand(const StrMsg::SharedPtr msg);
    void GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight);
    // void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    // void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();
    void Publish();

    rclcpp::Subscription<StrMsg>::SharedPtr   subCommands_;
    // rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    // rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg> > subImgLeft_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg> > subImgRight_;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    rclcpp::Publisher<PointCloudMsg>::SharedPtr pubPointCloud_;
    rclcpp::Publisher<PointCloudMsg>::SharedPtr pubPath;
    rclcpp::Publisher<PosMsg>::SharedPtr pubPos_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;
    Sophus::SE3<float> pose;

    rclcpp::TimerBase::SharedPtr publishThread_;


    // Image
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
