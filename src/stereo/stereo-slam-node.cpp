#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(const string &vocFile, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = new ORB_SLAM3::System(vocFile, strSettingsFile, ORB_SLAM3::System::STEREO, true);
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    std::cout << "init subs" << std::endl;
    //left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/left");
    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/left");
    std::cout << "camera left subscribed" << std::endl;
    //right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/right");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/right");
    std::cout << "camera right subscribed" << std::endl;

    std::cout << "init syncs" << std::endl;
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    std::cout << "sync approximate" << std::endl;
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
    std::cout << "sync initialized" << std::endl;
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    //std::cout << "grabbing image" << std::endl;
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        //cv::imshow("test window left", cv_ptrLeft->image);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout << "cv_bridge left error" << std::endl;
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
        //cv::imshow("test window right", cv_ptrRight->image);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout << "cv_bridge right error" << std::endl;
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    }
    else
    {
        //std::cout << "doing SLAM" << std::endl;
        //std::cout << Utility::StampToSec(msgLeft->header.stamp) << std::endl;
        m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    }
}
