#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
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

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        pose = Sophus::SE3f::rotX(0);

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subCommands_ = this->create_subscription<StrMsg>("SLAM/Commands", 10, std::bind(&StereoInertialNode::GetCommand, this, _1));
    //subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgLeft_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/left");
    //subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    subImgRight_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/right");

    pubPointCloud_ = this->create_publisher<PointCloudMsg>("/SLAM/PointCloud", 2);
    pubPath = this->create_publisher<PointCloudMsg>("/SLAM/Path", 2);
    pubPos_ = this->create_publisher<PosMsg>("/SLAM/Position", 2);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *subImgLeft_, *subImgRight_);
    syncApproximate->registerCallback(&StereoInertialNode::GrabStereo, this);
    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
    publishThread_ = this->create_wall_timer(2000ms, std::bind(&StereoInertialNode::Publish, this));

}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GetCommand(const StrMsg::SharedPtr msg){
    unsigned int end_command = msg->data.find_first_of(" ");
    std::string command = msg->data.substr(0,end_command);
    std::string argument = msg->data.substr(end_command+1, msg->data.find_first_of(" ",end_command));
    
    std::cout << "Received command: "<< command << " with argument: " << argument << std::endl;

    if(command== "SaveMap"){
        if(argument.size()>1){
            SLAM_->SavePointCloud(argument);
        }
        else{
            std::cout << "enter file path after command" << std::endl;
        }
    }
    else if(command == "LoadMap"){
        if(argument.size()>1){
            SLAM_->LoadMap(argument);
        }
        else{
            std::cout << "enter file path after command" << std::endl;
        }
    }
    else if(command == "Shutdown"){
        // // Delete sync thread
        // syncThread_->join();
        // delete syncThread_;
        SLAM_->DeactivateLocalizationMode();

        // Stop all threads
        SLAM_->Shutdown();

        // Save camera trajectory
        SLAM_->SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM_->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }
    else if(command == "LocalizationMode"){
        SLAM_->ActivateLocalizationMode();
    }
    else if(command == "TrackingMode"){
        SLAM_->DeactivateLocalizationMode();
    }
    else if(command == "Reset"){
        SLAM_->Reset();
    }
    else{
        std::cout << "Error: Enter valid command: [Shutdown; LocalizationMode; TrackingMode; Reset]" << std::endl;
    }
}

void StereoInertialNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    //grab left image
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();

    //grab right image
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

// void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
// {
//     bufMutexLeft_.lock();

//     if (!imgLeftBuf_.empty())
//         imgLeftBuf_.pop();
//     imgLeftBuf_.push(msgLeft);

//     bufMutexLeft_.unlock();
// }

// void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
// {
//     bufMutexRight_.lock();

//     if (!imgRightBuf_.empty())
//         imgRightBuf_.pop();
//     imgRightBuf_.push(msgRight);

//     bufMutexRight_.unlock();
// }

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            pose = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void StereoInertialNode::Publish(){
    

    // POINT CLOUD
    //std::cout << "trying to get map" << std::endl;
    std::vector<ORB_SLAM3::MapPoint*> pActiveMapPoints = SLAM_->GetAllMapPoints();
    if(pActiveMapPoints.empty()){
        cout << endl << "Vector of map points pActiveMap is empty!" << endl;
        return;
    }

    //std::cout << "got map" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    //std::cout << "created tmp" << std::endl;
    PointCloudMsg message;
    //std::cout << "created message" << std::endl;
    tmp->width = 0;
    tmp->height = 1;
    tmp->is_dense = false;
    tmp->points.resize(tmp->width * tmp->height);
    tmp->header.frame_id = "world";
    //std::cout << "set tmp parameters" << std::endl;
    for (int i = 0; i < pActiveMapPoints.size(); i++)
    {
        Eigen::Vector3f point = pActiveMapPoints[i]->GetWorldPos();
        pcl::PointXYZ pt;
        pt.x = point.x();
        pt.y = point.y();
        pt.z = point.z();
        tmp->points.push_back(pt);
        tmp->width++;
    }
    
    //std::cout << "converted map" << std::endl;
    pcl::toROSMsg(*tmp.get(), message);
    pubPointCloud_->publish(message);
	std::cout << "Published map" << std::endl;


    // // Publish pose data
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    pose_msg.header.frame_id = "world";
    
    Eigen::Quaternion<double> R = pose.unit_quaternion().cast<double>();
    Eigen::Matrix<double, 3, 1> T = pose.translation().cast<double>();
    g2o::SE3Quat quat = g2o::SE3Quat(R, T);
    Eigen::Matrix<double, 7, 1, 0, 7, 1> quaternion = quat.inverse().toVector();
    pose_msg.pose.position.x = quaternion[0];
    pose_msg.pose.position.y = quaternion[1];
    pose_msg.pose.position.z = quaternion[2];
    pose_msg.pose.orientation.x = quaternion[3];
    pose_msg.pose.orientation.y = quaternion[4];
    pose_msg.pose.orientation.z = quaternion[5];
    pose_msg.pose.orientation.w = quaternion[6];
    pubPos_->publish(pose_msg);
    std::cout << "Published pose" << std::endl;

    


    //publish path
    std::vector<ORB_SLAM3::KeyFrame*> KeyFrames = SLAM_->GetAllKeyFrames();
        if(KeyFrames.empty()){
        cout << endl << "Vector of KeyFrames is empty!" << endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudMsg PathMessage;
    tmp2->width = 0;
    tmp2->height = 1;
    tmp2->is_dense = false;
    tmp2->points.resize(tmp2->width * tmp2->height);
    tmp2->header.frame_id = "world";
    for (int i = 0; i < KeyFrames.size(); i++)
    {
        Eigen::Vector3<float> point = KeyFrames[i]->GetCameraCenter();
        pcl::PointXYZ pt;
        pt.x = point(0);
        pt.y = point(1);
        pt.z = point(2);
        tmp2->points.push_back(pt);
        tmp2->width++;
    }
    
    //std::cout << "converted map" << std::endl;
    pcl::toROSMsg(*tmp2.get(), PathMessage);
    pubPath->publish(PathMessage);
    std::cout << "Published path" << std::endl;


}
