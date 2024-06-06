#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }
    
    std::cout << "init rclpp" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "initialized" << std::endl;

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    //bool visualization = true;
    //ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    std::cout << "SLAM created" << std::endl;
    auto node = std::make_shared<StereoSlamNode>(argv[1], argv[2], argv[3]);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
