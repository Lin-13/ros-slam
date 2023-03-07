//System
#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <chrono>

#include <thread>
#include <mutex>
//Dependencies OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
//Dependencies Ros
#include<rclcpp/rclcpp.hpp>
#include<rclcpp/rclcpp.hpp>
//User Define
#include <System.h>
#include<utils.hpp>
ImgBuffer global_buffer;
void read_from_device(cv::VideoCapture cap,std::shared_ptr<bool> enable_read, uint64_t time_cap_ms = 30){
    cv::Mat img;
    *enable_read = true;
    while(*enable_read){
        if(!cap.isOpened()){
            std::cout<<"cap is not open, exit"<<std::endl;
            break;
        }
        cap.read(img);
        global_buffer.write(img);
        usleep(time_cap_ms*1000);
    }
    cap.release();
    *enable_read = false;
    std::cout << "read thread shutdown" << std::endl;
}

bool b_continue_session;

void exit_loop_handler(int s){
   std::cout << "Finishing session" << std::endl;
   b_continue_session = false;

}
int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    std::string str_vocabulary = "/home/lwx/code/slam/config/ORBvoc.txt";
    std::string camera_settings = "/home/lwx/code/slam/config/camera.yaml";

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    cv::Mat imCV;
    ORB_SLAM3::System SLAM(str_vocabulary,camera_settings,ORB_SLAM3::System::MONOCULAR, true);
    cv::VideoCapture cap;
    cap.open(0,cv::CAP_ANY);
    std::shared_ptr<bool> enable_read = std::make_shared<bool>(true);
    std::thread read_img_thread(read_from_device,cap,enable_read, 30);
    while(b_continue_session)
    {
        usleep(30*1000);
        if(cv::waitKey(30)>0){
            break;
        }
        if(!*enable_read){
            std::cout<<"Read thread shuwdown,shutdown main thread"<<std::endl;
            break;
        }
        global_buffer.read(imCV);
        std::cout << "img size" << imCV.size() << "Reading:timestamp " << getCUrrentTimeStamp() << std::endl;
        // cv::imshow("raw",imCV);
        SLAM.TrackMonocular(imCV, getCUrrentTimeStamp());
    }
    *enable_read = false;
    read_img_thread.join();
    // Stop all threads
    SLAM.Shutdown();
    std::cout << "Exit" <<std::endl;

    return 0;
}

