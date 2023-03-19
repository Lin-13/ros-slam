#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
//ROS
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
//C++
#include <chrono>
#include <string>
using namespace std::chrono_literals;
class ImgPublisher: public rclcpp::Node{
public:
    ImgPublisher():Node("opencv_frame_node"){
        RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
        _cap.open(0,cv::CAP_ANY);
        _info_publisher = this->create_publisher<std_msgs::msg::String>("frame_info_topic",10);
        _image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image_topic",10);
        _timer = this->create_wall_timer(30ms,std::bind(&ImgPublisher::time_callback,this));
    }
    ~ImgPublisher(){
        _cap.release();
    }
private:
    void time_callback(){
        if (_cap.isOpened()){
            cv::Mat img,raw;
            _cap.read(img);
            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            if(img.empty()){
                RCLCPP_INFO(rclcpp::get_logger("publisher"),"Get an empty frame");
                return;
            }
            std_msgs::msg::String str;
            str.data = std::string("Reveive a Frame in OpenCV cap: ") + 
                    std::to_string(img.size().height) + " " + 
                    std::to_string(img.size().width) + 
                    " TimeStamp: " + std::to_string(header.stamp.sec);
            RCLCPP_INFO(rclcpp::get_logger("publisher"),str.data.c_str());
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header,"bgr8",img).toImageMsg();
            _info_publisher->publish(str);
            _image_publisher->publish(*msg);
        }else{
            RCLCPP_INFO(rclcpp::get_logger("publisher"),"Device not open");
        }
    }
    rclcpp::TimerBase::SharedPtr _timer;
    cv::VideoCapture _cap;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _info_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
    
    std::shared_ptr<ImgPublisher> pub = std::make_shared<ImgPublisher>();
    rclcpp::spin(pub);
    rclcpp::shutdown();
}