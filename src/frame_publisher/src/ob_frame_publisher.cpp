#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <glog/logging.h>
//ROS
#include <rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
//C++
#include <chrono>
#include <string>
// My utils
#include <obUtils.hpp>
#include<utils.hpp>
#include <libobsensor/ObSensor.hpp>
using namespace std::chrono_literals;
class ImgPublisher: public rclcpp::Node{
public:
    ImgPublisher(std::shared_ptr<ob::Pipeline> pipe):Node("opencv_frame_node"),enable_color(false),enable_depth(true){
        RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
        //ob Pipe 
        _pipe = pipe;
        StartObdevice(_pipe,enable_color,enable_depth);
        _info_publisher = this->create_publisher<std_msgs::msg::String>("frame_info_topic",10);
        _bgr_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("bgr_image_topic",10);
        _depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_image_topic",10);
        _timer = this->create_wall_timer(30ms,std::bind(&ImgPublisher::time_callback,this));
    }
    ~ImgPublisher(){
        _pipe ->stop();
    }
private:
    void time_callback(){
        cv::Mat img;
        if (enable_color && readObdeviceColor(_pipe,img)){
            std_msgs::msg::String str;
            str.data = std::string("ObSDK Reveive a Color Frame : ") + 
                    "type " + type2str(img.type()) + " " +
                    std::to_string(img.size().height) + " " + 
                    std::to_string(img.size().width);
            RCLCPP_INFO(rclcpp::get_logger("publisher"),str.data.c_str());
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",img).toImageMsg();
            _info_publisher->publish(str);
            _bgr_image_publisher->publish(*msg);
            img.TYPE_MASK;
            img.type();
        }else{
            RCLCPP_INFO(rclcpp::get_logger("publisher"),"Get an empty color frame");
        }
        if (enable_depth && readObdeviceDepth(_pipe,img)){
            std_msgs::msg::String str;
            str.data = std::string("ObSDK Reveive a Depth Frame : ") + 
                    "type " + type2str(img.type()) + " " +
                    std::to_string(img.size().height) + " " + 
                    std::to_string(img.size().width);
            RCLCPP_INFO(rclcpp::get_logger("publisher"),str.data.c_str());
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",img).toImageMsg();
            _info_publisher->publish(str);
            _bgr_image_publisher->publish(*msg);
        }else{
            RCLCPP_INFO(rclcpp::get_logger("publisher"),"Get an empty depth frame");
        }
    }
    bool enable_color,enable_depth;
    std::shared_ptr<ob::Pipeline> _pipe;
    rclcpp::TimerBase::SharedPtr _timer;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _info_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _bgr_image_publisher,_depth_image_publisher;
};
int main(int argc,char** argv){
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
    rclcpp::init(argc,argv);
    RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>();
    std::shared_ptr<ImgPublisher> pub = std::make_shared<ImgPublisher>(pipe);
    rclcpp::spin(pub);
    rclcpp::shutdown();
}