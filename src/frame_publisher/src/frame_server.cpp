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
#include <libobsensor/ObSensor.hpp>
using namespace std::chrono_literals;
class ImgPublisher: public rclcpp::Node{
public:
    ImgPublisher(std::shared_ptr<ob::Pipeline> pipe):Node("opencv_frame_node"){
        RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
        //ob Pipe 
        _pipe = pipe;
        StartObdevice(_pipe);
        
    }
    ~ImgPublisher(){
        _pipe ->stop();
    }
private:
    void time_callback(){
        cv::Mat img;
        if (readObdeviceColor(_pipe,img)){
            
            if(img.empty()){
                RCLCPP_INFO(rclcpp::get_logger("publisher"),"Get an empty frame");
                return;
            }
            std_msgs::msg::String str;
            str.data = std::string("Reveive a Frame in OpenCV cap: ") + 
                    std::to_string(img.size().height) + " " + 
                    std::to_string(img.size().width) + 
                    " TimeStamp: ";
            RCLCPP_INFO(rclcpp::get_logger("publisher"),str.data.c_str());
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",img).toImageMsg();
        }else{
            RCLCPP_INFO(rclcpp::get_logger("publisher"),"Device not open");
        }
    }
    std::shared_ptr<ob::Pipeline> _pipe;
    rclcpp::TimerBase::SharedPtr _timer;

};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    google::InitGoogleLogging("radom");
    // google::LogSink sink;
    // google::AddLogSink(&sink);
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
    RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>();
    std::shared_ptr<ImgPublisher> pub = std::make_shared<ImgPublisher>(pipe);
    rclcpp::spin(pub);
    rclcpp::shutdown();
}