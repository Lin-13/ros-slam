#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <sstream>
#include <string>
using namespace std::chrono_literals;
class ImgPublisher: public rclcpp::Node{
public:
    ImgPublisher():Node("opencv_frame_node"){
        RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
        _cap.open(0,cv::CAP_ANY);
        _publisher = this->create_publisher<std_msgs::msg::String>("frame_size_topic",10);
        _timer = this->create_wall_timer(30ms,std::bind(&ImgPublisher::time_callback,this));
    }
    ~ImgPublisher(){
        _cap.release();
    }
private:
    void time_callback(){
        RCLCPP_INFO(rclcpp::get_logger("publisher"),"Reading");
        if (_cap.isOpened()){
            cv::Mat img;
            _cap.read(img);
            std_msgs::msg::String str;
            std::stringstream str_stream;
            std::string size;
            
            str_stream << img.size();
            str_stream >> size;
            str.data = "Reveive a Frame in OpenCV cap:" + size + "TimeStamp: ";
            _publisher->publish(str);
        }else{
            RCLCPP_INFO(rclcpp::get_logger("publisher"),"Device not open");
        }
    }
    rclcpp::TimerBase::SharedPtr _timer;
    cv::VideoCapture _cap;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    RCLCPP_INFO(rclcpp::get_logger("publisher"),"Setup");
    
    std::shared_ptr<ImgPublisher> pub = std::make_shared<ImgPublisher>();
    rclcpp::spin(pub);
    rclcpp::shutdown();
}