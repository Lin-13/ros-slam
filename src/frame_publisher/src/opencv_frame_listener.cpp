#include<string>
//OpenCV
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
class ImgSubscpriber : public rclcpp::Node{
public:
    ImgSubscpriber(): Node("listener"),_info_topic_name("frame_info_topic"),_img_topic_name("image_topic"){
        // _info_subscription = this->create_subscription<std_msgs::msg::String>
        //         (_info_topic_name,10,std::bind(&ImgSubscpriber::info_topic_callback,this,std::placeholders::_1));
        _img_subscription = this->create_subscription<sensor_msgs::msg::Image>
                (_img_topic_name,10,std::bind(&ImgSubscpriber::img_topic_callback,this,std::placeholders::_1));
    }
private:
    void info_topic_callback(std_msgs::msg::String::SharedPtr str){
        RCLCPP_INFO(this->get_logger(),"get frameinfo :%s",str->data.c_str());
    }
    void img_topic_callback(sensor_msgs::msg::Image::SharedPtr img){
        cv_bridge::CvImagePtr dst =  cv_bridge::toCvCopy(img,"bgr8");
        // int32_t timestamp_sec = img->header.stamp.sec;
        // int32_t timestamp_nano_sec = img->header.stamp.nanosec;
        // int64_t timestamp = timestamp_sec * 1000 + timestamp_nano_sec / 1e6;
        // RCLCPP_INFO(this->get_logger(),"timestamp: %ld %ld %s",timestamp_sec,timestamp_nano_sec,img->header.frame_id.c_str());
        cv::Mat imCV = dst->image;
        cv::imshow("listener",imCV);
        cv::waitKey(10);
    }
    std::string _info_topic_name,_img_topic_name;
    std::string _topic_name; 
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> _info_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_subscription;
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr node = std::make_shared<ImgSubscpriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}