#include <string>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
//SLAM
#include <utils.hpp>
#include <System.h>
std::string str_vocabulary = "/home/lwx/code/slam/config/ORBvoc.txt";
std::string camera_settings = "/home/lwx/code/slam/config/camera.yaml";
;
class ImgSubscpriber : public rclcpp::Node{
public:
    ImgSubscpriber(): Node("listener"),
                    _info_topic_name("frame_info_topic"),
                    _bgr_img_topic_name("bgr_image_topic"),
                    _depth_img_topic_name("depth_img_topic"),
                    SLAM(str_vocabulary,camera_settings,ORB_SLAM3::System::RGBD, true){
        // _info_subscription = this->create_subscription<std_msgs::msg::String>
        //         (_info_topic_name,10,std::bind(&ImgSubscpriber::info_topic_callback,this,std::placeholders::_1));
        _img_subscription = this->create_subscription<sensor_msgs::msg::Image>
                (_bgr_img_topic_name,10,std::bind(&ImgSubscpriber::img_topic_callback,this,std::placeholders::_1));
        
    }
    ~ImgSubscpriber(){
        SLAM.Shutdown();
    }
private:
    void info_topic_callback(std_msgs::msg::String::SharedPtr str){
        RCLCPP_INFO(this->get_logger(),"get frameinfo :%s",str->data.c_str());
    }
    void img_topic_callback(sensor_msgs::msg::Image::SharedPtr img){
        cv_bridge::CvImagePtr dst =  cv_bridge::toCvCopy(img,"bgr8");
        cv::Mat imCV = dst->image;
        SLAM.TrackMonocular(imCV, getCUrrentTimeStamp());
        // SLAM.TrackRGBD();
        cv::waitKey(10);
    }
    ORB_SLAM3::System SLAM;
    std::string _info_topic_name,_bgr_img_topic_name,_depth_img_topic_name;
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