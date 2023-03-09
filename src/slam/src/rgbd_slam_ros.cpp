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
std::string str_vocabulary =    "/home/lwx/code/ros-slam/config/ORBvoc.txt";
std::string camera_settings =   "/home/lwx/code/ros-slam/config/camera.yaml";
ImgBuffer color_img_buffer,depth_img_buffer;
void show_img(std::shared_ptr<bool> should_quit){
    std::cout << "Start thread " << __func__ << std::endl;
    std::string color_window_name = "Color";
    std::string depth_window_name = "Depth";
    cv::Mat color_img,depth_img;
    while(!*should_quit){
        //ESC
        try{
            if(cv::waitKey(30) == 27){
                break;
            }
            if(!color_img_buffer.isEmpty()){
                color_img_buffer.read(color_img);
                cv::imshow(color_window_name,color_img);
            }
            if(!depth_img_buffer.isEmpty()){
                depth_img_buffer.read(depth_img);
                // cv::imshow
                cv::imshow(depth_window_name,depth_img);
            }
        }catch(cv::Exception& e){
            RCLCPP_INFO(rclcpp::get_logger("imshow"),"%s",e.what());
        }
    }
    cv::destroyAllWindows();
}
class MultiImgSubscpriber : public rclcpp::Node{
public:
    MultiImgSubscpriber(): Node("multi_listener"),
                    _info_topic_name("frame_info_topic"),
                    _bgr_img_topic_name("bgr_image_topic"),
                    _depth_img_topic_name("depth_image_topic"),
                    SLAM(str_vocabulary,camera_settings,ORB_SLAM3::System::RGBD, true)
    {
        _info_subscription = this->create_subscription<std_msgs::msg::String>
                (_info_topic_name,10,std::bind(&MultiImgSubscpriber::info_topic_callback,this,std::placeholders::_1));
        _bgr_img_subscription = this->create_subscription<sensor_msgs::msg::Image>
                (_bgr_img_topic_name,10,std::bind(&MultiImgSubscpriber::bgr_img_topic_callback,this,std::placeholders::_1));
        _depth_img_subscription = this->create_subscription<sensor_msgs::msg::Image>
                (_depth_img_topic_name,10,std::bind(&MultiImgSubscpriber::depth_img_topic_callback,this,std::placeholders::_1)); 
        auto get_name = [this](){ std::string name;for(auto i: this->get_node_names()){name+=i;}return name;}; 
        RCLCPP_INFO(this->get_logger(), "node %s setup,listening topic:%s %s",
                    get_name(),
                    _bgr_img_topic_name.c_str(), _depth_img_topic_name.c_str());          
        
    }
    ~MultiImgSubscpriber(){
        SLAM.Shutdown();
    }
private:
    void info_topic_callback(std_msgs::msg::String::SharedPtr str){
        RCLCPP_DEBUG(this->get_logger(),"get frameinfo :%s",str->data.c_str());
    }
    void bgr_img_topic_callback(sensor_msgs::msg::Image::SharedPtr img){
        // RCLCPP_INFO(this->get_logger(),"Receive a color frame");    
        cv_bridge::CvImagePtr dst =  cv_bridge::toCvCopy(img,"bgr8");
        color_img = dst->image;
        timestamp = dst->header.stamp.sec *1e3 + dst->header.stamp.nanosec * 1.0e-6;
        RCLCPP_INFO(this->get_logger(),"timestamp : %lf",timestamp);
        _color_img_ready = true;
        color_img_buffer.write(color_img);
        update();
    }
    void depth_img_topic_callback(sensor_msgs::msg::Image::SharedPtr img){  
        // RCLCPP_INFO(this->get_logger(),"Receive a depth frame");
        //16UC1
        cv_bridge::CvImageConstPtr dst =  cv_bridge::toCvCopy(img,"mono16");
        depth_img = dst->image;
        _depth_img_ready = true;
        depth_img_buffer.write(depth_img);
        update();
    }
    void update(){
        if(_color_img_ready && _depth_img_ready){
            // SLAM.TrackMonocular(color_img,getCUrrentTimeStamp());
            SLAM.TrackRGBD(color_img,depth_img,timestamp);
            _color_img_ready = false;
            _depth_img_ready = false;
        }
        
    }
    std::string _info_topic_name,_bgr_img_topic_name,_depth_img_topic_name;
    ORB_SLAM3::System SLAM;
    bool _color_img_ready, _depth_img_ready;
    cv::Mat color_img,depth_img;
    double timestamp;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> _info_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _bgr_img_subscription,_depth_img_subscription;
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr node = std::make_shared<MultiImgSubscpriber>();
    std::shared_ptr<bool> should_quit = std::make_shared<bool>(0);
    std::thread imgshow(show_img,should_quit);
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(),"listener shutdown");
    *should_quit = 1;
    imgshow.join();
    std::cout << "imshow thread exit" << std::endl;
}