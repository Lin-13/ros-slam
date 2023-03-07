#include<string>
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
class ImgSubscpriber : public rclcpp::Node{
public:
    ImgSubscpriber(std::string topic_name): Node("recv_frame_node"),_topic_name(topic_name){
        _subsription = this->create_subscription<std_msgs::msg::String>(_topic_name,10,std::bind(&ImgSubscpriber::subs_callback,this,std::placeholders::_1));
    }
private:
    void subs_callback(std_msgs::msg::String::SharedPtr str){
        RCLCPP_INFO(this->get_logger(),"get frameinfo :%s",str->data.c_str());
    }
    std::string _topic_name; 
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> _subsription;
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr node = std::make_shared<ImgSubscpriber>("frame_size_topic");
    rclcpp::spin(node);
    rclcpp::shutdown();
}