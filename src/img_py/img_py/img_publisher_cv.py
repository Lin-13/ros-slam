import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Header
from sensor_msgs.msg import Image
import cv_bridge
import sys
sys.path.append("~/anaconda3/envs/py_env/lib/python3.8/site-packages")
import torch
import cv2
class ImgPublisher(Node):
    def __init__(self):
        super().__init__("img_publisher")
        self._info_publisher = self.create_publisher(String,"rospy_test",10)
        self._img_publisher = self.create_publisher(Image,"bgr_image_topic",10)
        str = String()
        str.data = "hello: " + "torch "  + torch.cuda.get_device_name() + " cv2 " + cv2.__version__
        self.get_logger().info(str.data)
        self.timer = self.create_timer(0.03,self.timer_callback)
        self.cap = cv2.VideoCapture(0)
    def timer_callback(self):
        # self.get_logger().info("Publishing...")
        str = String()
        bridge = cv_bridge.CvBridge()
        success,frame = self.cap.read()
        if(success):
            str.data = f"img shape: {frame.shape}"
            self.get_logger().info(str.data)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            msg = bridge.cv2_to_imgmsg(frame,'bgr8',header)
            self._info_publisher.publish(str)
            self._img_publisher.publish(msg)
def main():
    rclpy.init()
    print("Setup")
    print(sys.path)
    node = ImgPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    
    main()