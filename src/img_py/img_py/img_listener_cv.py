import sys
import cv2
import rclpy
from rclpy.node import Node
import cv_bridge
from sensor_msgs.msg import Image
sys.path.append("~/anaconda3/envs/py_env/lib/python3.8/site-packages")
class ImgListener(Node):
    def __init__(self):
        super().__init__("img_listener")

        self._bgr_img_subscription = self.create_subscription(Image,"bgr_image_topic",self.bgr_callback,10)
        self._depth_img_subscription = self.create_subscription(Image,"depth_image_topic",self.depth_callback,10)
        self.show_clolr = 1
        self.show_depth = 1
        self._color_img_get = 0
        self._depth_img_get = 0
    def bgr_callback(self,img:Image):
        bridge = cv_bridge.CvBridge()
        self.color_img = bridge.imgmsg_to_cv2(img,"bgr8")
        self.get_logger().debug(f"Get Color img {self.color_img.shape}, timestamp:{img.header._stamp}")
        self._color_img_get = 1
        self.update()
    def depth_callback(self,img:Image):
        bridge = cv_bridge.CvBridge()
        self.depth_img = bridge.imgmsg_to_cv2(img,"mono16")
        self.get_logger().debug(f"Get Depth img {self.depth_img.shape}, timestamp:{img.header._stamp}")
        self._depth_img_get = 1 
        self.update()
    def update(self):
        if(self._depth_img_get and self.show_depth):
            cv2.imshow("Depth",self.depth_img)
            self._depth_img_get = 0
        if(self._color_img_get and self.show_clolr):
            cv2.imshow("Color",self.color_img)
            self._color_img_get = 0
        cv2.waitKey(20)
    def __del__(self):
        cv2.destroyAllWindows()
        print("listener close.")

def main():
    rclpy.init()
    node = ImgListener()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ =="__main__":
    main()