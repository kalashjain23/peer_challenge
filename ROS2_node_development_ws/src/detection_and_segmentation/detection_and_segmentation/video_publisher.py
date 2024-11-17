import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.publisher_ = self.create_publisher(Image, "/camera/image_raw", 10)

        self.cap_ = cv2.VideoCapture(0)
        self.bridge_ = CvBridge()
        
        self.timer_ = self.create_timer(0.24, self.publish_image)
        
    def publish_image(self):
        ret, frame = self.cap_.read()
        
        if ret:
            msg = self.bridge_.cv2_to_imgmsg(frame, "bgr8")
            
            self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = VideoPublisher() # publishes the video feed from the camera
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
