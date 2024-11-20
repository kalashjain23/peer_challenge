import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

from rclpy.qos import QoSProfile, ReliabilityPolicy


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")

        self.model_ = YOLO("./src/detection_and_segmentation/models/best.pt") # the fine-tuned YOLOv11 model on our dataset
        self.bridge_ = CvBridge()
        
        self.get_logger().info("Now listening to your topic!")

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscriber_ = self.create_subscription(Image, "/robot1/zed2i/left/image_rect_color", self.callback, qos_profile)

        # for depth topics
        # self.subscriber_ = self.create_subscription(Image, "/camera/depth/image_raw", self.callback, 10)

    def callback(self, data: Image):        
        image = self.bridge_.imgmsg_to_cv2(data, "bgr8")

        results = self.model_(image, stream=True, show=False)

        for result in results:
            boxes = result.boxes
            
            detected = len(boxes.xyxy.numpy()) # total pallets detected
            
            for i in range(detected): # drawing boxes around the detected pallets
                x1, y1, x2, y2 = boxes.xyxy.numpy()[i]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                confidence = boxes.conf.numpy()[i]
                conf = round(float(confidence)*100, 3)

                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, "Pallet: " + str(conf), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            
        cv2.imshow("Object Detection", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = ObjectDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
