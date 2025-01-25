import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os


class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_number_detection_node')

        model_path = '/home/yerynkim/swlab_final/go1_robot/go1_detection/models/best.pt'
        
        # Load YOLOv8 model
        self.get_logger().info(f"Loading YOLO model from {os.path.abspath(model_path)}")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.create_subscription(
            Image,
            '/camera_face_camera/image_raw',  # Adjust this topic to match your camera topic
            self.image_callback,
            10  # QoS profile
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Run YOLOv8 inference
            results = self.model(cv_image, conf=0.5)  # Detect objects

            # Annotate image with bounding boxes
            annotated_image = results[0].plot() if len(results) > 0 else cv_image

            # Display the images in OpenCV windows
            cv2.imshow("Raw Image", cv_image)
            cv2.imshow("YOLOv8 Detection", annotated_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

