import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Gagal membuka kamera!")
            return

        self.timer = self.create_timer(0.1, self.publish_frame)
        self.get_logger().info("Node started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        cv.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
