import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import os
from ultralytics import YOLO

FOCAL_LENGTH = 510
REAL_WIDTH = 0.07
CAMERA_FOV_H = 70
CAMERA_FOV_V = 50
FRAME_WIDTH = 640  
FRAME_HEIGHT = 480 

current_dir = os.path.dirname(os.path.realpath(__file__)) 
model_path = os.path.join(current_dir, 'models', 'campur.pt')  # Merged model

class ObjectTracking(Node):
    def __init__(self):
        super().__init__('object_tracking')
        self.pub_gimbal = self.create_publisher(Float32MultiArray, '/gimbal_target', 10)
        self.pub_target_error = self.create_publisher(Float32MultiArray, '/target_error', 10)
        self.pub_target_distance = self.create_publisher(Float32, '/target_distance', 10)
        self.pub_terdeteksi = self.create_publisher(Bool, '/terdeteksi', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.get_logger().info("Node started")

        # Inisialisasi VideoWriter untuk menyimpan video dalam format MP4
        output_video_path = os.path.join(current_dir, 'output.mp4')
        fourcc = cv.VideoWriter_fourcc(*'mp4v')  # Codec untuk MP4
        self.video_writer = cv.VideoWriter(output_video_path, fourcc, 12, (FRAME_WIDTH, FRAME_HEIGHT))

    def get_target_error(self, x, y):
        return (x - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2), (y - FRAME_HEIGHT / 2) / (FRAME_HEIGHT / 2)

    def get_yaw_pitch(self, x, y):
        dx = (x - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
        dy = (y - FRAME_HEIGHT / 2) / (FRAME_HEIGHT / 2)
        return dx * (CAMERA_FOV_H / 2), -dy * (CAMERA_FOV_V / 2)

    def detect_largest_box(self, frame):
        results = self.model(frame)
        largest_box, largest_area, detected_color = None, 0, None
        
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                if area > largest_area:
                    largest_area = area
                    largest_box = (x1, y1, x2, y2)
                    detected_color = "red" if box.cls == 0 else "blue"  # Adjust class index if needed
        
        return largest_box, detected_color

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        box, detected_color = self.detect_largest_box(frame)
        detected = box is not None
        
        msg_terdeteksi = Bool()
        msg_terdeteksi.data = detected
        self.pub_terdeteksi.publish(msg_terdeteksi)
        
        if detected:
            x1, y1, x2, y2 = box
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
            width = x2 - x1
            error_x, error_y = self.get_target_error(center_x, center_y)
            yaw, pitch = self.get_yaw_pitch(center_x, center_y)
            distance = (FOCAL_LENGTH * REAL_WIDTH) / width

            self.pub_target_error.publish(Float32MultiArray(data=[error_x, error_y]))
            self.pub_gimbal.publish(Float32MultiArray(data=[yaw, pitch]))
            self.pub_target_distance.publish(Float32(data=distance))

            color_bgr = (0, 0, 255) if detected_color == "red" else (255, 0, 0)
            self.get_logger().info(f"{detected_color.capitalize()} Target: Error X = {error_x:.2f}, Y = {error_y:.2f}")
            self.get_logger().info(f"Gimbal: Yaw = {yaw:.2f}°, Pitch = {pitch:.2f}°")
            self.get_logger().info(f"Distance: {distance:.3f} m\n")

            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.circle(frame, (center_x, center_y), 5, (0, 255, 0), 5)
            cv.line(frame, (center_x, 0), (center_x, FRAME_HEIGHT), (0, 255, 0), 2)
        else:
            self.get_logger().info("Tidak ada objek yang terdeteksi.")

        cv.line(frame, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT), (0, 255, 255), 2)
        cv.imshow("Object Tracking", frame)
        self.video_writer.write(frame)  # Menulis frame ke video
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracking()
    try:
        rclpy.spin(node)
    finally:
        node.video_writer.release()  # Tutup VideoWriter
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()