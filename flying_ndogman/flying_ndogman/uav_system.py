import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Bool
from mavros_msgs.msg import VfrHud
from mavros_msgs.srv import CommandLong
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy

GRAVITY = 9.81
SERVO_DROP_LOW = 1400

class UAVController(Node):
    def __init__(self):
        super().__init__('autonomous_mission')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            depth=10
        )

        self.client_set_servo = self.create_client(CommandLong, '/mavros/cmd/command')

        self.sub_target_distance= self.create_subscription(Float32, '/target_distance', self.target_distance_callback, 10)
        self.sub_speed = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.ground_speed_callback, 10)
        self.sub_detect = self.create_subscription(Bool, '/terdeteksi', self.detect_callback, 10)
        self.sub_altitude = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, qos_profile)

        self.uav_altitude = None
        self.ground_speed = None
        self.target = None 
        self.mission_active = None
        self.drop_channel = 6
        self.current_drop = 1

        self.timer = self.create_timer(0.2, self.error_callback)
        self.get_logger().info("Node started")

    def target_distance_callback(self, msg):
        self.target = msg.data

    def altitude_callback(self, msg):
        self.uav_altitude = msg.data

    def ground_speed_callback(self, msg):
        self.ground_speed = msg.groundspeed
    
    def detect_callback(self, msg):
        self.mission_active = msg.data

    def error_callback(self):
        self.get_logger().info(f"DATA DROP: UAVALT{self.uav_altitude}, GSPEED{self.ground_speed}, JTARGET{self.target}")
        if self.uav_altitude is None or self.ground_speed is None or self.target is None:
            self.get_logger().warn(f"Target tidak terdeteksi {self.uav_altitude}, {self.ground_speed}, {self.target}")
            return
        
        if not self.mission_active:
            return
        
        if self.target > self.uav_altitude:
            value = self.target ** 2 - self.uav_altitude ** 2
            if value >= 0:
                distance_to_target = math.sqrt(value)
            else:
                self.get_logger().warn("Skipping calculation: Negative square root")
                return

        if self.uav_altitude >= 0:
            waktu = math.sqrt((2 * self.uav_altitude)/GRAVITY)
            jarak_dropping = waktu * self.ground_speed

            self.get_logger().info(f"Jarak Pesawat: {distance_to_target:.2f} | Jarak Dropping: {jarak_dropping:.2f} | Ground Speed: {self.ground_speed:.2f} ")

            if distance_to_target <= jarak_dropping:
                if self.current_drop == 1:
                    self.execute_dropping()
                    self.create_timer(10.0, self.reset_drop_flag)
                elif self.current_drop == 2:
                    self.drop_channel = 7
                    self.execute_dropping

    def execute_dropping(self):
        self.get_logger().info("Dropping payload!")
        self.set_servo(self.drop_channel, SERVO_DROP_LOW)
        self.mission_active = False
        self.get_logger().info("Payload dropped, mission completed.")

    def set_servo(self, servo_channel, pwm_value):
        request = CommandLong.Request()
        request.command = 183
        request.param1 = float(servo_channel)
        request.param2 = float(pwm_value)

        future = self.client_set_servo.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('DO_SET_SERVO berhasil dikirim')
            else:
                self.get_logger().error('DO_SET_SERVO gagal dikirim!')
        except Exception as e:
            self.get_logger().error(f'Error mengirim DO_SET_SERVO: {e}')

    def reset_drop_flag(self):
        self.get_logger().info("Drop kedua siap dilakukan.")
        self.current_drop = 2

def main(args=None):
    rclpy.init(args=args)
    node = UAVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
