import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Float32MultiArray, Bool
import math

GIMBAL_YAW_CHANNEL = 9
GIMBAL_PITCH_CHANNEL = 10

PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEUTRAL = 1500

SEARCH_RADIUS = 30
SEARCH_SPEED = 0.1
SEARCH_INTERVAL = 0.05

def angle_to_pwm(angle):    
    min_angle, max_angle = -45, 45
    angle = max(min(angle, max_angle), min_angle)
    return int(((angle - min_angle) / (max_angle - min_angle)) * (PWM_MAX - PWM_MIN) + PWM_MIN)

class GimbalControllerRC(Node):
    def __init__(self):
        super().__init__('gimbal_locking')

        self.rc_publisher = self.create_client(CommandLong, '/mavros/cmd/command')
        self.angle_gimbal = self.create_subscription(Float32MultiArray, '/gimbal_target', self.gimbal_callback, 10)
        self.terdeteksi = self.create_subscription(Bool, '/terdeteksi', self.terpal, 10)

        self.target = None
        self.search_timer = None
        self.t = 0 

        self.get_logger().info("Node started")

    def terpal(self, msg):
        self.target = msg.data
        if self.target:
            self.stop_searching()
        else:
            self.start_searching()

    def gimbal_callback(self, msg):
        yaw_target = msg.data[0]
        pitch_target = msg.data[1]

        yaw_pwm = angle_to_pwm(yaw_target)
        pitch_pwm = angle_to_pwm(pitch_target)

        if self.target: 
            self.set_servo(GIMBAL_YAW_CHANNEL, yaw_pwm)
            self.set_servo(GIMBAL_PITCH_CHANNEL, pitch_pwm)
            self.get_logger().info(f"Yaw: {yaw_target:.2f}° -> {yaw_pwm} | Pitch: {pitch_target:.2f}° -> {pitch_pwm}")

    def start_searching(self):
        if self.search_timer is None:
            self.t = 0
            self.search_timer = self.create_timer(SEARCH_INTERVAL, self.circular_search)

    def stop_searching(self):
        if self.search_timer:
            self.destroy_timer(self.search_timer)
            self.search_timer = None

    def circular_search(self):
        if self.target:
            return
        
        yaw_angle = SEARCH_RADIUS * math.sin(2 * math.pi * SEARCH_SPEED * self.t)
        pitch_angle = SEARCH_RADIUS * math.cos(2 * math.pi * SEARCH_SPEED * self.t)

        yaw_pwm = angle_to_pwm(yaw_angle)
        pitch_pwm = angle_to_pwm(pitch_angle)

        self.set_servo(GIMBAL_YAW_CHANNEL, yaw_pwm)
        self.set_servo(GIMBAL_PITCH_CHANNEL, pitch_pwm)

        self.t += SEARCH_INTERVAL

    def set_servo(self, servo_channel, pwm_value):
        request = CommandLong.Request()
        request.command = 183
        request.param1 = float(servo_channel)
        request.param2 = float(pwm_value)

        future = self.rc_publisher.call_async(request)
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

def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerRC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
