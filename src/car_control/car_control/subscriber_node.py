import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import tf_transformations
import numpy as np


class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)
        self.pose_orient_pub = self.create_publisher(Float64MultiArray, 'pose_orientation', 10)
        self.x = 0
        self.y = 0
        timer_period = 0.5  # seconds
        self.sub  # prevent unused variable warning
        self.timer = self.create_timer(timer_period, self.pose_orientation_publisher)

    def imu_callback(self, msg):
        self.get_logger().info('The orientation information is: "%s"' % msg.orientation)
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,]
        euler_angles = tf_transformations.euler_from_quaternion(quaternion)
        self.roll, self.pitch, self.yaw = euler_angles

    def pose_orientation_publisher(self):
        position_orientation = Float64MultiArray()            
        position_orientation.data= [self.roll, self.pitch, self.yaw]
        self.pose_orient_pub.publish(position_orientation)

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()