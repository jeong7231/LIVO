import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

class ImuTopicRemapper(Node):
    def __init__(self):
        super().__init__('imu_topic_remapper')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        qos_profile1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        # Subscriber: /camera/camera/imu
        self.subscription = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

        # Publisher: /imu/data
        self.publisher = self.create_publisher(
            Imu,
            '/imu/data',
            qos_profile1
        )
        self.get_logger().info('IMU Topic Remapper Node has started.')

    def imu_callback(self, msg):
        # Log receiving message
        self.get_logger().debug('Received IMU data, republishing...')
        # Publish the same message to the new topic
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTopicRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down IMU Topic Remapper Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
