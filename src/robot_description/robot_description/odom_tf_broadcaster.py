import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    """Publish only odom -> base_link. Wheel and sensor TFs come from RSP."""
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = msg.header.frame_id or 'odom'
        tf.child_frame_id = msg.child_frame_id or 'base_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Jazzy can interrupt a pending message conversion during shutdown.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
