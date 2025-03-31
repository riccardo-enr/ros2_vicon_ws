import rclpy
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from px4_msgs.msg import VehicleOdometry


class ViconToPx4Node(Node):
    """Converts VICON ENU pose data to PX4 NED VehicleOdometry."""

    def __init__(self):
        super().__init__('vicon_to_px4')

        # Declare parameters
        self.declare_parameter('pose_topic', '/vrpn_mocap/Obj1/pose')

        # Create publisher and subscriber
        self._odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10
        )

        self._pose_sub = self.create_subscription(
            Pose,
            self.get_parameter('pose_topic').value,
            self._pose_callback,
            10
        )

    def _pose_callback(self, msg):
        """Process incoming Pose messages and publish converted odometry."""
        odom_msg = self._convert_enu_to_ned(msg)
        self._odom_pub.publish(odom_msg)

    def _convert_enu_to_ned(self, pose_msg):
        """Perform ENU to NED frame conversion for pose data."""
        odom = VehicleOdometry()

        # Convert timestamp (nanoseconds to microseconds)
        odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Convert position (ENU -> NED)
        odom.position = [
            float(pose_msg.position.y),   # North
            float(pose_msg.position.x),   # East
            float(-pose_msg.position.z)   # Down
        ]

        # Convert orientation (ENU -> NED)
        enu_quat = [
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w
        ]

        # Create rotation matrices
        r_enu = Rotation.from_quat(enu_quat)
        r_enu_to_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        r_ned = r_enu_to_ned @ r_enu.as_matrix()

        # Convert back to quaternion
        ned_quat = Rotation.from_matrix(r_ned).as_quat()
        odom.q = ned_quat.tolist()
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED

        return odom


def main(args=None):
    rclpy.init(args=args)
    node = ViconToPx4Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
