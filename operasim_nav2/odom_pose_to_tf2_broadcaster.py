from geometry_msgs.msg import TransformStamped
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformListener
from tf2_ros.buffer import Buffer

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from functools import partial
from scipy.spatial.transform import Rotation as Rot


def TransformStamped_to_T(msg: TransformStamped):
    T = np.eye(4)
    T[:3, 3] = [
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z,
    ]
    quat = [
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w,
    ]
    R = Rot.from_quat(quat).as_matrix()
    T[:3, :3] = R
    return T


def PoseStamped_to_T(msg: PoseStamped):
    T = np.eye(4)
    T[:3, 3] = [
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ]
    quat = [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ]
    R = Rot.from_quat(quat).as_matrix()
    T[:3, :3] = R
    return T


class OdomPose2TFPublisher(Node):
    def __init__(self):
        super().__init__("odom_pose_to_tf2_broadcaster")
        self.clock = self.get_clock()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # List machine ids
        self.machine_ids = (
            self.declare_parameter("machine_ids", ["c30r_0"])
            .get_parameter_value()
            .string_array_value
        )
        self.get_logger().info(f"machine_ids: {self.machine_ids}")

        # Pose subscribers
        self.pose_subs = []
        for machine_id in self.machine_ids:
            self.get_logger().info(
                f"Create pose subscriber for /{machine_id}/base_link/pose"
            )
            sub = self.create_subscription(
                PoseStamped,
                f"/{machine_id}/base_link/pose",
                partial(self.pose_callback, frame_id=machine_id),
                5,
                callback_group=ReentrantCallbackGroup(),
            )
            self.pose_subs.append(sub)

        # Odom subscribers
        self.odom_subs = []
        for machine_id in self.machine_ids:
            self.get_logger().info(f"Create odom subscriber for /{machine_id}/odom")
            sub = self.create_subscription(
                Odometry,
                f"/{machine_id}/odom",
                partial(self.odom_callback, frame_id=machine_id),
                5,
                callback_group=ReentrantCallbackGroup(),
            )
            self.odom_subs.append(sub)

        # Local buffer
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create Static Transform Broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_world_map()

        # Use clock to get current time
        self.clock = self.get_clock()

    def publish_static_world_map(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "map"
        t.transform.translation.x = float(0.0)
        t.transform.translation.y = float(0.0)
        t.transform.translation.z = float(0.0)
        t.transform.rotation.x = float(0.0)
        t.transform.rotation.y = float(0.0)
        t.transform.rotation.z = float(0.0)
        t.transform.rotation.w = float(1.0)
        self.tf_static_broadcaster.sendTransform(t)
        self._set_transform(t, static_tf=True)

    def pose_callback(self, msg: PoseStamped, frame_id: str):
        stamp = msg.header.stamp

        # Get transformation matrix from base_link to world
        T_world_base = PoseStamped_to_T(msg)

        # Get transformation matrix from world to map
        try:
            world_to_map = self.tf_buffer.lookup_transform(
                "map", "world", time=stamp, timeout=Duration(seconds=10)
            )
            T_map_world = TransformStamped_to_T(world_to_map)
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform from 'world' to 'map': {e}")
            return

        # Get transformation matrix from odom to base_link
        try:
            odom_to_base_link = self.tf_buffer.lookup_transform(
                f"{frame_id}/base_link",
                f"{frame_id}_tf/odom",
                time=rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec) - Duration(seconds=0.1),  # Allow 0.1s delay
                timeout=Duration(seconds=10),
            )
            T_base_odom = TransformStamped_to_T(odom_to_base_link)
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform from 'odom' to 'base_link' for {frame_id}: {e}")
            return

        # Calculate transformation matrix from odom to map
        T_map_odom = T_map_world @ T_world_base @ T_base_odom

        # Publish tf odom in map coordinates
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = f"{frame_id}_tf/odom"

        t.transform.translation.x = T_map_odom[0, 3]
        t.transform.translation.y = T_map_odom[1, 3]
        t.transform.translation.z = T_map_odom[2, 3]
        quat = Rot.from_matrix(T_map_odom[:3, :3]).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry, frame_id: str):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = f"{frame_id}_tf/odom"
        t.child_frame_id = f"{frame_id}/base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        self._set_transform(t)

    def _set_transform(self, t: TransformStamped, static_tf=False):
        authority = "local"
        if static_tf:
            self.tf_buffer.set_transform_static(t, authority)
        else:
            self.tf_buffer.set_transform(t, authority)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPose2TFPublisher()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
