import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_2d_msgs.msg import Pose2DStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Twist, Point, PoseStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import transformations
import copy
import math
from transformations import quaternion_from_euler

class AdvancedOdomPublisher(Node):
    def __init__(self):
        super().__init__('adapter_tf2_broadcaster_4_toy_robots')
        self.wheel_distance = 0.145  # Distance between wheels in meters

        # Retrieve robot names from parameters
        self.du_robot_name = self.declare_parameter('du_robot_name', 'du_126').get_parameter_value().string_value
        self.c30r_robot_name = self.declare_parameter('c30r_robot_name', 'c30r_0').get_parameter_value().string_value
        
        # Retrieve transformation parameters
        # self.static_transform_x = self.declare_parameter('static_transform_x', 0.0).get_parameter_value().double_value
        # self.static_transform_y = self.declare_parameter('static_transform_y', 0.0).get_parameter_value().double_value
        # self.static_transform_z = self.declare_parameter('static_transform_z', 0.0).get_parameter_value().double_value

        self.static_transform_x = 0.0
        self.static_transform_y = 0.0
        self.static_transform_z = 0.0


        # wu add exp moving average for wheel encoder data
        self.right_wheel_smoothed = 0
        self.left_wheel_smoothed = 0

        self.v_right = 0.0
        self.v_left = 0.0
        self.odom_position = None
        self.odom_orientation = None
        self.odom_publisher = self.create_publisher(Odometry, f'/{self.c30r_robot_name}/odom', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_world_map()
        self.publish_static_map_Codom()
        self.timer = self.create_timer(0.1, self.update_and_publish)

        self.twist_subscription = self.create_subscription(
            Twist, f'/{self.c30r_robot_name}/tracks/cmd_vel', 
            self.twist_callback, 
            10)
        self.vel_publisher = self.create_publisher(Twist, f'/{self.du_robot_name}/cmd/body/vel', 10)

        self.pose_subscription = self.create_subscription(
            Pose2DStamped,
            f'/{self.du_robot_name}/status/body/pose',
            self.pose_callback,
            10)
        self.pose_publisher = self.create_publisher(PoseStamped, f'/{self.c30r_robot_name}/base_link/pose', 10)
        
        self.wheel_speed_subscription = self.create_subscription(
            Float32MultiArray,
            f'/{self.du_robot_name}/status/wheel/vel',
            self.wheel_speed_callback,
            10)
        self.wheel_speed_subscription  # prevent unused variable warning


    def publish_static_world_map(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "map"
        t.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_static_broadcaster.sendTransform(t)
        self._set_transform(t, static_tf=True)

    def publish_static_map_Codom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = f"{self.c30r_robot_name}_tf/odom"
        t.transform.translation = Vector3(x=self.static_transform_x, y=self.static_transform_y, z=self.static_transform_z)
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_static_broadcaster.sendTransform(t)
        self._set_transform(t, static_tf=True)

    def _set_transform(self, t: TransformStamped, static_tf=False):
        authority = "local"
        if static_tf:
            self.tf_buffer.set_transform_static(t, authority)
        else:
            self.tf_buffer.set_transform(t, authority)

    def twist_callback(self, msg):
        self.vel_publisher.publish(msg)

    def pose_callback(self, msg):
        new_msg = PoseStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = "base_link"
        new_msg.pose.position.x = msg.pose.x
        new_msg.pose.position.y = msg.pose.y
        new_msg.pose.position.z = 0.0
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta
        q = quaternion_from_euler(0, 0, msg.pose.theta)
        new_msg.pose.orientation.x = q[0]
        new_msg.pose.orientation.y = q[1]
        new_msg.pose.orientation.z = q[2]
        new_msg.pose.orientation.w = q[3]
        self.pose_publisher.publish(new_msg)


    def wheel_speed_callback(self, msg):
        self.right_wheel_smoothed = self.right_wheel_smoothed*0.99 + msg.data[0]*3.25*0.01
        self.left_wheel_smoothed = self.left_wheel_smoothed*0.99 + msg.data[1]*3.25*0.01
        self.v_right = self.right_wheel_smoothed # msg.data[0]*0.0325
        self.v_left = self.left_wheel_smoothed # msg.data[1]*0.0325




    def update_and_publish(self):
        try:
            
            # Get the latest transform
            latest_time = self.tf_buffer.get_latest_common_time(f'{self.c30r_robot_name}_tf/odom', f'{self.du_robot_name}/base_link')
            trans_codom_to_base_link = self.tf_buffer.lookup_transform(f'{self.c30r_robot_name}_tf/odom', f'{self.du_robot_name}/base_link', latest_time)
        

            new_trans_c30r_odom_to_base_link = copy.deepcopy(trans_codom_to_base_link)
            new_trans_c30r_odom_to_base_link.child_frame_id = f'{self.c30r_robot_name}/base_link'
            self.tf_broadcaster.sendTransform(new_trans_c30r_odom_to_base_link)



            self.odom_position = copy.deepcopy(trans_codom_to_base_link.transform.translation)
            self.odom_orientation = copy.deepcopy(trans_codom_to_base_link.transform.rotation)


            #print(f"x: {self.odom_position.x}, y: {self.odom_position.y}, z: {self.odom_position.z}")

            now = self.get_clock().now()

            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = f'{self.c30r_robot_name}_tf/odom'
            odom_msg.child_frame_id = f'{self.c30r_robot_name}/base_link'
            odom_msg.pose.pose.position = Point(x=self.odom_position.x, y=self.odom_position.y, z=self.odom_position.z)
            odom_msg.pose.pose.orientation = self.odom_orientation
            odom_msg.pose.covariance = [1e-3]*36  # Simple example of low uncertainty
            odom_msg.twist.twist.linear.x = (self.v_left + self.v_right) / 2
            odom_msg.twist.twist.angular.z = (self.v_right - self.v_left) / self.wheel_distance
            odom_msg.twist.covariance = [1e-3]*36  # Simple example of low uncertainty
            # odom_msg.twist.twist.linear = velocity_linear
            # odom_msg.twist.twist.angular = velocity_angular
            self.odom_publisher.publish(odom_msg)

            # self.get_logger().info(f'No error')

        except Exception as e:
            self.get_logger().info(f'Error in transformation or publication: {str(e)}')

    # Other methods remain unchanged

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
