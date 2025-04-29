import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Int8, String
from ackermann_msgs.msg import AckermannDrive
import math
import time

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        
        self.current_state = None
        self.path_type = 3  # Default to manual drive
        
        # Publishers
        self.drive_pub = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/ego_pose5', self.pose_callback, 10)
        self.create_subscription(Int8, '/obstacle_status', self.obstacle_status_callback, 10)
        self.create_subscription(PoseStamped, '/other_vehicle_pose', self.other_vehicle_pose_callback, 10)
        self.create_subscription(String, '/current_state', self.state_callback, 10)
        self.create_subscription(Int8, '/selected_path', self.path_type_callback, 10)

        # Internal state
        self.current_pose = None
        self.other_vehicle_pose = None
        self.path = self.generate_path()  # Generate the waypoint loop
        self.current_target_index = None  # Will be set once based on the follower's initial pose
        self.goal_target_index = None     # Set as the lead vehicle's closest waypoint
        self.obstacle_detected = False
        self.last_other_vehicle_update = time.time()  # Timestamp of last /other_vehicle_pose update

    def path_type_callback(self, msg):
        self.path_type = msg.data
        self.get_logger().info(f"Received path type: {self.path_type}")
        self.path = self.generate_path()  # Regenerate path based on the new path type

    def state_callback(self, msg):
        self.current_state = msg.data
        self.get_logger().info(f"Received current_state: {self.current_state}")

    def pose_callback(self, msg):
        if self.current_state not in ["Platoon_Moving_Autonomous"]:
            self.get_logger().info("Ignoring pose updates due to invalid state.")
            return

        self.current_pose = msg.pose

        # Initialize current_target_index once using the follower's current position.
        if self.current_target_index is None:
            self.current_target_index = self.get_closest_point_index(self.current_pose.position)
            self.get_logger().info(f"Initialized current_target_index: {self.current_target_index}")

        if self.path and not self.obstacle_detected:
            self.follow_path()

    def obstacle_status_callback(self, msg):
        """Stop immediately if an obstacle is detected."""
        if msg.data == 1:
            self.get_logger().warn("Obstacle detected! Stopping the vehicle.")
            self.obstacle_detected = True
            self.publish_stop()
        else:
            self.get_logger().info("No obstacle detected. Resuming trajectory planning.")
            self.obstacle_detected = False

    def other_vehicle_pose_callback(self, msg):
        if self.current_state not in ["Platoon_Moving_Autonomous"]:
            self.get_logger().info("Ignoring lead vehicle updates due to invalid state.")
            return

        self.other_vehicle_pose = msg.pose
        self.last_other_vehicle_update = time.time()  # Update timestamp

        # Compute the lead vehicle's closest waypoint and set it as the goal.
        lead_closest_index = self.get_closest_point_index(self.other_vehicle_pose.position)
        if self.path:
            self.goal_target_index = lead_closest_index
            self.get_logger().info(
                f"Lead vehicle closest index: {lead_closest_index}, "
                f"Goal waypoint index set to: {self.goal_target_index}"
            )

    def generate_path(self):
        path = Path()
        path.header.frame_id = "map"

        # Define positions for each path type.
        if self.path_type == 0:  # Left
            positions = [
                (0.5274, 3.2471), (0.524, 3.8137), (0.5187, 4.4618), (0.4844, 5.3248), (0.5305, 6.1266),
                (0.8123, 6.784), (1.34, 7.2318), (1.9602, 7.2938), (2.9488, 6.7115), (3.1777, 5.8348),
                (3.2446, 4.5748), (3.2637, 3.6256), (3.5826, 2.8678), (4.4764, 2.5189), (5.2749, 2.578),
                (6.3069, 2.5776), (7.0824, 2.1495), (7.2466, 1.4136), (6.9622, 0.8076), (6.3116, 0.535),
                (5.3178, 0.5808), (4.4358, 0.5516), (3.2734, 0.4999), (1.9559, 0.4708), (1.0804, 0.7332),
                (0.8101, 1.3195), (0.6831, 2.2638)
            ]
            # Added new path (straight and right) based on the user story M6.6
        elif self.path_type == 1:  # Straight
            positions = [
                (0.5274, 3.2471), (0.524, 3.8137), (0.5187, 4.4618), (0.4844, 5.3248), (0.5305, 6.1266),
                (0.8123, 6.784), (1.34, 7.2318), (1.9602, 7.2938), (2.9488, 6.7115), (3.1777, 5.8348), 
                (3.3057, 3.9616), (3.298, 2.5177), (3.3497, 1.6556), (3.0256, 0.9404), (2.1241, 0.5974), 
                (1.3361, 0.6296), (1.0804, 0.6932), (0.8101, 1.3195), (0.6831, 2.2638)
            ]
        elif self.path_type == 2:  # Right
            positions = [
                (3.2653, 5.6369), (3.3393, 4.6664), (3.3623, 3.5682), (3.2249, 3.018), (2.8251, 2.6644),
                (2.2902, 2.5823), (1.5251, 2.5827), (1.089, 2.679), (0.4458, 3.6423), (0.4488, 4.5282),
                (0.5139, 5.5765), (0.7567, 6.3882), (1.3943, 6.9942), (2.2368, 7.1878),
                (2.9669, 6.7549), (3.2179, 5.9687)
            ]
        else:  # Manual drive or invalid path type
            positions = []

        waypoints = []
        for x, y in positions:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)
        return waypoints

    def follow_path(self):
        if self.current_state not in ["Platoon_Moving_Autonomous"]:
            self.get_logger().info("Ignoring follow_path due to invalid state.")
            return

        if not self.path or not self.current_pose:
            return

        # Check that lead vehicle information is fresh (30-second timeout).
        if time.time() - self.last_other_vehicle_update > 30.0:
            self.get_logger().info("Other vehicle pose has stopped updating. Stopping the car.")
            self.publish_stop()
            return

        # Get the current sequential target waypoint.
        target_pose = self.path[self.current_target_index].pose

        # If a lead vehicle waypoint (goal) is set, check if it is closer than the sequential target.
        if self.goal_target_index is not None:
            goal_pose = self.path[self.goal_target_index].pose
            dist_seq = self.euclidean_distance(self.current_pose.position, target_pose.position)
            dist_goal = self.euclidean_distance(self.current_pose.position, goal_pose.position)
            if dist_goal < dist_seq:
                self.get_logger().info("Lead vehicle waypoint is closer than sequential target; overriding target.")
                target_pose = goal_pose
                self.current_target_index = self.goal_target_index  # Force target to be the lead's waypoint

            self.get_logger().info(
                f"Follower Position: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}), "
                f"Next Waypoint: ({target_pose.position.x:.2f}, {target_pose.position.y:.2f}), "
                f"Goal Waypoint (Lead): ({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f})"
            )
        else:
            self.get_logger().info(
                f"Follower Position: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}), "
                f"Next Waypoint: ({target_pose.position.x:.2f}, {target_pose.position.y:.2f})"
            )

        # If the follower is within 0.5 m of the target waypoint...
        if self.euclidean_distance(self.current_pose.position, target_pose.position) < 0.5:
            # If this target is the lead vehicle's waypoint, then stop.
            if self.goal_target_index is not None and self.current_target_index == self.goal_target_index:
                self.get_logger().info("Reached lead vehicle waypoint within 0.5m. Stopping.")
                self.publish_stop()
                return
            else:
                self.get_logger().info(f"Reached waypoint index: {self.current_target_index}")
                # Otherwise, proceed to the next waypoint in the sequence.
                self.current_target_index = (self.current_target_index + 1) % len(self.path)
                self.get_logger().info(f"New target waypoint index: {self.current_target_index}")
                target_pose = self.path[self.current_target_index].pose

        # Compute steering angle and speed toward the target waypoint.
        steering_angle = self.compute_steering_angle(target_pose.position)
        speed = self.compute_speed(steering_angle)
        self.get_logger().info(f"Steering angle: {steering_angle:.2f}, Speed: {speed:.2f}")

        self.publish_drive(steering_angle, speed)

    def get_closest_point_index(self, position):
        if self.current_state not in ["Platoon_Moving_Autonomous"]:
            self.get_logger().info("Ignoring closest point calculation due to invalid state.")
            return 0

        min_distance = float('inf')
        closest_index = 0

        for i, waypoint in enumerate(self.path):
            waypoint_position = waypoint.pose.position
            distance = self.euclidean_distance(position, waypoint_position)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def compute_steering_angle(self, target_point):
        if self.current_state not in ["Platoon_Moving_Autonomous"]:
            self.get_logger().info("Ignoring steering angle computation due to invalid state.")
            return 0.0

        car_position = self.current_pose.position
        yaw = self.get_yaw_from_orientation(self.current_pose.orientation)
        target_angle = math.atan2(target_point.y - car_position.y, target_point.x - car_position.x)
        steering_angle = math.degrees(target_angle - yaw)
        # Normalize to [-180, 180]
        steering_angle = (steering_angle + 180) % 360 - 180
        return max(-45, min(45, steering_angle))  # Clamp steering angle

    def compute_speed(self, steering_angle):
        if self.current_state not in ["Platoon_Moving_Autonomous"]:
            self.get_logger().info("Ignoring speed computation due to invalid state.")
            return 0.0

        max_speed = 0.30  # m/s
        min_speed = 0.20  # m/s
        # Reduce speed for sharper turns.
        speed = max_speed - (abs(steering_angle) / 40) * (max_speed - min_speed)
        return speed

    def publish_drive(self, steering_angle, speed):
        if self.current_state not in ["Platoon_Moving_Autonomous"]:
            self.get_logger().info("Ignoring drive command due to invalid state.")
            return

        drive_msg = AckermannDrive()
        drive_msg.steering_angle = float(steering_angle)
        drive_msg.speed = speed
        self.drive_pub.publish(drive_msg)

    def publish_stop(self):
        drive_msg = AckermannDrive()
        drive_msg.steering_angle = 0.0
        drive_msg.speed = 0.0
        self.drive_pub.publish(drive_msg)
        self.get_logger().info("Published STOP command to /ackermann_drive.")

    @staticmethod
    def euclidean_distance(p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    @staticmethod
    def get_yaw_from_orientation(orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
