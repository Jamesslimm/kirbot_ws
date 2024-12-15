import os
import math
import yaml
import rclpy
import cv2
import matplotlib.pyplot as plt
from rclpy.node import Node
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
import geometry_msgs.msg
import rclpy.duration


class MultiModeNavigator(Node):
    def __init__(self):
        super().__init__('multi_mode_navigator')

        # Load the map metadata
        self.map_metadata = self.load_map_metadata("/home/james/workspace/robofun-1.0/my_map_6.yaml")

        # Mode and navigation state variables
        self.current_mode = 1  # Default mode: off
        self.last_received_mode = None  # Track last received mode to prevent duplicate mode switches
        self.predefined_waypoints = [[0.17, -0.73],[5.32,-0.58],[5.17,-5.83],[-0.13,-5.98]]#[[-0.17, 0.17],[4.5,-0.52],[5.98,-5.53],[-0.62,-5.43]]#[[4.6, 3.059], [11.25, 2.96], [1.55, 7.46]]
        
        # Waypoint tracking
        self.current_waypoint_index = 0
        self.uwb_waypoint = None
        self.interactive_waypoints = []  # New list for interactive waypoint selection
        
        # Map and waypoint selection variables
        self.map_path = "/home/james/workspace/robofun-1.0/my_map_5.pgm"
        self.map_data = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        
        # Flags and state tracking
        self.waypoint_in_progress = False
        self.navigation_cancelled = False
        self.interactive_selection_active = False

        # Navigator object
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Define following distance
        self.follow_distance = 0.3  # Ideal distance from object in meters
        self.front_angle_range = 60  # 120 degrees behind (half on each side)
        self.target_angle = 60  # Target alignment angle
        self.center_angle_range = 4  # Small range for considering "aligned"
        self.too_close_distance = 0.2  # Minimum safe distance to avoid collision
        # PID parameters
        self.kp = 0.2  # Proportional gain
        self.kp_angular = 0.2  # Gain for angular velocity

        # QoS Profile for LaserScan subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers with larger queue size for message buffering
        self.mode_subscription = self.create_subscription(
            Int8, '/mode', self.mode_callback, 
            10  # Increased queue size
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, '/kirbot/scan', self.scan_callback, 
            qos_profile
        )

        self.create_subscription(PointStamped, '/dwm_passive/output/DWC504_transformed', self.position_callback, 10)

        # Timer to check navigation status
        self.navigation_timer = self.create_timer(0.2, self.check_navigation_status)

        # Publisher for robot velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, '/kirbot/cmd_vel', 10)

        # Add subscriber to cmd_vel to track robot movement
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 
            '/kirbot/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Track velocity state
        self.is_moving = False
        self.last_velocity_time = None
        self.current_linear_x = None

        # Initialize Mode 5 functionality
        self.mode_5_init()

        # Logging
        self.get_logger().info("MultiModeNavigator initialized.")

    def cmd_vel_callback(self, msg):
        # Track linear x velocity
        self.current_linear_x = msg.linear.x
        
        # Check if the robot is moving in linear x
        if abs(msg.linear.x) > 0.01:
            self.last_velocity_time = self.get_clock().now()

    def load_map_metadata(self, path):
        with open(path, 'r') as file:
            return yaml.safe_load(file)

    def create_pose(self, x, y, frame_id='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # No rotation
        return pose

    def mode_5_init(self):
        """
        Initialize interactive waypoint selection functionality.
        """
        self.interactive_waypoints = []  # Reset interactive waypoints

    def onclick(self, event):
        """
        Handle mouse click events for interactive waypoint selection.
        """
        if not self.interactive_selection_active:
            return

        if event.xdata is not None and event.ydata is not None:
            # Get pixel coordinates of the click
            x_pixel, y_pixel = int(event.xdata), int(event.ydata)
            self.get_logger().info(f"Clicked at pixel: ({x_pixel}, {y_pixel})")

            # Convert pixel coordinates to world coordinates
            resolution = self.map_metadata['resolution']
            origin = self.map_metadata['origin']
            waypoint = self.pixel_to_world(x_pixel, y_pixel, resolution, origin)
            
            # Add waypoint to the list
            self.interactive_waypoints.append(waypoint)
            self.get_logger().info(f"Waypoint in world coordinates: {waypoint}")

            # Plot a point on the map to visualize the clicked location
            plt.plot(x_pixel, y_pixel, 'ro')
            plt.draw()

    def pixel_to_world(self, x_pixel, y_pixel, resolution, origin):
        """
        Convert pixel coordinates to world coordinates.
        """
        x_world = x_pixel * resolution + origin[0]
        y_world = (self.map_data.shape[0] - y_pixel) * resolution + origin[1]  # y inverted
        return (x_world, y_world)

    def display_interactive_map(self):
        """
        Display the map for interactive waypoint selection.
        """
        # Reset interactive waypoints
        self.interactive_waypoints = []
        
        # Ensure we're in interactive mode
        self.interactive_selection_active = True

        # Plot the map
        plt.figure()
        plt.imshow(self.map_data, cmap='gray')
        plt.title('Click on the map to select waypoints (Close window to start navigation)')
        plt.xlabel(f"Resolution: {self.map_metadata['resolution']} meters/pixel")
        plt.ylabel("Map height (pixels)")
        plt.colorbar(label='Occupancy Value')

        # Connect the click event to the 'onclick' method
        plt.gcf().canvas.mpl_connect('button_press_event', self.onclick)
        
        # Show the plot (this will block until the window is closed)
        plt.show()
        
        # After the map is closed, deactivate interactive selection
        self.interactive_selection_active = False

        return self.interactive_waypoints

    def mode_callback(self, msg):
        # Check if the received mode is different from the last received mode
        new_mode = msg.data
        if new_mode == self.last_received_mode:
            # Ignore duplicate mode messages
            if new_mode == 1 or new_mode == 4 or new_mode == 5:
                return

        # Log the mode change
        self.get_logger().info(f"Mode change request received: from {self.current_mode} to {new_mode}")

        # Update last received mode
        self.last_received_mode = new_mode

        # Update current mode
        self.current_mode = new_mode

        # Take mode-specific actions
        if self.current_mode == 1:
            self.get_logger().info("Switched to Mode 1: All Stop")
            # Stop any ongoing actions
            self.stop_all_actions()
            self.stop_robot()
        elif self.current_mode == 2:
            self.navigation_cancelled = False
            self.get_logger().info("Switching to Mode 2: Single Waypoint Navigation")
            # Note: We don't reset current_waypoint_index 
            # This allows continuing from the last attempted or completed waypoint
            self.start_single_waypoint_navigation()
        elif self.current_mode == 3:
            self.get_logger().info("Switching to Mode 3: UWB Waypoint Navigation")
            if not self.uwb_waypoint:
                self.get_logger().warn("No waypoint available to navigate to.")
                return
            self.navigation_cancelled = False
            self.start_single_waypoint_navigation()
        elif self.current_mode == 4:
            self.get_logger().info("Switching to Mode 4: Dynamic Object Following")
            # Stop any ongoing actions
            self.stop_all_actions()
            self.stop_robot()
        elif self.current_mode == 5:
            self.get_logger().info("Switching to Mode 5: Interactive Waypoint Navigation")
            # Display map and get interactive waypoints
            interactive_points = self.display_interactive_map()
            
            # Convert interactive points to poses
            if interactive_points:
                # Reset waypoint tracking
                self.current_waypoint_index = 0
                # Create poses for navigation
                poses = [self.create_pose(wp[0], wp[1]) for wp in interactive_points]
                
                # Start following waypoints
                self.navigator.followWaypoints(poses)
                self.navigation_cancelled = False
                self.waypoint_in_progress = True
            else:
                self.get_logger().warn("No waypoints selected.")

    def stop_all_actions(self):
        # Cancel any ongoing navigation
        try:
            self.navigation_cancelled = True
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().warn(f"Error canceling navigation: {e}")

        # Reset navigation flags
        self.waypoint_in_progress = False

    def stop_robot(self):
        # Stop the robot's movement
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
    
    def position_callback(self, msg):
        self.uwb_waypoint = [msg.point.x, msg.point.y]
        if self.current_mode == 3:
            self.get_logger().info(f"Updated uwb waypoint to: ({msg.point.x}, {msg.point.y})")

    def start_single_waypoint_navigation(self):
        # Check if there are waypoints left
        if self.current_waypoint_index >= len(self.predefined_waypoints):
            self.get_logger().info("All waypoints have been navigated.")
            self.current_waypoint_index = 0
        
        # Create pose for current waypoint
        if self.current_mode == 2:
            current_pose = self.create_pose(
                self.predefined_waypoints[self.current_waypoint_index][0], 
                self.predefined_waypoints[self.current_waypoint_index][1]
            )
        elif self.current_mode == 3:
            current_pose = self.create_pose(
                self.uwb_waypoint[0], 
                self.uwb_waypoint[1]
            )

        # Start navigation
        if not self.navigation_cancelled:
            self.waypoint_in_progress = True
            self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}: {current_pose.pose.position.x}, {current_pose.pose.position.y}")
            
            # Navigate to single waypoint
            self.navigator.goToPose(current_pose)

    def check_navigation_status(self):
        # Check navigation status for Modes 2, 3, and 5
        if self.current_mode not in [2, 3, 5] or not self.waypoint_in_progress:
            return
        
        try:
            # Check if task is complete
            is_task_complete = self.navigator.isTaskComplete()
            
            # Custom goal reached verification
            goal_reached = self.is_goal_reached()
            
            self.get_logger().info(f"Is Task Complete: {is_task_complete}")
            self.get_logger().info(f"Custom Goal Reached: {goal_reached}")
            
            # If either method suggests completion, process the goal
            if is_task_complete or goal_reached:
                self.get_logger().info("Goal completion detected!")
                
                # For Mode 2 (Predefined Waypoints)
                if self.current_mode == 2:
                    self.current_waypoint_index += 1
                    
                    # Check if we've reached the end of predefined waypoints
                    if self.current_waypoint_index >= len(self.predefined_waypoints):
                        self.get_logger().info("All predefined waypoints have been navigated.")
                        self.current_mode = 1  # Switch to stop mode
                        self.current_waypoint_index = 0  # Reset index
                    else:
                        # Automatically start navigation to next waypoint
                        self.start_single_waypoint_navigation()
                
                # Reset waypoint in progress flag
                self.waypoint_in_progress = False

        except Exception as e:
            self.get_logger().error(f"Exception in navigation status check: {e}")

    def is_goal_reached(self):
        """
        Custom goal reached verification based on cmd_vel topic
        """
        try:
            if self.current_mode == 5:
                return False
            # Check if robot has stopped moving in linear x
            if abs(self.current_linear_x) < 0.01:  # Very small threshold to account for minor fluctuations
                # Check if stopped for 5 seconds
                if self.last_velocity_time:
                    current_time = self.get_clock().now()
                    stopped_duration = current_time - self.last_velocity_time
                    
                    # Log debugging information
                    self.get_logger().info(f"Stopped moving at: {self.last_velocity_time}")
                    self.get_logger().info(f"Current time: {current_time}")
                    self.get_logger().info(f"Stopped duration: {stopped_duration.nanoseconds / 1e9:.3f} seconds")
                    
                    # Consider goal reached if stopped for 5 seconds
                    if stopped_duration.nanoseconds / 1e9 >= 5.0:
                        self.get_logger().info("Goal reached: Robot has stopped moving for 5 seconds")
                        self.last_velocity_time = None
                        return True
            
            return False
        except Exception as e:
            self.get_logger().error(f"Error in goal reached check: {e}")
            return False
    
    # Backward - Mode 5
    # def scan_callback(self, msg):
    #     if self.current_mode != 4:
    #         return
        
    #     # Extract scan ranges within the back 120 degrees
    #     ranges = msg.ranges
    #     back_start = 180 - self.front_angle_range
    #     back_end = 180 + self.front_angle_range

    #     # Extract back ranges, handling wrap-around
    #     if back_end > len(ranges):
    #         back_ranges = ranges[back_start:] + ranges[:back_end % len(ranges)]
    #     else:
    #         back_ranges = ranges[back_start:back_end]

    #     # Filter out invalid values (inf and nan)
    #     valid_ranges = [r for r in back_ranges if math.isfinite(r) and r > 0]

    #     if not valid_ranges:
    #         # If no valid ranges, stop the robot
    #         self.get_logger().warn("No valid ranges detected. Stopping robot.")
    #         twist = Twist()
    #         twist.linear.x = 0.0
    #         twist.angular.z = 0.0
    #         self.cmd_vel_publisher.publish(twist)
    #         return

    #     # Find the minimum distance in this range (closest object)
    #     min_distance = min(valid_ranges)
    #     min_index = back_ranges.index(min_distance)
    #     self.get_logger().info(f"Closest Object Angle = {min_index}")
    #     self.get_logger().info(f"Closest Object: {min_distance:.2f}")

    #     # Initialize movement command
    #     twist = Twist()

    #     # If object is detected within a certain range
    #     if min_distance < float('inf'):
    #         if min_distance > self.follow_distance:
    #             # Move backward if object is far
    #             twist.linear.x = self.kp_linear * (min_distance - self.follow_distance)
    #         elif min_distance < self.too_close_distance:
    #             # Move forward if the object is too close
    #             twist.linear.x = 0.2
    #         else:
    #             # Stop if within the desired follow distance
    #             twist.linear.x = 0.0

    #         # Determine turning or backward-only movement based on object's angle
    #         if abs(min_index - 60) <= self.center_angle_range:
    #             twist.angular.z = 0.0  # Move straight if within central range
    #         else:
    #             twist.angular.z = self.kp_angular * (min_index - 60)

    #         # Clamp linear and angular velocities
    #         twist.linear.x = max(-0.4, min(0.4, twist.linear.x))
    #         twist.angular.z = max(-1.0, min(1.0, twist.angular.z))
    #     else:
    #         # No object detected, stop
    #         twist.linear.x = 0.0
    #         twist.angular.z = 0.0

    #     # Publish command to move the bot
    #     self.get_logger().info(f"Twist Message Published: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}")
    #     self.cmd_vel_publisher.publish(twist)
    # Forward - Mode 4
    def scan_callback(self, msg):
        if self.current_mode != 4:
            return
        
        # Extract scan ranges within the back 120 degrees
        ranges = msg.ranges
        back_start = 180 - self.front_angle_range
        back_end = 180 + self.front_angle_range

        # Extract back ranges, handling wrap-around
        if back_end > len(ranges):
            back_ranges = ranges[back_start:] + ranges[:back_end % len(ranges)]
        else:
            back_ranges = ranges[back_start:back_end]

        # Filter out invalid values (inf and nan)
        valid_ranges = [r for r in back_ranges if math.isfinite(r) and r > 0]

        if not valid_ranges:
            # If no valid ranges, stop the robot
            self.get_logger().warn("No valid ranges detected. Stopping robot.")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            return

        # Find the minimum distance in this range (closest object)
        min_distance = min(valid_ranges)
        min_index = back_ranges.index(min_distance)
        self.get_logger().info(f"Closest Object Angle = {min_index}")
        self.get_logger().info(f"Closest Object: {min_distance:.2f}")

        # Initialize movement command
        twist = Twist()

        # If object is detected within a certain range
        if min_distance < float('inf'):
            if min_distance > self.follow_distance:
                # Move left if object is far
                twist.angular.z = self.kp_linear * (min_distance - self.follow_distance)
            elif min_distance < self.too_close_distance:
                # Move right if the object is too close
                twist.angular.z = -0.2
            else:
                # Stop if within the desired follow distance
                twist.angular.z = 0.0

            # Determine forward/backward movement based on object's angle
            if abs(min_index - 60) <= self.center_angle_range:
                twist.linear.x = 0.0  # Move straight if within central range
            else:
                twist.linear.x = self.kp_angular * (min_index - 60)

            # Clamp linear and angular velocities
            twist.linear.x = max(-0.4, min(0.4, twist.linear.x))
            twist.angular.z = max(-1.0, min(1.0, twist.angular.z))
        else:
            # No object detected, stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish command to move the bot
        self.get_logger().info(f"Twist Message Published: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}")
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MultiModeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# import os
# import math
# import yaml
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import PoseStamped, Twist, PointStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# class MultiModeNavigator(Node):
#     def __init__(self):
#         super().__init__('multi_mode_navigator')

#         # Load the map metadata
#         self.map_metadata = self.load_map_metadata("/home/james/workspace/robofun-1.0/my_map_5.yaml")

#         # Mode and navigation state variables
#         self.current_mode = 1  # Default mode: off
#         self.last_received_mode = None  # Track last received mode to prevent duplicate mode switches
#         self.predefined_waypoints = [[4.6, 3.059], [11.25, 2.96], [1.55, 7.46]]
        
#         # Waypoint tracking
#         self.current_waypoint_index = 0
#         self.uwb_waypoint = None #[[6.65, 1.459]]#None
        
#         # Flags and state tracking
#         self.waypoint_in_progress = False
#         self.navigation_cancelled = False

#         # Navigator object
#         self.navigator = BasicNavigator()
#         self.navigator.waitUntilNav2Active()

#         # Dynamic object following parameters
#         self.angle_origin = 60
#         self.follow_distance = 0.3
#         self.front_angle_range = 60
#         self.center_angle_range = 4
#         self.too_close_distance = 0.2
#         self.kp_linear = 0.2
#         self.kp_angular = 0.2

#         # QoS Profile for LaserScan subscription
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # Subscribers with larger queue size for message buffering
#         self.mode_subscription = self.create_subscription(
#             Int8, '/mode', self.mode_callback, 
#             10  # Increased queue size
#         )
#         self.scan_subscription = self.create_subscription(
#             LaserScan, '/kirbot/scan', self.scan_callback, 
#             qos_profile  # Increased queue size
#         )

#         self.create_subscription(PointStamped, '/dwm_passive/output/DWC504_transformed', self.position_callback, 10)

#         # Timer to check navigation status
#         self.navigation_timer = self.create_timer(0.2, self.check_navigation_status)

#         # Publisher for robot velocity
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/kirbot/cmd_vel', 10)

#         # Logging
#         self.get_logger().info("MultiModeNavigator initialized.")

#     def load_map_metadata(self, path):
#         with open(path, 'r') as file:
#             return yaml.safe_load(file)

#     def create_pose(self, x, y, frame_id='map'):
#         pose = PoseStamped()
#         pose.header.frame_id = frame_id
#         pose.header.stamp = self.navigator.get_clock().now().to_msg()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.orientation.w = 1.0  # No rotation
#         return pose

#     def mode_callback(self, msg):
#         # Check if the received mode is different from the last received mode
#         new_mode = msg.data
#         if new_mode == self.last_received_mode:
#             # Ignore duplicate mode messages
#             if new_mode == 1 or new_mode == 4:
#                 return

#         # Log the mode change
#         self.get_logger().info(f"Mode change request received: from {self.current_mode} to {new_mode}")

#         # Update last received mode
#         self.last_received_mode = new_mode

#         # Update current mode
#         self.current_mode = new_mode

#         # Take mode-specific actions
#         if self.current_mode == 1:
#             self.get_logger().info("Switched to Mode 1: All Stop")
#             # Stop any ongoing actions
#             self.stop_all_actions()
#             self.stop_robot()
#         elif self.current_mode == 2:
#             self.navigation_cancelled = False
#             self.get_logger().info("Switching to Mode 2: Single Waypoint Navigation")
#             # Note: We don't reset current_waypoint_index 
#             # This allows continuing from the last attempted or completed waypoint
#             self.start_single_waypoint_navigation()
#         elif self.current_mode == 3:
#             self.get_logger().info("Switching to Mode 3: UWB Waypoint Navigation")
#             if not self.uwb_waypoint:
#                 self.get_logger().warn("No waypoint available to navigate to.")
#                 return
#             self.navigation_cancelled = False
#             self.start_single_waypoint_navigation()
#         elif self.current_mode == 4:
#             self.get_logger().info("Switching to Mode 4: Dynamic Object Following")
#             # Stop any ongoing actions
#             self.stop_all_actions()
#             self.stop_robot()

#     def stop_all_actions(self):
#         # Cancel any ongoing navigation
#         try:
#             self.navigation_cancelled = True
#             self.navigator.cancelTask()
#         except Exception as e:
#             self.get_logger().warn(f"Error canceling navigation: {e}")

#         # Reset navigation flags
#         self.waypoint_in_progress = False

#     def stop_robot(self):
#         # Stop the robot's movement
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.cmd_vel_publisher.publish(twist)
    
#     def position_callback(self, msg):
#         self.uwb_waypoint = self.create_pose(msg.point.x, msg.point.y)
#         self.get_logger().info(f"Updated uwb waypoint to: ({msg.point.x}, {msg.point.y})")

#     def start_single_waypoint_navigation(self):
#         # Check if there are waypoints left
#         if self.current_waypoint_index >= len(self.predefined_waypoints):
#             self.get_logger().info("All waypoints have been navigated.")
#             self.current_waypoint_index = 0
        
#         # Create pose for current waypoint
#         if self.current_mode == 2:
#             current_pose = self.create_pose(
#                 self.predefined_waypoints[self.current_waypoint_index][0], 
#                 self.predefined_waypoints[self.current_waypoint_index][1]
#             )
#         elif self.current_mode == 3:
#             current_pose = self.create_pose(
#                 self.uwb_waypoint[0][0], 
#                 self.uwb_waypoint[0][1]
#             )

#         # Start navigation
#         if not self.navigation_cancelled:
#             self.waypoint_in_progress = True
#             self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}: {current_pose.pose.position.x}, {current_pose.pose.position.y}")
            
#             # Navigate to single waypoint
#             self.navigator.goToPose(current_pose)

#     def check_navigation_status(self):
#         # Only check navigation status if in Mode 2 and a waypoint is in progress
#         if self.current_mode != 2 or self.current_mode != 3 or not self.waypoint_in_progress:
#             return
        
#         # Check if navigation is complete
#         if self.navigator.isTaskComplete():
#             # Get navigation result
#             result = self.navigator.getResult()
#             if result == TaskResult.SUCCEEDED:
#                 self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} navigation completed successfully")
#                 # Increment waypoint index for next navigation
#                 if self.current_mode == 2:
#                     self.current_waypoint_index += 1
#             else:
#                 self.get_logger().error(f"Waypoint {self.current_waypoint_index + 1} navigation failed")
            
#             # Reset waypoint in progress flag
#             self.waypoint_in_progress = False

#     def scan_callback(self, msg):
#         if self.current_mode != 4:
#             return

#         # [Existing scan callback logic remains the same]
#         ranges = msg.ranges

#         front_start = -self.front_angle_range
#         front_end = self.front_angle_range

#         front_ranges = ranges[front_start:] + ranges[:front_end]

#         valid_ranges = [r for r in front_ranges if math.isfinite(r) and r > 0]

#         if not valid_ranges:
#             # If no valid ranges, stop the robot
#             self.get_logger().warn("No valid ranges detected. Stopping robot.")
#             twist = Twist()
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.cmd_vel_publisher.publish(twist)
#             return
        
#         min_distance = min(valid_ranges)
#         min_index = front_ranges.index(min_distance)
#         self.get_logger().info(f"Closest Object Angle ={min_index}")
#         self.get_logger().info(f"Closest Object: {min_distance:.2f}")

#         # Initialize movement command
#         twist = Twist()

#         # If object is detected within a certain range
#         if min_distance < float('inf'):
#             if min_distance > self.follow_distance:
#                 # Move forward if object is far
#                 twist.linear.x = self.kp_linear * (min_distance - self.follow_distance)
#             elif min_distance < self.too_close_distance:
#                 # Move backward if the object is too close
#                 twist.linear.x = -0.2
#             else:
#                 # Stop if within the desired follow distance
#                 twist.linear.x = 0.0

#             # Determine turning or forward-only movement based on object's angle
#             if (self.front_angle_range - self.center_angle_range) <= min_index <= (self.front_angle_range + self.center_angle_range):
#                 twist.angular.z = 0.0  # Move straight forward if within central 60 degrees
#             else:
#                 twist.angular.z = self.kp_angular * (min_index - self.angle_origin)  # Turn left

            
#             twist.linear.x = max(-0.2, min(0.4, twist.linear.x))
#             twist.angular.z = max(-0.5, min(0.5, twist.angular.z))

#         else:
#             # No object detected, stop
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0

#         # Publish command to move the bot
#         self.get_logger().info(f"Twist Message Published: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}")
#         self.cmd_vel_publisher.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MultiModeNavigator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, PointStamped, PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import math
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from asyncio import create_task, sleep

# class WaypointNavigator(Node):
#     def __init__(self):
#         super().__init__('waypoint_navigator')

#         # QoS Profile for LaserScan subscription
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # Initialize navigator
#         self.navigator = BasicNavigator()
#         self.navigator.waitUntilNav2Active()

#         # Current state and waypoint
#         self.current_mode = 1  # Default: Mode 1 (Paused)
#         self.current_waypoint = None
#         self.task = None  # Active async task

#         # Subscribe to topics
#         self.create_subscription(PointStamped, '/dwm_passive/output/DWC504_transformed', self.position_callback, 10)
#         self.create_subscription(Int8, '/mode', self.mode_callback, 10)
#         self.create_subscription(LaserScan, '/kirbot/scan', self.scan_callback, qos_profile)

#         # Publisher to control robot movement
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/kirbot/cmd_vel', 10)

#         # Dynamic object following parameters
#         self.follow_distance = 0.3
#         self.front_angle_range = 60
#         self.center_angle_range = 4
#         self.too_close_distance = 0.2
#         self.kp_linear = 0.2
#         self.kp_angular = 0.2

#         self.get_logger().info("WaypointNavigator initialized.")

#     def position_callback(self, msg):
#         self.current_waypoint = self.create_pose(msg.point.x, msg.point.y)
#         self.get_logger().info(f"Updated current waypoint to: ({msg.point.x}, {msg.point.y})")

#     def mode_callback(self, msg):
#         new_mode = msg.data
#         if new_mode != self.current_mode:
#             self.get_logger().info(f"Mode changed to {new_mode}")
#             self.current_mode = new_mode

#             # Cancel current task and start the new mode's behavior
#             if self.task:
#                 self.task.cancel()
#             self.task = create_task(self.execute_mode())

#     async def execute_mode(self):
#         try:
#             if self.current_mode == 1:  # Paused
#                 self.get_logger().info("Mode 1: Navigation paused.")
#                 await self.pause_navigation()

#             elif self.current_mode == 2:  # Predefined waypoints loop
#                 self.get_logger().info("Mode 2: Starting predefined waypoints loop.")
#                 await self.navigate_predefined_waypoints()

#             elif self.current_mode == 3:  # Navigate to latest waypoint
#                 self.get_logger().info("Mode 3: Navigating to latest waypoint.")
#                 await self.navigate_to_waypoint()

#             elif self.current_mode == 4:  # Dynamic object following
#                 self.get_logger().info("Mode 4: Starting dynamic object following.")
#                 await self.follow_object()

#         except Exception as e:
#             self.get_logger().error(f"Error in mode {self.current_mode}: {e}")

#     async def pause_navigation(self):
#         self.navigator.cancelTask()
#         self.publish_twist(0.0, 0.0)
#         self.get_logger().info("Navigation paused.")

#     async def navigate_predefined_waypoints(self):
#         waypoints = [[-0.78, 0.37], [-0.78, 5.92], [5.52, 6.37], [5.62, 0.47]]
#         poses = [self.create_pose(x, y) for x, y in waypoints]

#         while self.current_mode == 2 and rclpy.ok():
#             self.navigator.followWaypoints(poses)
#             while not self.navigator.isTaskComplete():
#                 feedback = self.navigator.getFeedback()
#                 if feedback:
#                     self.get_logger().info(f"Executing waypoint {feedback.current_waypoint + 1}/{len(poses)}")
#                 await sleep(0.1)

#             result = self.navigator.getResult()
#             if result != TaskResult.SUCCEEDED:
#                 self.get_logger().warn("Navigation through waypoints failed or was canceled.")
#                 break

#     async def navigate_to_waypoint(self):
#         if not self.current_waypoint:
#             self.get_logger().warn("No waypoint available to navigate to.")
#             return

#         self.navigator.goToPose(self.current_waypoint)
#         while not self.navigator.isTaskComplete():
#             feedback = self.navigator.getFeedback()
#             if feedback:
#                 self.get_logger().info("Navigating to the latest waypoint...")
#             await sleep(0.1)

#         result = self.navigator.getResult()
#         if result == TaskResult.SUCCEEDED:
#             self.get_logger().info("Successfully navigated to the waypoint.")
#         else:
#             self.get_logger().warn("Failed to navigate to the waypoint.")

#     async def follow_object(self):
#         while self.current_mode == 4 and rclpy.ok():
#             await sleep(0.1)  # Allow LaserScan callback to control the robot

#     def scan_callback(self, msg):
#         if self.current_mode != 4:
#             return

#         ranges = msg.ranges
#         valid_ranges = [r for r in ranges if math.isfinite(r) and r > 0]
#         if not valid_ranges:
#             self.publish_twist(0.0, 0.0)
#             return

#         min_distance = min(valid_ranges)
#         min_index = ranges.index(min_distance)

#         twist = Twist()
#         if min_distance > self.follow_distance:
#             twist.linear.x = self.kp_linear * (min_distance - self.follow_distance)
#         elif min_distance < self.too_close_distance:
#             twist.linear.x = -0.2

#         if abs(min_index - 180) > self.center_angle_range:
#             twist.angular.z = self.kp_angular * (min_index - 180)

#         self.publish_twist(twist.linear.x, twist.angular.z)

#     def publish_twist(self, linear_x, angular_z):
#         twist = Twist()
#         twist.linear.x = max(-0.3, min(0.3, linear_x))
#         twist.angular.z = max(-1.0, min(1.0, angular_z))
#         self.cmd_vel_publisher.publish(twist)

#     def create_pose(self, x, y):
#         pose = PoseStamped()
#         pose.header.frame_id = 'map'
#         pose.header.stamp = self.get_clock().now().to_msg()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.orientation.w = 1.0
#         return pose


# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointNavigator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import PointStamped, PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import math
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# class WaypointNavigator(Node):
#     def __init__(self):
#         super().__init__('waypoint_navigator')

#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # Initialize navigator as None until navigation is triggered
#         self.navigator = None

#         # Variable to store the latest waypoint and mode
#         self.current_waypoint = None
#         self.current_mode = None
#         self.mode_2 = False
#         self.mode_3 = False
#         self.mode_4 = False  # Dynamic object following mode

#         # Flags to control navigation modes
#         self.ready_to_navigate = False  # Flag to trigger navigation
#         self.continuous_navigation = False  # Flag for cycling through waypoints in Mode 2

#         # Subscribe to DWM1001 output for waypoints
#         self.subscription_pos = self.create_subscription(
#             PointStamped,
#             '/dwm_passive/output/DWC504_transformed',
#             self.position_callback,
#             10
#         )

#         # Subscribe to mode command
#         self.subscription_mode = self.create_subscription(
#             Int8,
#             '/mode',
#             self.mode_callback,
#             10
#         )

#         # Subscribe to LaserScan for dynamic object following
#         self.scan_subscription = self.create_subscription(
#             LaserScan,
#             '/kirbot/scan',
#             self.scan_callback,
#             qos_profile
#         )

#         # Publisher to control bot movement
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/kirbot/cmd_vel', 10)

#         # Define parameters for dynamic object following
#         self.follow_distance = 0.3  # Ideal distance from object in meters
#         self.front_angle_range = 60  # Front range in degrees
#         self.center_angle_range = 4  # Center tolerance for forward motion
#         self.too_close_distance = 0.2  # Minimum safe distance to avoid collision
#         self.kp_linear = 0.2  # Proportional gain for linear velocity
#         self.kp_angular = 0.2  # Proportional gain for angular velocity

#         self.get_logger().info("WaypointNavigator initialized with mode and object-following capabilities.")

#     def position_callback(self, msg):
#         x, y = msg.point.x, msg.point.y
#         self.current_waypoint = self.create_pose(x, y)

#     def mode_callback(self, msg):
#         self.current_mode = msg.data

#         if self.current_mode == 1:
#             self.get_logger().info("Mode 1 received: Navigation paused.")
#             self.stop_navigation()

#         elif self.current_mode == 2:
#             self.get_logger().info("Mode 2 received: Starting predefined waypoints loop.")
#             self.stop_navigation()
#             if not self.mode_2 and not self.mode_3:
#                 self.start_predefined_navigation()

#         elif self.current_mode == 3 and self.current_waypoint:
#             self.get_logger().info("Mode 3 received: Navigating to latest waypoint.")
#             self.stop_navigation()
#             if not self.mode_2 and not self.mode_3:
#                 self.navigate_to_waypoint()

#         elif self.current_mode == 4:
#             self.get_logger().info("Mode 4 received: Starting dynamic object following.")
#             self.stop_navigation()
#             if not self.mode_2 and not self.mode_3:
#                 self.mode_4 = True

#     def create_pose(self, x, y):
#         pose = PoseStamped()
#         pose.header.frame_id = 'map'
#         pose.header.stamp = self.get_clock().now().to_msg()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.orientation.w = 1.0  # No rotation
#         return pose

#     def start_predefined_navigation(self):
#         self.mode_2 = True
#         self.continuous_navigation = True
#         predefined_waypoints = [[-0.78, 0.37], [-0.78, 5.92], [5.52, 6.37], [5.62, 0.47]]

#         if not self.navigator:
#             self.navigator = BasicNavigator()
#             self.navigator.waitUntilNav2Active()

#         poses = [self.create_pose(wp[0], wp[1]) for wp in predefined_waypoints]

#         while self.continuous_navigation and rclpy.ok():
#             self.navigator.followWaypoints(poses)

#             while not self.navigator.isTaskComplete() and self.continuous_navigation:
#                 feedback = self.navigator.getFeedback()
#                 if feedback:
#                     self.get_logger().info(
#                         f"Executing current waypoint: {feedback.current_waypoint + 1}/{len(poses)}"
#                     )

#             result = self.navigator.getResult()
#             if result == TaskResult.SUCCEEDED:
#                 self.get_logger().info("Navigation through waypoints succeeded!")
#             elif result in [TaskResult.CANCELED, TaskResult.FAILED]:
#                 self.get_logger().info(f"Navigation failed or canceled. Status: {result}")
#                 break

#         self.continuous_navigation = False
#         self.navigator.lifecycleShutdown()
#         self.navigator = None
#         self.mode_2 = False
#         if self.current_mode == 3:
#             self.navigate_to_waypoint()
#         elif self.current_mode == 4:
#             self.mode_4 = True

#     def stop_navigation(self):
#         self.continuous_navigation = False
#         if self.navigator:
#             self.navigator.lifecycleShutdown()
#             self.navigator = None
#             self.mode_2 = False
#             self.mode_3 = False
#             self.mode_4 = False

#     def navigate_to_waypoint(self):
#         self.mode_3 = True
#         if not self.navigator:
#             self.navigator = BasicNavigator()
#             self.navigator.waitUntilNav2Active()

#         if self.current_waypoint and self.navigator:
#             self.navigator.goToPose(self.current_waypoint)
#             while not self.navigator.isTaskComplete():
#                 feedback = self.navigator.getFeedback()
#                 if feedback:
#                     self.get_logger().info("Navigating to the latest waypoint...")
            
#             # Log result of navigation
#             result = self.navigator.getResult()
#             if result == TaskResult.SUCCEEDED:
#                 self.get_logger().info("Navigation succeeded!")
#             elif result == TaskResult.CANCELED:
#                 self.get_logger().info("Navigation was canceled!")
#             elif result == TaskResult.FAILED:
#                 self.get_logger().info("Navigation failed!")

#             self.navigator.lifecycleShutdown()
#             self.navigator = None
#             self.mode_3 = False
#             if self.current_mode == 2:
#                 self.start_predefined_navigation()
#             elif self.current_mode == 4:
#                 self.mode_4 = True

#     def scan_callback(self, msg):
#         if not self.mode_4:
#             return

#         ranges = msg.ranges
#         valid_ranges = [r for r in ranges if math.isfinite(r) and r > 0]

#         if not valid_ranges:
#             self.get_logger().warn("No valid object detected.")
#             self.publish_twist(0.0, 0.0)
#             return

#         min_distance = min(valid_ranges)
#         min_index = ranges.index(min_distance)

#         twist = Twist()
#         if min_distance < float('inf'):
#             if min_distance > self.follow_distance:
#                 twist.linear.x = self.kp_linear * (min_distance - self.follow_distance)
#             elif min_distance < self.too_close_distance:
#                 twist.linear.x = -0.2
#             else:
#                 twist.linear.x = 0.0

#             if abs(min_index - 180) <= self.center_angle_range:
#                 twist.angular.z = 0.0
#             else:
#                 twist.angular.z = self.kp_angular * (min_index - 180)

#         self.publish_twist(twist.linear.x, twist.angular.z)

#     def publish_twist(self, linear_x, angular_z):
#         twist = Twist()
#         twist.linear.x = max(-0.3, min(0.3, linear_x))
#         twist.angular.z = max(-1.0, min(1.0, angular_z))
#         self.cmd_vel_publisher.publish(twist)
#         self.get_logger().info(f"Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointNavigator()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8
# from geometry_msgs.msg import PointStamped, PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# class WaypointNavigator(Node):
#     def __init__(self):
#         super().__init__('waypoint_navigator')
        
#         # Initialize navigator as None until navigation is triggered
#         self.navigator = None

#         # Variable to store the latest waypoint and mode
#         self.current_waypoint = None
#         self.current_mode = None
#         self.mode_2 = False
#         self.mode_3 = False
#         # self.predefined_waypoints = [[4.6, 3.059], [11.25, 2.96], [1.55, 7.46]]
        
#         # Flags to control navigation modes
#         self.ready_to_navigate = False  # Flag to trigger navigation
#         self.continuous_navigation = False  # Flag for cycling through waypoints in Mode 2

#         # Subscribe to DWM1001 output for waypoints
#         self.subscription_pos = self.create_subscription(
#             PointStamped,
#             '/dwm_passive/output/DWC504_transformed',
#             self.position_callback,
#             10
#         )

#         # Subscribe to mode command
#         self.subscription_mode = self.create_subscription(
#             Int8,
#             '/mode',
#             self.mode_callback,
#             10
#         )

#         self.get_logger().info("Subscribed to /dwm_active/output/pos for waypoints and /mode for navigation trigger.")

#     def position_callback(self, msg):
#         # Store the latest waypoint coordinates
#         x, y = msg.point.x, msg.point.y
#         self.current_waypoint = self.create_pose(x, y)
#         # self.get_logger().info(f"New waypoint stored: x={x}, y={y}")
        
#     def mode_callback(self, msg):
#         self.current_mode = msg.data

#         if self.current_mode == 1:
#             # Stop all navigation actions
#             self.get_logger().info("Mode 1 received: Navigation paused.")
#             self.stop_navigation()

#         elif self.current_mode == 2:
#             self.get_logger().info("Mode 2 received: Starting predefined waypoints loop.")
#             self.stop_navigation()
#             if not self.mode_2 and not self.mode_3:
#                 self.start_predefined_navigation()

#         elif self.current_mode == 3 and self.current_waypoint:
#             self.get_logger().info("Mode 3 received: Navigating to latest waypoint.")
#             self.stop_navigation()
#             if not self.mode_2 and not self.mode_3:
#                 self.navigate_to_waypoint()
#             # Initialize the navigator only when /mode is 3 and navigator is not active
#             # if not self.navigator:
#             #     self.navigator = BasicNavigator()
#             #     self.navigator.waitUntilNav2Active()
#             # self.navigate_to_waypoint()

#     def create_pose(self, x, y):
#         pose = PoseStamped()
#         pose.header.frame_id = 'map'
#         pose.header.stamp = self.get_clock().now().to_msg()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.orientation.w = 1.0  # No rotation
#         return pose


#     def start_predefined_navigation(self):
#         self.mode_2 = True
#         self.continuous_navigation = True
#         predefined_waypoints = [[0.3, -0.07], [5.0, -0.57]]
        
#         # Initialize navigator if not already done
#         if not self.navigator:
#             self.navigator = BasicNavigator()
#             self.navigator.waitUntilNav2Active()

#         # Create PoseStamped messages for each waypoint
#         poses = [self.create_pose(wp[0], wp[1]) for wp in predefined_waypoints]

#         while self.continuous_navigation and rclpy.ok():
#             # Begin following the waypoints in sequence
#             self.navigator.followWaypoints(poses)

#             # Monitor navigation status
#             while not self.navigator.isTaskComplete() and self.continuous_navigation:
#                 feedback = self.navigator.getFeedback()
#                 if feedback:
#                     self.get_logger().info(
#                         f"Executing current waypoint: {feedback.current_waypoint + 1}/{len(poses)}"
#                     )

#             # Check the result of the task after completion
#             result = self.navigator.getResult()
#             if result == TaskResult.SUCCEEDED:
#                 self.get_logger().info("Navigation through waypoints succeeded!")
#             elif result == TaskResult.CANCELED:
#                 self.get_logger().info("Navigation was canceled!")
#                 break
#             elif result == TaskResult.FAILED:
#                 self.get_logger().info("Navigation failed!")
#                 break
#             else:
#                 self.get_logger().info("Navigation returned an invalid status!")

#         # Final shutdown if navigation loop exits
#         self.continuous_navigation = False
#         self.navigator.lifecycleShutdown()
#         self.navigator = None
#         self.mode_2 = False
#         if self.current_mode == 3:
#             self.navigate_to_waypoint()

                
#     def stop_navigation(self):
#         # Stop continuous navigation in mode 2 and shutdown the navigator
#         self.continuous_navigation = False
#         # while self.navigator:
#         #     self.navigator.lifecycleShutdown()
#         #     self.navigator = None
#         #     self.get_logger().info("Navigation stopped.")

#     def navigate_to_waypoint(self):
#         self.mode_3 = True
#         if not self.navigator:
#             self.navigator = BasicNavigator()
#             self.navigator.waitUntilNav2Active()

#         if self.current_waypoint and self.navigator:
#             self.navigator.goToPose(self.current_waypoint)
#             while not self.navigator.isTaskComplete():
#                 feedback = self.navigator.getFeedback()
#                 if feedback:
#                     self.get_logger().info("Navigating to the latest waypoint...")
            
#             # Log result of navigation
#             result = self.navigator.getResult()
#             if result == TaskResult.SUCCEEDED:
#                 self.get_logger().info("Navigation succeeded!")
#             elif result == TaskResult.CANCELED:
#                 self.get_logger().info("Navigation was canceled!")
#             elif result == TaskResult.FAILED:
#                 self.get_logger().info("Navigation failed!")

#             self.navigator.lifecycleShutdown()
#             self.navigator = None
#             self.mode_3 = False
#             if self.current_mode == 2:
#                 self.start_predefined_navigation()
#             else:
#                 self.current_mode == 1

# def main(args=None):
#     rclpy.init(args=args)
#     navigator_node = WaypointNavigator()
#     rclpy.spin(navigator_node)
#     navigator_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
