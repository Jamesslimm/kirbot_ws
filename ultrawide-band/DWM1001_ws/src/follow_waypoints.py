import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Initialize navigator as None until navigation is triggered
        self.navigator = None

        # Variable to store the latest waypoint
        self.current_waypoint = None
        self.ready_to_navigate = False  # Flag to trigger navigation
        
        # Subscribe to DWM1001 output for waypoints
        self.subscription_pos = self.create_subscription(
            PointStamped,
            '/dwm_passive/output/DWC504',
            self.position_callback,
            10
        )

        # Subscribe to move command
        self.subscription_move = self.create_subscription(
            String,
            '/move',
            self.move_callback,
            10
        )

        self.get_logger().info("Subscribed to /dwm_active/output/pos for waypoints and /move for navigation trigger.")

    def position_callback(self, msg):
        # Store the latest waypoint coordinates but don't navigate yet
        x, y = msg.point.x, msg.point.y
        self.current_waypoint = self.create_pose(x, y)
        self.get_logger().info(f"New waypoint stored: x={x}, y={y}")

        self.navigate_to_waypoint()
        
    def move_callback(self, msg):
        if msg.data == "1" and self.current_waypoint:
            self.get_logger().info("Move command received! Starting navigation.")
            # Initialize the navigator only when /move receives "1"
            if not self.navigator:
                self.navigator = BasicNavigator()
                self.navigator.waitUntilNav2Active()
            # Start navigation to the latest waypoint
            self.navigate_to_waypoint()
        else:
            self.get_logger().info("Waiting for move command or waypoint...")

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # No rotation
        return pose

    def navigate_to_waypoint(self):
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()
        if self.current_waypoint and self.navigator:
            self.navigator.goToPose(self.current_waypoint)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info("Navigating to the waypoint...")
            
            # Log result of navigation
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Navigation succeeded!")
            elif result == TaskResult.CANCELED:
                self.get_logger().info("Navigation was canceled!")
            elif result == TaskResult.FAILED:
                self.get_logger().info("Navigation failed!")
                
            navigator.lifecycleShutdown()
            # Reset navigator after task completion
            # self.navigator.lifecycleShutdown()
            self.navigator = None  # Allow reinitialization on next "1"

def main(args=None):
    rclpy.init(args=args)
    navigator_node = WaypointNavigator()
    rclpy.spin(navigator_node)
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
