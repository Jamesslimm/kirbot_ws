import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import threading

class TcpToRosPublisher(Node):
    def __init__(self):
        super().__init__('tcp_to_ros_publisher')
        # ROS publisher
        self.publisher_ = self.create_publisher(Int8, '/mode', 10)
        self.get_logger().info("TCP to ROS publisher node has been started!")
        
        # State variables
        self.last_value = 0  # Default value to publish
        self.new_value_received = False
        
        # Timer to publish at a fixed rate (1 Hz)
        self.create_timer(1.0, self.publish_value)

    def update_value(self, value):
        """Update the value to be published."""
        print(f"[DEBUG] Updating value to: {value}")  # Debug print
        self.last_value = value
        self.new_value_received = True

    def publish_value(self):
        """Publish the last received value or 0 if no value received."""
        msg = Int8()
        msg.data = self.last_value
        self.publisher_.publish(msg)
        
        # Log only when a new value is published or at intervals
        if self.new_value_received:
            self.get_logger().info(f"Published value: {self.last_value}")
            self.new_value_received = False

def start_tcp_server(host, port, ros_publisher):
    # Create and set up the TCP server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((host, port))
        server_socket.listen(1)
        print(f"Server listening on {host}:{port}...")
        
        while True:
            # Accept incoming client connections
            client_socket, client_address = server_socket.accept()
            with client_socket:
                print(f"Connection from {client_address}")
                while True:
                    data = client_socket.recv(1024)  # Receive up to 1024 bytes
                    if not data:
                        break  # Break if no data is received (connection closed)
                    
                    # Decode and strip whitespace from the received data
                    input_string = data.decode('utf-8', errors='ignore').strip()
                    print(f"[DEBUG] Received from client: {input_string}")
                    
                    # Process the input string
                    parts = input_string.split(';')  # Split by semicolon
                    for part in parts:
                        # Split by either colon or comma
                        key_value = part.split(':') if ':' in part else part.split(',')
                        
                        if len(key_value) == 2:
                            key = key_value[0].strip()  # Get the key
                            try:
                                value = int(key_value[1].strip())  # Convert value to integer
                            except ValueError:
                                print(f"[DEBUG] Failed to convert value: {key_value[1]}")
                                continue  # Skip if value can't be converted to an integer
                            
                            # Check if key is 'E' and store the value
                            if key == 'E':
                                print(f"[DEBUG] Extracted value after 'E': {value}")
                                # Use thread-safe method to update value
                                ros_publisher.get_logger().info(f"Updating value to {value}")
                                ros_publisher.update_value(value)

def main():
    # Initialize ROS 2
    rclpy.init()
    
    # Create a ROS node
    ros_publisher = TcpToRosPublisher()
    
    # Start the TCP server on a separate thread
    host = "localhost"
    port = 5000
    tcp_thread = threading.Thread(target=start_tcp_server, args=(host, port, ros_publisher))
    tcp_thread.daemon = True  # Allow the thread to be terminated when the main program exits
    tcp_thread.start()
    
    try:
        # Keep spinning the ROS 2 node to handle communication
        rclpy.spin(ros_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS 2 
        ros_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import socket
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8

# class TcpToRosPublisher(Node):
#     def __init__(self):
#         super().__init__('tcp_to_ros_publisher')
#         self.publisher_ = self.create_publisher(Int8, '/mode', 10)
#         self.get_logger().info("TCP to ROS publisher node has been started!")

#     def publish_value(self, value):
#         msg = Int8()
#         msg.data = value
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Published value: {value}")


# def start_tcp_server(host, port, ros_publisher):
#     # Create and set up the TCP server
#     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
#         server_socket.bind((host, port))
#         server_socket.listen(1)
#         print(f"Server listening on {host}:{port}...")

#         while True:
#             # Accept incoming client connections
#             client_socket, client_address = server_socket.accept()
#             with client_socket:
#                 print(f"Connection from {client_address}")
#                 while True:
#                     data = client_socket.recv(1024)  # Receive up to 1024 bytes
#                     if not data:
#                         break  # Break if no data is received (connection closed)

#                     # Decode and strip whitespace from the received data
#                     input_string = data.decode('utf-8', errors='ignore').strip()
#                     print(f"Received from client: {input_string}")

#                     # Process the input string, similar to the Node-RED script
#                     parts = input_string.split(';')  # Split by semicolon

#                     for part in parts:
#                         key_value = part.split(',')
#                         if len(key_value) == 2:
#                             key = key_value[0].strip()  # Get the key
#                             try:
#                                 value = int(key_value[1].strip())  # Convert value to integer
#                             except ValueError:
#                                 continue  # Skip if value can't be converted to an integer

#                             # Check if key is 'E' and store the value
#                             if key == 'E':
#                                 print(f"Extracted value after 'E': {value}")
#                                 # Publish the value to the '/mode' ROS topic
#                                 ros_publisher.publish_value(value)


# def main():
#     # Initialize ROS 2
#     rclpy.init()

#     # Create a ROS node and TCP server
#     ros_publisher = TcpToRosPublisher()
    
#     # Start the TCP server on localhost:5000
#     host = "localhost"
#     port = 5000
#     start_tcp_server(host, port, ros_publisher)

#     # Keep spinning the ROS 2 node to handle communication
#     rclpy.spin(ros_publisher)

#     # Shutdown ROS 2 after finishing
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
