#!/usr/bin/env python3

import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/home/james/workspace/robofun-1.0/object-detection-ws/src/yolobot_recognition/scripts/person.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/kirbot/camera/color/image_raw',
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                class_name = self.model.names[int(c)]
                self.inference_result.class_name = class_name
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

                # Check if the detected class is 'person' and draw a center marker
                if class_name.lower() == 'human':
                    center_x = int((b[0] + b[2]) / 2)
                    center_y = int((b[1] + b[3]) / 2)
                    cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)  # Draw a red circle at the center of the bounding box

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
# 
# 
# # #!/usr/bin/env python3

# import cv2
# from ultralytics import YOLO
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from geometry_msgs.msg import Twist, Vector3

# from yolov8_msgs.msg import InferenceResult
# from yolov8_msgs.msg import Yolov8Inference

# bridge = CvBridge()

# class CameraSubscriber(Node):

#     def __init__(self):
#         super().__init__('camera_subscriber')

#         # self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolov8n.pt')
#         self.model = YOLO('/home/james/drone_ws/src/drone_ros2/yolobot_recognition/scripts/solar_panel.pt')
#         self.yolov8_inference = Yolov8Inference()

#         self.subscription = self.create_subscription(
#             Image,
#             '/simple_drone/bottom/image_raw',
#             self.camera_callback,
#             10)

#         self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
#         self.img_pub = self.create_publisher(Image, "/inference_result", 1)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 1)

#     def camera_callback(self, data):
#         img = bridge.imgmsg_to_cv2(data, "bgr8")
#         results = self.model(img)

#         # Frame dimensions
#         frame_width = 640
#         frame_height = 384
#         center_region_width = 40
#         center_region_height = 40

#         self.yolov8_inference.header.frame_id = "inference"
#         self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

#         for r in results:
#             boxes = r.boxes
#             for box in boxes:
#                 self.inference_result = InferenceResult()
#                 b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates (top, left, bottom, right)
#                 c = box.cls
#                 class_name = self.model.names[int(c)]
#                 self.inference_result.class_name = class_name
#                 self.inference_result.top = int(b[0])
#                 self.inference_result.left = int(b[1])
#                 self.inference_result.bottom = int(b[2])
#                 self.inference_result.right = int(b[3])
#                 self.yolov8_inference.yolov8_inference.append(self.inference_result)
#                 print(class_name.lower())

#                 # Check if the detected object is a 'person' and determine movement direction
#                 if class_name.lower() == 'human':
#                     center_x = int((b[0] + b[2]) / 2)
#                     center_y = int((b[1] + b[3]) / 2)

#                     # Draw a red circle at the center of the bounding box
#                     cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)

#                     # Define the central region boundaries
#                     central_x_min = (frame_width - center_region_width) // 2
#                     central_x_max = (frame_width + center_region_width) // 2
#                     central_y_min = (frame_height - center_region_height) // 2
#                     central_y_max = (frame_height + center_region_height) // 2

#                     # Check if the center of the bounding box is outside the target central region

#                     linear_vec = Vector3()
#                     angular_vec = Vector3()

#                     if center_x < central_x_min:
#                         self.get_logger().info("Move left")
#                         linear_vec.y = 0.2 # Left
#                     elif center_x > central_x_max:
#                         self.get_logger().info("Move right")
#                         linear_vec.y = -0.2 # Right

#                     if center_y < central_y_min:
#                         self.get_logger().info("Move forward")
#                         linear_vec.x = 0.2 # Front
#                     elif center_y > central_y_max:
#                         self.get_logger().info("Move backward")
#                         linear_vec.x = -0.2 # Back
#                     # else:
#                     #     self.get_logger().info("Centered")
                    
#                     twist = Twist(linear=linear_vec, angular=angular_vec)
#                     self.cmd_vel_publisher.publish(twist)

#         annotated_frame = results[0].plot()
#         img_msg = bridge.cv2_to_imgmsg(annotated_frame)

#         self.img_pub.publish(img_msg)
#         self.yolov8_pub.publish(self.yolov8_inference)
#         self.yolov8_inference.yolov8_inference.clear()

# if __name__ == '__main__':
#     rclpy.init(args=None)
#     camera_subscriber = CameraSubscriber()
#     rclpy.spin(camera_subscriber)
#     rclpy.shutdown()

