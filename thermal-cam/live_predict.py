import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import board
import busio
import adafruit_mlx90640
from ultralytics import YOLO

# Setup MLX90640 thermal camera
i2c = busio.I2C(board.SCL, board.SDA)  # Initialize I2C communication
mlx = adafruit_mlx90640.MLX90640(i2c)  # Create MLX90640 thermal camera object
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ  # Set refresh rate to 2 Hz for thermal data

# Load the YOLO model for human detection
model = YOLO(r"/home/andrew/zj/best.pt")  # Load a pre-trained YOLO model from specified path

class ThermalImagePublisher(Node):
    def __init__(self):
        super().__init__('thermal_image_publisher')
        
        # Publisher for annotated image
        self.image_publisher = self.create_publisher(Image, 'annotated_thermal_image', 10)
        
        # Publisher for human detection count
        self.count_publisher = self.create_publisher(Int32, 'human_detection_count', 10)
        
        self.bridge = CvBridge()
        
        # Timer for periodic frame capture and processing
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz update rate
    
    def get_thermal_image(self):
        frame = np.zeros((24 * 32,))
        mlx.getFrame(frame)  # Capture frame from MLX90640
        data_array = np.reshape(frame, (24, 32))  # Reshape to 24x32 matrix
        return data_array
    
    def process_image(self, data_array):
        min_val, max_val = np.min(data_array), np.max(data_array)
        image = (data_array - min_val) / (max_val - min_val) * 255  # Normalize to 0-255
        image = np.uint8(image)  # Convert to 8-bit
        image = cv2.applyColorMap(image, cv2.COLORMAP_JET)  # Apply color map
        image = cv2.resize(image, (640, 560), interpolation=cv2.INTER_CUBIC)  # Resize for better display
        return image
    
    def timer_callback(self):
        data_array = self.get_thermal_image()
        image = self.process_image(data_array)
        
        # Predict human presence using YOLO model
        results = model.track(image, persist=True)
        
        # Initialize annotated frame and human detection count
        annotated_frame = image.copy()
        human_count = 0

        for detection in results[0].boxes:
            # Filter detections with confidence > 0.5
            if detection.conf >= 0.5:
                x1, y1, x2, y2 = map(int, detection.xyxy[0])
                confidence = float(detection.conf)

                # Rescale bounding box to match thermal image resolution (24x32)
                x1 = int(x1 * 24 / 640)
                y1 = int(y1 * 32 / 560)
                x2 = int(x2 * 24 / 640)
                y2 = int(y2 * 32 / 560)

                # Clamp bounding box to fit within thermal image boundaries
                x1, y1, x2, y2 = max(0, x1), max(0, y1), min(23, x2), min(31, y2)

                # Extract the ROI for temperature analysis
                roi = data_array[y1:y2, x1:x2]

                if roi.size > 0:
                    max_temp = np.max(roi)
                else:
                    max_temp = 0

                # Validate human detection based on temperature range (28-35°C)
                if 28 <= max_temp <= 35:
                    human_count += 1
                    
                    # Draw bounding box on the annotated frame
                    cv2.rectangle(
                        annotated_frame,
                        (x1 * 640 // 24, y1 * 560 // 32),
                        (x2 * 640 // 24, y2 * 560 // 32),
                        (0, 0, 0), 2
                    )

                    # Display confidence and max temperature
                    cv2.putText(
                        annotated_frame,
                        f'Human: {confidence:.2f} | Max Temp: {max_temp:.2f}C',
                        (x1 * 640 // 24, y1 * 560 // 32 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 0),
                        2
                    )

        # Publish the annotated frame
        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.image_publisher.publish(annotated_image_msg)
        
        # Publish the human detection count
        human_count_msg = Int32()
        human_count_msg.data = human_count
        self.count_publisher.publish(human_count_msg)
        
        self.get_logger().info(f'Published annotated image and human count: {human_count}')


def main(args=None):
    rclpy.init(args=args)
    thermal_image_publisher = ThermalImagePublisher()
    rclpy.spin(thermal_image_publisher)
    thermal_image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# import cv2  # OpenCV library for image processing and display
# import numpy as np  # Library for numerical computations
# import board  # Used to access board pins
# import busio  # Handles I2C communication
# import adafruit_mlx90640  # Library for interacting with the MLX90640 thermal camera
# from ultralytics import YOLO  # YOLO model library for object detection

# # Setup MLX90640 thermal camera
# i2c = busio.I2C(board.SCL, board.SDA)  # Initialize I2C communication
# mlx = adafruit_mlx90640.MLX90640(i2c)  # Create MLX90640 thermal camera object
# mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ  # Set refresh rate to 2 Hz for thermal data

# # Load the YOLO model for human detection
# model = YOLO(r"/home/andrew/zj/best.pt")  # Load a pre-trained YOLO model from specified path


# def get_thermal_image():
#     """
#     Captures a frame from the MLX90640 thermal camera, reshapes it into a 24x32 matrix
#     to match the sensor's resolution, and returns the data as a 2D array.
#     """
#     frame = np.zeros((24 * 32,))  # Initialize a flat array for thermal data
#     mlx.getFrame(frame)  # Get temperature data from the MLX90640 sensor
#     data_array = np.reshape(frame, (24, 32))  # Reshape data into a 24x32 grid
#     return data_array


# def process_image(data_array):
#     """
#     Normalizes thermal data to a 0-255 range, applies a color map, and resizes it to 640x560.
#     This makes the image easier to visualize and process for human detection.
#     """
#     # Normalize data_array to 0-255 for creating an 8-bit grayscale image
#     min_val, max_val = np.min(data_array), np.max(data_array)  # Find min and max values
#     image = (data_array - min_val) / (max_val - min_val) * 255  # Normalize to 0-255
#     image = np.uint8(image)  # Convert to unsigned 8-bit (required for OpenCV functions)
#     image = cv2.applyColorMap(image, cv2.COLORMAP_JET)  # Apply a color map for visualization
#     # Resize the image to 640x560 for better display
#     image = cv2.resize(image, (640, 560), interpolation=cv2.INTER_CUBIC)
#     return image


# def main():
#     """
#     Main loop for continuously capturing, processing, and displaying thermal images.
#     Applies human detection on each processed frame and displays the result.
#     """
#     while True:
#         data_array = get_thermal_image()  # Capture a thermal frame
#         image = process_image(data_array)  # Process the thermal data for visualization

#         # Predict human presence in the frame using YOLO model
#         results = model.track(image, persist=True)  # Track objects with YOLO

#         # Initialize a copy of the image for annotation
#         annotated_frame = image.copy()

#         # Filter detections based on confidence and annotate
#         for detection in results[0].boxes:
#             # Display only if confidence level is above 0.5
#             if detection.conf >= 0.2:
#                 x1, y1, x2, y2 = map(int, detection.xyxy[0])  # Get bounding box coordinates
#                 confidence = float(detection.conf)  # Convert confidence to a float

#                 # Rescale bounding box from 640x560 image to 24x32 thermal image
#                 x1 = int(x1 * 24 / 640)  # Rescale x1 to thermal image width (24)
#                 y1 = int(y1 * 32 / 560)  # Rescale y1 to thermal image height (32)
#                 x2 = int(x2 * 24 / 640)  # Rescale x2 to thermal image width (24)
#                 y2 = int(y2 * 32 / 560)  # Rescale y2 to thermal image height (32)

#                 # Ensure the bounding box is within the thermal image size (24x32)
#                 x1 = max(0, x1)
#                 y1 = max(0, y1)
#                 x2 = min(23, x2)  # 23 is the max index for the x-dimension in 24x32
#                 y2 = min(31, y2)  # 31 is the max index for the y-dimension in 24x32

#                 # Extract the region of interest (ROI) from the thermal data
#                 roi = data_array[y1:y2, x1:x2]  # Get temperature data inside the bounding box

#                 # Check if the ROI is not empty
#                 if roi.size > 0:
#                     max_temp = np.max(roi)  # Calculate the maximum temperature in the ROI
#                 else:
#                     max_temp = 0  # Set max_temp to 0 if ROI is empty

#                 # Check if the max temperature is within the desired range (26 to 35°C)
#                 if 28 <= max_temp <= 35:
#                     # Draw bounding box around detection in black color with increased thickness
#                     cv2.rectangle(annotated_frame,
#                                   (x1 * 640 // 24, y1 * 560 // 32),
#                                   (x2 * 640 // 24, y2 * 560 // 32),
#                                   (255, 255, 255), 4)  # Black color with thickness of 4

#                     # Display confidence and maximum temperature near the bounding box in black
#                     cv2.putText(
#                         annotated_frame,
#                         f'Human: {confidence:.2f} | Max Temp: {max_temp:.2f}C',  # Show confidence and max temp
#                         (x1 * 640 // 24, y1 * 560 // 32 - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX,
#                         0.5,
#                         (255, 255, 255),  # Black color for text
#                         2  # Text thickness
#                     )

#         # Display the annotated thermal image with detection boxes, confidence, and max temperature
#         cv2.imshow('Human Detection Result', annotated_frame)

#         # Exit loop when 'q' is pressed
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cv2.destroyAllWindows()  # Close all OpenCV windows after loop exits


# if __name__ == '__main__':
#     main()  # Run the main function if script is executed directly
