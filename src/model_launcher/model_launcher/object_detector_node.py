import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
except ImportError:
    print("Warning: Ultralytics library not found. Please install it with 'pip install ultralytics'.")
    print("Or update the code to load your model manually.")
    exit()

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        # Declare parameters for model path, camera topic, and output topic
        self.declare_parameter('model_path', 'default_model_path_will_be_set_below')
        self.declare_parameter('camera_topic', '/camera_rgb/image_raw')
        self.declare_parameter('detection_image_topic', '/ika/object_detections_image')
        self.declare_parameter('display_image', True)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)

        # Get parameter values
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.detection_image_topic = self.get_parameter('detection_image_topic').get_parameter_value().string_value
        
        # Handle display_image parameter - it might come as string from launch file
        display_param = self.get_parameter('display_image').get_parameter_value()
        if hasattr(display_param, 'bool_value'):
            self.display_image = display_param.bool_value
        else:
            # If it comes as string, convert it
            self.display_image = display_param.string_value.lower() in ['true', '1', 'yes']
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value

        # Set the default model path
        package_share_directory = get_package_share_directory('model_launcher') # Assuming 'model_launcher' is your package name
        default_model_full_path = os.path.join(package_share_directory, 'models', 'best.pt') # <--- CHANGE 'your_model.pt' to your model's filename
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if self.model_path == 'default_model_path_will_be_set_below':
            self.model_path = default_model_full_path

        self.get_logger().info(f'Using model path: {self.model_path}')

        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        try:
            # Load the YOLOv8 model
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'YOLOv8 model successfully loaded: {self.model_path}')
            self.model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"Error loading YOLOv8 model: {e}")
            self.get_logger().error("Please check if the model path is correct and Ultralytics is installed.")
            rclpy.shutdown()
            return

        # Create a subscriber for the camera image
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # Create a publisher for the annotated image
        self.image_publisher_ = self.create_publisher(
            Image,
            self.detection_image_topic,
            10
        )

        self.get_logger().info(f'Object Detection Node started.')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Detection image topic: {self.detection_image_topic}')
        self.get_logger().info(f'Display image: {self.display_image}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'IOU threshold: {self.iou_threshold}')

        self.class_names = ['1', '10', '11', '12', '2', '3', '4', '4_bitis', '5', '6', '7', '8', '9', 'Dur'] # <--- UPDATED WITH YAML DATA

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        original_shape = cv_image.shape

        # Run YOLOv8 inference
        predictions = self.model.predict(cv_image, conf=self.confidence_threshold, iou=self.iou_threshold, verbose=False)

        # Process the raw predictions
        processed_detections = self.postprocess_detections(predictions, original_shape)

        output_image = cv_image.copy()

        # Draw detections on the image
        output_image = self.draw_detections(output_image, processed_detections)

        # Display the image if enabled
        if self.display_image:
            try:
                cv2.imshow("Object Detections", output_image)
                cv2.waitKey(1)
                self.get_logger().debug('Image displayed successfully')
            except Exception as e:
                self.get_logger().error(f'Error displaying image: {e}')

        # Publish the annotated image
        try:
            self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f'Processed image publishing error: {e}')

    def postprocess_detections(self, predictions, original_shape):
        detections = []
        for r in predictions:
            if r.boxes is not None:
                boxes_data = r.boxes.data.cpu().numpy()

                for *xyxy, conf, cls in boxes_data:
                    x1, y1, x2, y2 = map(int, xyxy)
                    confidence = float(conf)
                    class_id = int(cls)

                    # Ensure coordinates are within image bounds
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(original_shape[1], x2)
                    y2 = min(original_shape[0], y2)

                    if x2 <= x1 or y2 <= y1:
                        self.get_logger().warn(f"Invalid bounding box coordinates: [{x1}, {y1}, {x2}, {y2}]")
                        continue

                    label = self.class_names[class_id] if class_id < len(self.class_names) else str(class_id)
                    detections.append({
                        'box': [x1, y1, x2, y2],
                        'confidence': confidence,
                        'class_id': class_id,
                        'label': label
                    })
        return detections

    def draw_detections(self, image, detections):
        for det in detections:
            x1, y1, x2, y2 = det['box']
            label = det.get('label', str(det.get('class_id', '')))
            confidence = det.get('confidence', 0)
            color = (0, 255, 0) # Green color for bounding box

            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            display_text = f"{label} {confidence:.2f}"

            # Adjust font scale and thickness based on bounding box size for better readability
            font_scale = max(0.4, min(1.0, (x2 - x1) / 200.0))
            font_thickness = max(1, int(font_scale * 2))

            cv2.putText(image, display_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, font_thickness)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()