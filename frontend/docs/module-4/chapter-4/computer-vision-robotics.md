---
sidebar_position: 4
title: 'Computer Vision for Robotics'
---

# Computer Vision for Robotics

This chapter covers computer vision techniques specifically designed for robotics applications, including object detection, tracking, recognition, and 3D scene understanding that enable robots to perceive and interact with their environment.

## What You'll Learn

In this chapter, you'll explore:
- Robot vision systems and camera calibration
- Object detection and recognition for robotics
- Visual tracking and SLAM
- 3D reconstruction and depth estimation
- Vision-based manipulation
- Real-time vision optimization

## Prerequisites

- Completion of Module 1-4, Chapters 1-3
- Understanding of ROS 2 messaging
- Basic knowledge of image processing
- Experience with Python and OpenCV
- Understanding of coordinate systems and transformations

## Robot Vision Systems and Camera Calibration

### Camera Calibration for Robotics

Camera calibration is crucial for accurate vision-based robotics applications:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for camera info
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Calibration parameters
        self.calibration_data = {
            'camera_matrix': None,
            'distortion_coeffs': None,
            'rvecs': None,
            'tvecs': None
        }

        # Chessboard parameters for calibration
        self.chessboard_size = (9, 6)  # Internal corners
        self.square_size = 0.025  # 2.5cm squares

        # Calibration images storage
        self.calibration_images = []
        self.max_calibration_images = 20

        # Threading for calibration
        self.calibration_lock = threading.Lock()

        self.get_logger().info('Camera Calibrator initialized')

    def image_callback(self, msg):
        """Process camera image for calibration"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Check if this image is suitable for calibration
            if self.is_good_calibration_image(cv_image):
                with self.calibration_lock:
                    if len(self.calibration_images) < self.max_calibration_images:
                        self.calibration_images.append(cv_image.copy())
                        self.get_logger().info(f'Added calibration image {len(self.calibration_images)}/{self.max_calibration_images}')

                        # Perform calibration when we have enough images
                        if len(self.calibration_images) == self.max_calibration_images:
                            self.perform_calibration()

        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')

    def is_good_calibration_image(self, image):
        """Check if image is suitable for calibration"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, self.chessboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        # Check if corners were found and image quality
        if ret:
            # Refine corner positions
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            return True

        return False

    def perform_calibration(self):
        """Perform camera calibration"""
        self.get_logger().info('Starting camera calibration...')

        # Prepare object points
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size  # Scale to real-world units

        # Arrays to store object points and image points from all images
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane

        # Process calibration images
        for img in self.calibration_images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            if ret:
                objpoints.append(objp)

                # Refine corner positions
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                imgpoints.append(corners_refined)

        if len(objpoints) > 0 and len(imgpoints) > 0:
            # Perform calibration
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )

            if ret:
                # Store calibration data
                self.calibration_data = {
                    'camera_matrix': camera_matrix,
                    'distortion_coeffs': dist_coeffs,
                    'rvecs': rvecs,
                    'tvecs': tvecs
                }

                # Publish camera info
                self.publish_camera_info(camera_matrix, dist_coeffs, img.shape)

                self.get_logger().info(f'Calibration successful! Reprojection error: {ret:.4f}')
                self.get_logger().info(f'Camera matrix:\n{camera_matrix}')
                self.get_logger().info(f'Distortion coefficients: {dist_coeffs.flatten()}')

            else:
                self.get_logger().error('Calibration failed')
        else:
            self.get_logger().error('No valid calibration patterns found')

    def publish_camera_info(self, camera_matrix, dist_coeffs, img_shape):
        """Publish camera calibration info"""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = "camera_link"

        info_msg.height = img_shape[0]
        info_msg.width = img_shape[1]

        # Camera matrix (3x3) as 9-element array
        info_msg.k = camera_matrix.flatten().tolist()

        # Distortion coefficients
        info_msg.d = dist_coeffs.flatten().tolist()

        # Rectification matrix (identity for monocular camera)
        info_msg.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]

        # Projection matrix
        info_msg.p = [
            camera_matrix[0, 0], 0, camera_matrix[0, 2], 0,
            0, camera_matrix[1, 1], camera_matrix[1, 2], 0,
            0, 0, 1, 0
        ]

        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down camera calibrator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Object Detection and Recognition

### YOLO-based Object Detection for Robotics

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class YOLOObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(String, '/object_detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection_markers', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # YOLO parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4

        # Load YOLO model (for this example, we'll use a mock detector)
        # In practice, you would load a real YOLO model
        try:
            # self.net = cv2.dnn.readNetFromDarknet('yolo.cfg', 'yolo.weights')
            # self.layer_names = self.net.getLayerNames()
            # self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
            self.yolo_loaded = True
            self.get_logger().info('YOLO model loaded')
        except:
            self.yolo_loaded = False
            self.get_logger().warn('YOLO model not available, using mock detector')

        # COCO class names
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Threading
        self.detection_lock = threading.Lock()

        self.get_logger().info('YOLO Object Detector initialized')

    def image_callback(self, msg):
        """Process image for object detection"""
        with self.detection_lock:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                if self.yolo_loaded:
                    detections = self.detect_objects_yolo(cv_image)
                else:
                    # Mock detection for simulation
                    detections = self.mock_detection(cv_image)

                # Draw detections on image (optional)
                output_image = self.draw_detections(cv_image, detections)

                # Publish detections
                self.publish_detections(detections, msg.header)

                # Publish visualization markers
                self.publish_detection_markers(detections, msg.header)

            except Exception as e:
                self.get_logger().error(f'Image callback error: {str(e)}')

    def mock_detection(self, image):
        """Mock object detection for simulation"""
        height, width = image.shape[:2]

        # Create some mock detections
        mock_detections = [
            {
                'label': 'person',
                'confidence': 0.85,
                'bbox': [int(width * 0.4), int(height * 0.3), int(width * 0.6), int(height * 0.7)],
                'center': [int(width * 0.5), int(height * 0.5)]
            },
            {
                'label': 'chair',
                'confidence': 0.78,
                'bbox': [int(width * 0.2), int(height * 0.5), int(width * 0.35), int(height * 0.8)],
                'center': [int(width * 0.275), int(height * 0.65)]
            }
        ]

        return mock_detections

    def detect_objects_yolo(self, image):
        """Perform object detection using YOLO"""
        height, width, channels = image.shape

        # Create blob from image
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Process outputs
        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > self.confidence_threshold:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)

        # Format detections
        detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                detections.append({
                    'label': self.class_names[class_ids[i]] if class_ids[i] < len(self.class_names) else 'unknown',
                    'confidence': confidences[i],
                    'bbox': [x, y, x + w, y + h],
                    'center': [x + w // 2, y + h // 2]
                })

        return detections

    def draw_detections(self, image, detections):
        """Draw detection results on image"""
        output_image = image.copy()

        for detection in detections:
            x1, y1, x2, y2 = map(int, detection['bbox'])
            label = f"{detection['label']}: {detection['confidence']:.2f}"

            # Draw bounding box
            cv2.rectangle(output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw center point
            center_x, center_y = detection['center']
            cv2.circle(output_image, (center_x, center_y), 5, (0, 0, 255), -1)

            # Draw label
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(output_image, (x1, y1 - label_size[1] - 10), (x1 + label_size[0], y1), (0, 255, 0), cv2.FILLED)
            cv2.putText(output_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        return output_image

    def publish_detections(self, detections, header):
        """Publish object detections"""
        detection_data = []
        for detection in detections:
            detection_data.append({
                'label': detection['label'],
                'confidence': detection['confidence'],
                'bbox': detection['bbox'],
                'center': detection['center']
            })

        detection_msg = String()
        detection_msg.data = str(detection_data)
        self.detection_pub.publish(detection_msg)

    def publish_detection_markers(self, detections, header):
        """Publish visualization markers for detections"""
        marker_array = MarkerArray()

        for i, detection in enumerate(detections):
            # Create marker for bounding box
            marker = Marker()
            marker.header = header
            marker.ns = "object_detections"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Set position (will be set properly in a real implementation with depth)
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0

            # Set scale
            marker.scale.x = 0.02  # Line width

            # Set color based on confidence
            confidence = detection['confidence']
            marker.color.r = 1.0 - confidence  # Red decreases with confidence
            marker.color.g = confidence  # Green increases with confidence
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Define points for the bounding box
            x1, y1, x2, y2 = detection['bbox']
            points = [
                Point(x=x1, y=y1, z=0),
                Point(x=x2, y=y1, z=0),
                Point(x=x2, y=y2, z=0),
                Point(x=x1, y=y2, z=0),
                Point(x=x1, y=y1, z=0)  # Close the loop
            ]

            marker.points = points
            marker_array.markers.append(marker)

            # Create text marker for label
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "detection_labels"
            text_marker.id = i + len(detections)  # Different ID to avoid conflict
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = x1
            text_marker.pose.position.y = y1 - 10
            text_marker.pose.position.z = 1.0
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.2  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"{detection['label']} ({detection['confidence']:.2f})"

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down YOLO object detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visual Tracking and SLAM

### Feature-Based Visual Tracking

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class FeatureTracker(Node):
    def __init__(self):
        super().__init__('feature_tracker')

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.tracked_point_pub = self.create_publisher(PointStamped, '/tracked_point', 10)
        self.tracking_quality_pub = self.create_publisher(Float32, '/tracking_quality', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Tracking parameters
        self.feature_params = {
            'maxCorners': 100,
            'qualityLevel': 0.3,
            'minDistance': 7,
            'blockSize': 7
        }

        self.lk_params = {
            'winSize': (15, 15),
            'maxLevel': 2,
            'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        }

        # Tracking state
        self.prev_image = None
        self.prev_points = None
        self.track_quality = 0.0
        self.track_id = 0

        # Threading lock
        self.tracking_lock = threading.Lock()

        self.get_logger().info('Feature Tracker initialized')

    def image_callback(self, msg):
        """Process image for feature tracking"""
        with self.tracking_lock:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

                if self.prev_image is not None and self.prev_points is not None:
                    # Calculate optical flow
                    new_points, status, error = cv2.calcOpticalFlowPyrLK(
                        self.prev_image, cv_image, self.prev_points, None, **self.lk_params
                    )

                    # Select good points
                    good_new = new_points[status == 1]
                    good_old = self.prev_points[status == 1]

                    if len(good_new) > 0:
                        # Calculate tracking quality
                        self.track_quality = len(good_new) / len(self.prev_points)

                        # Update tracked point (use the first tracked point as example)
                        tracked_point = good_new[0]

                        point_msg = PointStamped()
                        point_msg.header = msg.header
                        point_msg.point.x = float(tracked_point[0])
                        point_msg.point.y = float(tracked_point[1])
                        point_msg.point.z = 0.0  # Depth would come from stereo or depth sensor

                        self.tracked_point_pub.publish(point_msg)

                        # Publish tracking quality
                        quality_msg = Float32()
                        quality_msg.data = self.track_quality
                        self.tracking_quality_pub.publish(quality_msg)

                        # Update points for next iteration
                        self.prev_points = good_new.reshape(-1, 1, 2)

                        self.get_logger().info(f'Tracked {len(good_new)} points, quality: {self.track_quality:.2f}')
                    else:
                        # Lost track, need to reinitialize
                        self.prev_points = None
                        self.track_quality = 0.0
                        self.get_logger().info('Lost track, reinitializing...')

                else:
                    # Initialize tracking points
                    self.prev_points = cv2.goodFeaturesToTrack(
                        cv_image, mask=None, **self.feature_params
                    )
                    if self.prev_points is not None:
                        self.get_logger().info(f'Initialized {len(self.prev_points)} tracking points')

                # Store current image for next iteration
                self.prev_image = cv_image

            except Exception as e:
                self.get_logger().error(f'Tracking error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FeatureTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down feature tracker...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3D Reconstruction and Depth Estimation

### Stereo Vision and 3D Reconstruction

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class StereoReconstruction(Node):
    def __init__(self):
        super().__init__('stereo_reconstruction')

        # Subscribe to stereo images
        self.left_sub = self.create_subscription(
            Image,
            '/stereo/left/image_rect',
            self.left_image_callback,
            10
        )

        self.right_sub = self.create_subscription(
            Image,
            '/stereo/right/image_rect',
            self.right_image_callback,
            10
        )

        # Publishers
        self.point_pub = self.create_publisher(PointStamped, '/reconstructed_point', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Stereo parameters
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*10,  # Must be divisible by 16
            blockSize=5,
            P1=8 * 3 * 5**2,
            P2=32 * 3 * 5**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # Camera parameters (these should be calibrated)
        self.camera_matrix_left = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        self.camera_matrix_right = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Baseline (distance between cameras) - example value
        self.baseline = 0.12  # meters

        # Image storage
        self.left_image = None
        self.right_image = None
        self.images_lock = threading.Lock()

        self.get_logger().info('Stereo Reconstruction initialized')

    def left_image_callback(self, msg):
        """Process left camera image"""
        with self.images_lock:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

    def right_image_callback(self, msg):
        """Process right camera image"""
        with self.images_lock:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

    def reconstruct_3d_point(self, u, v, disparity):
        """Reconstruct 3D point from disparity"""
        if disparity == 0:
            return None

        # Calculate 3D coordinates
        x = (u - self.camera_matrix_left[0, 2]) * self.baseline / disparity
        y = (v - self.camera_matrix_left[1, 2]) * self.baseline / disparity
        z = self.camera_matrix_left[0, 0] * self.baseline / disparity

        return np.array([x, y, z])

    def process_stereo_pair(self):
        """Process stereo image pair for 3D reconstruction"""
        with self.images_lock:
            if self.left_image is None or self.right_image is None:
                return

            # Compute disparity map
            disparity = self.stereo.compute(self.left_image, self.right_image).astype(np.float32) / 16.0

            # Find a point of interest (for example, center of image)
            height, width = disparity.shape
            center_u, center_v = width // 2, height // 2

            # Get disparity at center point
            center_disparity = disparity[center_v, center_u]

            if center_disparity > 0:
                # Reconstruct 3D point
                point_3d = self.reconstruct_3d_point(center_u, center_v, center_disparity)

                if point_3d is not None:
                    # Publish reconstructed point
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = "camera_link"
                    point_msg.point.x = float(point_3d[0])
                    point_msg.point.y = float(point_3d[1])
                    point_msg.point.z = float(point_3d[2])

                    self.point_pub.publish(point_msg)

                    self.get_logger().info(f'Reconstructed 3D point: ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = StereoReconstruction()

    # Timer to process stereo pairs
    timer = node.create_timer(0.1, node.process_stereo_pair)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down stereo reconstruction...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Vision-Based Manipulation

### Object Manipulation with Vision Feedback

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class VisionBasedManipulator(Node):
    def __init__(self):
        super().__init__('vision_based_manipulator')

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, '/manipulation_target', 10)
        self.manipulation_status_pub = self.create_publisher(String, '/manipulation_status', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Manipulation state
        self.current_task = None
        self.target_object = None
        self.object_position = None

        # Object detection parameters
        self.object_templates = {}  # Would store template images for matching
        self.detection_threshold = 0.7

        # Threading
        self.manipulation_lock = threading.Lock()

        self.get_logger().info('Vision-Based Manipulator initialized')

    def image_callback(self, msg):
        """Process image for manipulation tasks"""
        with self.manipulation_lock:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                # If we have a manipulation task, look for the target object
                if self.current_task and self.target_object:
                    object_found, position = self.find_object(cv_image, self.target_object)

                    if object_found and position:
                        self.object_position = position
                        self.publish_target_pose(position, msg.header)
                        self.get_logger().info(f'Found {self.target_object} at {position}')

                        # Check if object is in manipulable position
                        if self.is_object_manipulable(position):
                            self.execute_manipulation(position)
                    else:
                        self.get_logger().info(f'Could not find {self.target_object}')

            except Exception as e:
                self.get_logger().error(f'Manipulation error: {str(e)}')

    def find_object(self, image, object_name):
        """Find specific object in image"""
        # For this example, we'll use color-based detection
        # In practice, you might use template matching, feature detection, or deep learning

        if object_name == 'red_cup':
            return self.find_red_object(image)
        elif object_name == 'blue_box':
            return self.find_blue_object(image)
        else:
            # Generic object detection could go here
            return self.generic_object_detection(image, object_name)

    def find_red_object(self, image):
        """Find red-colored objects in image"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assuming it's the target object)
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                # Get the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return True, (cx, cy)

        return False, None

    def find_blue_object(self, image):
        """Find blue-colored objects in image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for blue color
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Create mask for blue color
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 100:
                # Get the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return True, (cx, cy)

        return False, None

    def generic_object_detection(self, image, object_name):
        """Generic object detection method"""
        # This would typically use a pre-trained model
        # For simulation, return a fixed position
        height, width = image.shape[:2]
        return True, (width // 2, height // 2)  # Center of image

    def is_object_manipulable(self, position):
        """Check if object is in manipulable position"""
        # In a real system, this would check if the object is reachable by the robot arm
        # For simulation, assume object at center is manipulable
        if position:
            height, width = 480, 640  # Assuming standard image size
            center_x, center_y = width // 2, height // 2

            # Check if object is roughly in the center (within 10% of image)
            x, y = position
            tolerance = 0.1  # 10% tolerance
            return (abs(x - center_x) < width * tolerance and
                    abs(y - center_y) < height * tolerance)

        return False

    def publish_target_pose(self, position, header):
        """Publish target pose for manipulation"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "camera_link"

        # Convert image coordinates to 3D pose
        # This is simplified - in reality, you'd need depth information
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = 0.0  # Depth would come from other sensors

        # Set orientation (for manipulation, you might want to approach from above)
        pose_msg.pose.orientation.w = 1.0

        self.target_pose_pub.publish(pose_msg)

    def execute_manipulation(self, position):
        """Execute manipulation task"""
        self.get_logger().info(f'Executing manipulation for object at {position}')

        # Publish manipulation status
        status_msg = String()
        status_msg.data = f"manipulating_object_at_{position[0]}_{position[1]}"
        self.manipulation_status_pub.publish(status_msg)

        # In a real system, this would send commands to the robot arm
        # For simulation, just log the action
        self.get_logger().info('Manipulation command sent to robot arm')

    def start_manipulation_task(self, object_name):
        """Start a manipulation task"""
        with self.manipulation_lock:
            self.current_task = 'manipulation'
            self.target_object = object_name
            self.object_position = None
            self.get_logger().info(f'Started manipulation task for {object_name}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionBasedManipulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down vision-based manipulator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-time Vision Optimization

### Optimized Vision Processing Pipeline

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
from collections import deque

class OptimizedVisionPipeline(Node):
    def __init__(self):
        super().__init__('optimized_vision_pipeline')

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers for performance metrics
        self.fps_pub = self.create_publisher(Float32, '/vision_fps', 10)
        self.processing_time_pub = self.create_publisher(Float32, '/processing_time', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Processing parameters
        self.target_fps = 30.0
        self.processing_rate = 1  # Process every Nth frame
        self.frame_counter = 0

        # Performance tracking
        self.frame_times = deque(maxlen=30)  # Track last 30 frames
        self.last_frame_time = time.time()

        # Threading
        self.processing_queue = deque(maxlen=2)  # Limit queue size
        self.processing_thread = threading.Thread(target=self.processing_worker, daemon=True)
        self.processing_thread.start()

        # Processing flags
        self.enable_detection = True
        self.enable_tracking = True
        self.enable_optimization = True

        # Optimization parameters
        self.image_scale_factor = 0.5  # Scale down for faster processing
        self.roi_enabled = False
        self.roi = (0, 0, 1, 1)  # (x, y, width, height) as fraction of image

        # Performance timer
        self.perf_timer = self.create_timer(1.0, self.publish_performance_metrics)

        self.get_logger().info('Optimized Vision Pipeline initialized')

    def image_callback(self, msg):
        """Process incoming image with optimization"""
        # Throttle processing rate
        self.frame_counter += 1
        if self.frame_counter % self.processing_rate != 0:
            return

        # Add to processing queue (non-blocking)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Apply optimizations
            if self.enable_optimization:
                cv_image = self.apply_optimizations(cv_image)

            self.processing_queue.append((cv_image, msg.header))
        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')

    def apply_optimizations(self, image):
        """Apply various optimizations to image"""
        # Scale image down for faster processing
        if self.image_scale_factor < 1.0:
            new_size = (int(image.shape[1] * self.image_scale_factor),
                       int(image.shape[0] * self.image_scale_factor))
            image = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)

        # Apply ROI if enabled
        if self.roi_enabled:
            h, w = image.shape[:2]
            x1 = int(self.roi[0] * w)
            y1 = int(self.roi[1] * h)
            x2 = int((self.roi[0] + self.roi[2]) * w)
            y2 = int((self.roi[1] + self.roi[3]) * h)
            image = image[y1:y2, x1:x2]

        return image

    def processing_worker(self):
        """Background processing worker"""
        while rclpy.ok():
            try:
                if self.processing_queue:
                    image, header = self.processing_queue.popleft()

                    # Record start time for performance tracking
                    start_time = time.time()

                    # Perform vision processing
                    if self.enable_detection:
                        processed_image = self.perform_detection(image)
                    else:
                        processed_image = image

                    # Record processing time
                    processing_time = time.time() - start_time
                    self.frame_times.append(processing_time)

                    # Adjust processing rate based on performance
                    self.adjust_processing_rate(processing_time)

            except IndexError:
                # Queue is empty, sleep briefly
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'Processing worker error: {str(e)}')

    def perform_detection(self, image):
        """Perform vision detection tasks"""
        # This is where you'd implement your specific vision algorithms
        # For example: object detection, feature extraction, etc.

        # Simple example: edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to color for consistency
        processed = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        return processed

    def adjust_processing_rate(self, processing_time):
        """Adjust processing rate based on performance"""
        if len(self.frame_times) < 10:
            return

        avg_processing_time = sum(self.frame_times) / len(self.frame_times)
        current_fps = 1.0 / avg_processing_time if avg_processing_time > 0 else 0

        # Adjust processing rate to maintain target FPS
        if current_fps > self.target_fps * 1.1:  # Too fast, can process more
            self.processing_rate = max(1, self.processing_rate - 1)
        elif current_fps < self.target_fps * 0.9:  # Too slow, process less
            self.processing_rate = min(10, self.processing_rate + 1)

    def publish_performance_metrics(self):
        """Publish performance metrics"""
        if self.frame_times:
            avg_processing_time = sum(self.frame_times) / len(self.frame_times)
            current_fps = 1.0 / avg_processing_time if avg_processing_time > 0 else 0

            # Publish FPS
            fps_msg = Float32()
            fps_msg.data = current_fps
            self.fps_pub.publish(fps_msg)

            # Publish processing time
            time_msg = Float32()
            time_msg.data = avg_processing_time
            self.processing_time_pub.publish(time_msg)

            self.get_logger().info(f'Vision pipeline - FPS: {current_fps:.2f}, Processing time: {avg_processing_time:.4f}s, Rate: {self.processing_rate}')

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedVisionPipeline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down optimized vision pipeline...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Complete Vision System

In this lab, you'll integrate all vision components into a complete system.

### Step 1: Create the Vision System Launch File

Create `vision_system_launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Vision system nodes
    calibrator = Node(
        package='ai_robo_learning',
        executable='camera_calibrator',
        name='camera_calibrator',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    detector = Node(
        package='ai_robo_learning',
        executable='yolo_object_detector',
        name='yolo_object_detector',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tracker = Node(
        package='ai_robo_learning',
        executable='feature_tracker',
        name='feature_tracker',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    stereo = Node(
        package='ai_robo_learning',
        executable='stereo_reconstruction',
        name='stereo_reconstruction',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    manipulator = Node(
        package='ai_robo_learning',
        executable='vision_based_manipulator',
        name='vision_based_manipulator',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    optimizer = Node(
        package='ai_robo_learning',
        executable='optimized_vision_pipeline',
        name='optimized_vision_pipeline',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return launch description
    ld = LaunchDescription()

    # Add all nodes
    ld.add_action(calibrator)
    ld.add_action(detector)
    ld.add_action(tracker)
    ld.add_action(stereo)
    ld.add_action(manipulator)
    ld.add_action(optimizer)

    return ld
```

### Step 2: Create the Complete Vision Node

Create `complete_vision_system.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
from collections import deque

class CompleteVisionSystem(Node):
    def __init__(self):
        super().__init__('complete_vision_system')

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(String, '/vision_detections', 10)
        self.tracking_pub = self.create_publisher(PointStamped, '/tracked_object', 10)
        self.status_pub = self.create_publisher(String, '/vision_status', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Vision system components
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.current_image = None

        # Feature tracking
        self.prev_image = None
        self.prev_points = None
        self.feature_params = {
            'maxCorners': 100,
            'qualityLevel': 0.3,
            'minDistance': 7,
            'blockSize': 7
        }

        self.lk_params = {
            'winSize': (15, 15),
            'maxLevel': 2,
            'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        }

        # Object detection (simplified)
        self.object_detection_enabled = True
        self.tracking_enabled = True

        # Performance tracking
        self.frame_times = deque(maxlen=30)
        self.last_frame_time = time.time()

        # Threading
        self.vision_lock = threading.Lock()

        # System status
        self.system_active = True
        self.vision_quality = 0.0

        self.get_logger().info('Complete Vision System initialized')

    def image_callback(self, msg):
        """Process camera image with all vision components"""
        with self.vision_lock:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                # Record frame time for performance tracking
                current_time = time.time()
                if self.last_frame_time > 0:
                    frame_time = current_time - self.last_frame_time
                    self.frame_times.append(frame_time)
                self.last_frame_time = current_time

                # Update current image
                self.current_image = cv_image

                # Perform object detection
                if self.object_detection_enabled:
                    detections = self.simple_object_detection(cv_image)
                    if detections:
                        detection_msg = String()
                        detection_msg.data = str(detections)
                        self.detection_pub.publish(detection_msg)

                # Perform feature tracking
                if self.tracking_enabled:
                    self.perform_tracking(cv_image, msg.header)

                # Update vision quality metric
                self.update_vision_quality()

                # Publish system status
                status_msg = String()
                status_msg.data = f"active:{self.system_active},quality:{self.vision_quality:.2f}"
                self.status_pub.publish(status_msg)

            except Exception as e:
                self.get_logger().error(f'Vision system error: {str(e)}')

    def simple_object_detection(self, image):
        """Simple object detection for demonstration"""
        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define ranges for common colors
        color_ranges = {
            'red': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            'blue': (np.array([100, 50, 50]), np.array([130, 255, 255])),
            'green': (np.array([40, 50, 50]), np.array([80, 255, 255]))
        }

        detections = []
        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    detections.append({
                        'color': color_name,
                        'bbox': [x, y, x + w, y + h],
                        'center': [center_x, center_y],
                        'area': cv2.contourArea(contour)
                    })

        return detections

    def perform_tracking(self, image, header):
        """Perform feature tracking"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.prev_image is not None and self.prev_points is not None:
            # Calculate optical flow
            new_points, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_image, gray, self.prev_points, None, **self.lk_params
            )

            # Select good points
            good_new = new_points[status == 1]
            good_old = self.prev_points[status == 1]

            if len(good_new) > 0:
                # Publish the first tracked point as example
                tracked_point = good_new[0]

                point_msg = PointStamped()
                point_msg.header = header
                point_msg.point.x = float(tracked_point[0])
                point_msg.point.y = float(tracked_point[1])
                point_msg.point.z = 0.0  # Depth would come from other sensors

                self.tracking_pub.publish(point_msg)

                # Update points for next iteration
                self.prev_points = good_new.reshape(-1, 1, 2)
            else:
                # Lost track, reinitialize
                self.prev_points = None
        else:
            # Initialize tracking points
            self.prev_points = cv2.goodFeaturesToTrack(
                gray, mask=None, **self.feature_params
            )

        # Store current image for next iteration
        self.prev_image = gray

    def update_vision_quality(self):
        """Update vision system quality metric"""
        if len(self.frame_times) > 0:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            current_fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

            # Quality metric based on FPS (target 30 FPS)
            self.vision_quality = min(1.0, current_fps / 30.0)
        else:
            self.vision_quality = 0.0

    def get_vision_status(self):
        """Get current vision system status"""
        return {
            'active': self.system_active,
            'quality': self.vision_quality,
            'fps': 1.0 / self.frame_times[-1] if self.frame_times else 0,
            'detection_enabled': self.object_detection_enabled,
            'tracking_enabled': self.tracking_enabled
        }

def main(args=None):
    rclpy.init(args=args)
    node = CompleteVisionSystem()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down complete vision system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test the Vision System

1. Make sure you have the required dependencies:
```bash
pip3 install opencv-python numpy
```

2. Run the complete vision system:
```bash
python3 complete_vision_system.py
```

3. Test with a camera feed or simulated images:
```bash
# If using a real camera
ros2 run v4l2_camera v4l2_camera_node

# Or use a sample image publisher
ros2 run image_publisher image_publisher_node --ros-args -p file_name:=/path/to/test/image.jpg
```

## Best Practices

1. **Calibration**: Always calibrate cameras before using vision algorithms
2. **Performance**: Optimize algorithms for real-time performance
3. **Robustness**: Handle various lighting conditions and environments
4. **Validation**: Continuously validate vision results against ground truth
5. **Modularity**: Design vision components to be modular and reusable
6. **Error Handling**: Implement robust error handling for vision failures
7. **Resource Management**: Efficiently manage computational resources
8. **Testing**: Thoroughly test vision systems in various scenarios

## Next Steps

After completing this chapter, you'll be ready to learn about object manipulation in Chapter 5, where you'll explore how robots can interact with and manipulate objects in their environment.