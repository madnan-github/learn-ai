---
sidebar_position: 6
title: 'Perception Systems'
---

# Perception Systems

This chapter covers advanced perception systems for humanoid robots, including computer vision, depth sensing, object recognition, and environment understanding that enable autonomous navigation and interaction.

## What You'll Learn

In this chapter, you'll explore:
- Multi-sensor fusion for humanoid perception
- Computer vision algorithms for object detection
- Depth estimation and 3D reconstruction
- SLAM for environment mapping
- Object recognition and scene understanding
- Real-time perception optimization

## Prerequisites

- Completion of Module 1-3, Chapters 1-5
- Understanding of sensor integration
- Basic computer vision knowledge
- Experience with ROS 2 message types
- Knowledge of machine learning concepts

## Multi-Sensor Fusion for Humanoid Perception

### Sensor Fusion Architecture

Humanoid robots typically use multiple sensors that need to be fused for robust perception:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2, Imu, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from collections import deque
import threading

class MultiSensorFusionNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to all sensors
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )

        # Publishers
        self.fused_data_pub = self.create_publisher(Float32MultiArray, '/fused_sensor_data', 10)
        self.environment_map_pub = self.create_publisher(PointCloud2, '/environment_map', 10)

        # Sensor data storage
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None
        self.depth_data = None
        self.camera_info = None

        # Sensor fusion state
        self.sensor_buffer = deque(maxlen=10)
        self.fusion_lock = threading.Lock()

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        self.get_logger().info('Multi-Sensor Fusion Node initialized')

    def camera_callback(self, msg):
        """Process camera data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = {
                'image': cv_image,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            self.process_sensor_fusion()
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {str(e)}')

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        try:
            ranges = np.array(msg.ranges)
            angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(ranges))])

            # Filter valid ranges
            valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (~np.isnan(ranges)) & (~np.isinf(ranges))
            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]

            # Convert to Cartesian coordinates
            x_points = valid_ranges * np.cos(valid_angles)
            y_points = valid_ranges * np.sin(valid_angles)

            self.lidar_data = {
                'x': x_points,
                'y': y_points,
                'ranges': valid_ranges,
                'angles': valid_angles,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            self.process_sensor_fusion()
        except Exception as e:
            self.get_logger().error(f'LiDAR callback error: {str(e)}')

    def imu_callback(self, msg):
        """Process IMU data"""
        try:
            self.imu_data = {
                'orientation': [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ],
                'angular_velocity': [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ],
                'linear_acceleration': [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ],
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            self.process_sensor_fusion()
        except Exception as e:
            self.get_logger().error(f'IMU callback error: {str(e)}')

    def depth_callback(self, msg):
        """Process depth data"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.depth_data = {
                'image': depth_image,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            self.process_sensor_fusion()
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {str(e)}')

    def info_callback(self, msg):
        """Process camera info"""
        try:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
        except Exception as e:
            self.get_logger().error(f'Camera info callback error: {str(e)}')

    def process_sensor_fusion(self):
        """Process and fuse sensor data"""
        with self.fusion_lock:
            # Check if we have all necessary sensor data
            if not all([self.camera_data, self.lidar_data, self.imu_data, self.depth_data]):
                return

            # Perform sensor fusion
            fused_data = self.fuse_sensors()

            # Publish fused data
            if fused_data is not None:
                fused_msg = Float32MultiArray()
                fused_msg.data = fused_data.flatten().tolist()
                self.fused_data_pub.publish(fused_msg)

    def fuse_sensors(self):
        """Fuse data from multiple sensors"""
        try:
            # Create a unified representation of the environment
            fused_data = np.zeros((100, 6))  # [x, y, z, r, g, b] for points

            # Process LiDAR data to get 2D points
            lidar_points = np.column_stack((self.lidar_data['x'], self.lidar_data['y']))

            # Process depth data to get 3D points (if camera matrix is available)
            if self.camera_matrix is not None and self.depth_data is not None:
                depth_points = self.process_depth_to_3d()

                # Combine LiDAR and depth data
                # This is a simplified approach - in practice, you'd use more sophisticated fusion
                for i, (x, y) in enumerate(lidar_points[:min(len(lidar_points), len(depth_points))]):
                    fused_data[i, 0] = x  # x coordinate
                    fused_data[i, 1] = y  # y coordinate
                    fused_data[i, 2] = depth_points[i, 2] if i < len(depth_points) else 0  # z coordinate
                    fused_data[i, 3] = 0.5  # dummy red value
                    fused_data[i, 4] = 0.5  # dummy green value
                    fused_data[i, 5] = 0.5  # dummy blue value

            return fused_data

        except Exception as e:
            self.get_logger().error(f'Sensor fusion error: {str(e)}')
            return None

    def process_depth_to_3d(self):
        """Convert depth image to 3D points"""
        try:
            depth_image = self.depth_data['image']
            height, width = depth_image.shape

            # Generate coordinate grids
            u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))
            u_coords = u_coords.astype(np.float32)
            v_coords = v_coords.astype(np.float32)

            # Apply camera matrix to convert to 3D
            if self.camera_matrix is not None:
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]

                # Convert to 3D coordinates
                x_coords = (u_coords - cx) * depth_image / fx
                y_coords = (v_coords - cy) * depth_image / fy
                z_coords = depth_image

                # Flatten and stack
                points_3d = np.column_stack((
                    x_coords.flatten(),
                    y_coords.flatten(),
                    z_coords.flatten()
                ))

                # Filter out invalid points (where depth is 0 or invalid)
                valid_mask = (z_coords.flatten() > 0) & (np.isfinite(z_coords.flatten()))
                points_3d = points_3d[valid_mask]

                return points_3d

        except Exception as e:
            self.get_logger().error(f'Depth to 3D conversion error: {str(e)}')

        return np.zeros((0, 3))  # Return empty array if conversion fails

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Computer Vision for Object Detection

### Object Detection Pipeline

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(Image, '/detection_output', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection_markers', 10)

        # Object detection parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4

        # Load YOLO model (simplified - in practice, use a real model)
        # For this example, we'll use OpenCV's DNN module with a pre-trained model
        try:
            # In a real implementation, you would load a model like this:
            # self.net = cv2.dnn.readNetFromDarknet('yolo.cfg', 'yolo.weights')
            # self.layer_names = self.net.getLayerNames()
            # self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

            # For simulation, we'll use a mock detector
            self.detector_available = False
            self.get_logger().info('Object detection model not loaded (simulation mode)')
        except:
            self.detector_available = False
            self.get_logger().info('Object detection model not available (using mock)')

        # Class names for detection
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

        self.detection_lock = threading.Lock()

        self.get_logger().info('Object Detection Node initialized')

    def image_callback(self, msg):
        """Process incoming camera image for object detection"""
        with self.detection_lock:
            try:
                # Convert ROS Image to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                # Perform object detection
                if self.detector_available:
                    detections = self.detect_objects(cv_image)
                else:
                    # Mock detection for simulation
                    detections = self.mock_detection(cv_image)

                # Draw detections on image
                output_image = self.draw_detections(cv_image, detections)

                # Publish detection output
                output_msg = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
                output_msg.header = msg.header
                self.detection_pub.publish(output_msg)

                # Publish visualization markers
                self.publish_detection_markers(detections, msg.header)

            except Exception as e:
                self.get_logger().error(f'Error in object detection: {str(e)}')

    def mock_detection(self, image):
        """Mock object detection for simulation"""
        # In a real system, this would be replaced with actual detection
        # For simulation, we'll create some mock detections
        height, width = image.shape[:2]

        # Create some mock detections (person, chair, bottle)
        mock_detections = [
            {
                'label': 'person',
                'confidence': 0.85,
                'bbox': [width * 0.4, height * 0.3, width * 0.6, height * 0.7]
            },
            {
                'label': 'chair',
                'confidence': 0.78,
                'bbox': [width * 0.2, height * 0.5, width * 0.35, height * 0.8]
            },
            {
                'label': 'bottle',
                'confidence': 0.65,
                'bbox': [width * 0.7, height * 0.6, width * 0.75, height * 0.8]
            }
        ]

        return mock_detections

    def detect_objects(self, image):
        """Perform object detection using loaded model"""
        # This is where the actual detection would happen
        # Using OpenCV DNN module with YOLO
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
                    'bbox': [x, y, x + w, y + h]
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

            # Draw label
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(output_image, (x1, y1 - label_size[1] - 10), (x1 + label_size[0], y1), (0, 255, 0), cv2.FILLED)
            cv2.putText(output_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        return output_image

    def publish_detection_markers(self, detections, header):
        """Publish visualization markers for detections"""
        from visualization_msgs.msg import MarkerArray, Marker
        from geometry_msgs.msg import Point

        marker_array = MarkerArray()

        for i, detection in enumerate(detections):
            # Create marker for bounding box
            marker = Marker()
            marker.header = header
            marker.ns = "object_detections"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Set position (simplified - in real system, would use depth to get 3D position)
            marker.pose.position.z = 1.0  # Default height
            marker.pose.orientation.w = 1.0

            # Set scale
            marker.scale.x = 0.05  # Line width

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
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Estimation and 3D Reconstruction

### Depth Processing Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField
import struct

class DepthProcessingNode(Node):
    def __init__(self):
        super().__init__('depth_processing_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to depth and camera info
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )

        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)
        self.processed_depth_pub = self.create_publisher(Image, '/processed_depth', 10)
        self.obstacle_pub = self.create_publisher(PointStamped, '/obstacle_point', 10)

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info = None

        # Depth processing parameters
        self.depth_scale = 0.001  # Default: millimeters to meters
        self.obstacle_distance_threshold = 1.0  # meters

        self.get_logger().info('Depth Processing Node initialized')

    def info_callback(self, msg):
        """Process camera info to get intrinsic parameters"""
        try:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.width = msg.width
            self.height = msg.height
        except Exception as e:
            self.get_logger().error(f'Camera info callback error: {str(e)}')

    def depth_callback(self, msg):
        """Process depth image and convert to point cloud"""
        try:
            # Convert ROS Image to OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

            # Process depth image
            processed_depth = self.process_depth_image(depth_image)

            # Convert to point cloud if camera parameters are available
            if self.camera_matrix is not None:
                pointcloud_msg = self.depth_to_pointcloud(depth_image, msg.header)
                self.pointcloud_pub.publish(pointcloud_msg)

            # Publish processed depth image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_depth, "32FC1")
            processed_msg.header = msg.header
            self.processed_depth_pub.publish(processed_msg)

            # Detect obstacles
            self.detect_obstacles(depth_image, msg.header)

        except Exception as e:
            self.get_logger().error(f'Depth callback error: {str(e)}')

    def process_depth_image(self, depth_image):
        """Process depth image for noise reduction and filtering"""
        # Apply median filter to reduce noise
        filtered_depth = cv2.medianBlur(depth_image, 5)

        # Apply bilateral filter for edge-preserving smoothing
        filtered_depth = cv2.bilateralFilter(filtered_depth, 9, 75, 75)

        # Remove invalid depth values (NaN, infinity)
        filtered_depth = np.nan_to_num(filtered_depth, nan=0.0, posinf=0.0, neginf=0.0)

        # Apply depth range filtering
        filtered_depth = np.clip(filtered_depth, 0.1, 10.0)  # Clamp to 0.1m - 10m

        return filtered_depth

    def depth_to_pointcloud(self, depth_image, header):
        """Convert depth image to PointCloud2 message"""
        if self.camera_matrix is None:
            return PointCloud2()

        height, width = depth_image.shape

        # Generate coordinate grids
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))
        u_coords = u_coords.astype(np.float32)
        v_coords = v_coords.astype(np.float32)

        # Get camera intrinsic parameters
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Convert pixel coordinates to 3D coordinates
        x_coords = (u_coords - cx) * depth_image / fx
        y_coords = (v_coords - cy) * depth_image / fy
        z_coords = depth_image

        # Flatten arrays
        x_flat = x_coords.flatten()
        y_flat = y_coords.flatten()
        z_flat = z_coords.flatten()

        # Filter out invalid points
        valid_mask = (z_flat > 0) & (np.isfinite(z_flat)) & (np.isfinite(x_flat)) & (np.isfinite(y_flat))
        x_valid = x_flat[valid_mask]
        y_valid = y_flat[valid_mask]
        z_valid = z_flat[valid_mask]

        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create point data
        points = []
        for i in range(len(x_valid)):
            points.append([x_valid[i], y_valid[i], z_valid[i]])

        # Create PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.is_dense = True
        pointcloud_msg.point_step = 12  # 3 * 4 bytes (float32)
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width

        # Pack point data
        data = []
        for point in points:
            for value in point:
                data.append(struct.pack('f', value))

        pointcloud_msg.data = b''.join(data)

        return pointcloud_msg

    def detect_obstacles(self, depth_image, header):
        """Detect obstacles in the depth image"""
        # Define ROI for obstacle detection (front of robot)
        height, width = depth_image.shape
        roi_top = int(height * 0.3)
        roi_bottom = int(height * 0.7)
        roi_left = int(width * 0.2)
        roi_right = int(width * 0.8)

        roi_depth = depth_image[roi_top:roi_bottom, roi_left:roi_right]

        # Find points closer than threshold
        obstacle_mask = (roi_depth > 0) & (roi_depth < self.obstacle_distance_threshold)
        obstacle_points = np.where(obstacle_mask)

        if len(obstacle_points[0]) > 0:  # If obstacles detected
            # Calculate average obstacle position in ROI
            avg_y = np.mean(obstacle_points[0]) + roi_top
            avg_x = np.mean(obstacle_points[1]) + roi_left

            # Convert to 3D coordinates
            if self.camera_matrix is not None:
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]

                # Get average depth
                avg_depth = np.mean(roi_depth[obstacle_mask])

                # Convert to 3D coordinates
                x_3d = (avg_x - cx) * avg_depth / fx
                y_3d = (avg_y - cy) * avg_depth / fy
                z_3d = avg_depth

                # Publish obstacle point
                obstacle_point = PointStamped()
                obstacle_point.header = header
                obstacle_point.point.x = float(x_3d)
                obstacle_point.point.y = float(y_3d)
                obstacle_point.point.z = float(z_3d)

                self.obstacle_pub.publish(obstacle_point)

                self.get_logger().info(f'Obstacle detected at ({x_3d:.2f}, {y_3d:.2f}, {z_3d:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## SLAM for Environment Mapping

### Visual-Inertial SLAM Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import deque
import threading

class VISLAMNode(Node):
    def __init__(self):
        super().__init__('vislam_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to sensors
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/vislam_pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/vislam_map', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # SLAM state
        self.prev_image = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.imu_data = None
        self.trajectory = deque(maxlen=1000)

        # Feature detector
        self.feature_detector = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # SLAM lock
        self.slam_lock = threading.Lock()

        self.get_logger().info('Visual-Inertial SLAM Node initialized')

    def imu_callback(self, msg):
        """Process IMU data for sensor fusion"""
        with self.slam_lock:
            self.imu_data = {
                'angular_velocity': np.array([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ]),
                'linear_acceleration': np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ]),
                'orientation': np.array([
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ])
            }

    def image_callback(self, msg):
        """Process image for SLAM"""
        with self.slam_lock:
            try:
                # Convert to OpenCV
                current_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

                if self.prev_image is not None:
                    # Extract features from current image
                    current_keypoints, current_descriptors = self.feature_detector.detectAndCompute(
                        current_image, None
                    )

                    if (self.prev_keypoints is not None and
                        current_keypoints is not None and
                        current_descriptors is not None and
                        len(self.prev_keypoints) > 10 and
                        len(current_keypoints) > 10):

                        # Match features
                        matches = self.matcher.knnMatch(
                            self.prev_descriptors, current_descriptors, k=2
                        )

                        # Apply Lowe's ratio test
                        good_matches = []
                        for match_pair in matches:
                            if len(match_pair) == 2:
                                m, n = match_pair
                                if m.distance < 0.75 * n.distance:
                                    good_matches.append(m)

                        if len(good_matches) >= 10:
                            # Get matched points
                            prev_points = np.float32([
                                self.prev_keypoints[m.queryIdx].pt for m in good_matches
                            ]).reshape(-1, 1, 2)
                            curr_points = np.float32([
                                current_keypoints[m.trainIdx].pt for m in good_matches
                            ]).reshape(-1, 1, 2)

                            # Estimate essential matrix
                            E, mask = cv2.findEssentialMat(
                                curr_points, prev_points,
                                focal=500, pp=(320, 240), method=cv2.RANSAC, prob=0.999, threshold=1.0
                            )

                            if E is not None:
                                # Recover pose
                                _, R, t, _ = cv2.recoverPose(
                                    E, curr_points, prev_points
                                )

                                # Create transformation matrix
                                motion = np.eye(4)
                                motion[:3, :3] = R
                                motion[:3, 3] = t.flatten()

                                # Apply motion to current pose
                                self.current_pose = self.current_pose @ motion

                                # Store in trajectory
                                position = self.current_pose[:3, 3]
                                self.trajectory.append({
                                    'position': position,
                                    'timestamp': msg.header.stamp
                                })

                                # Publish pose and TF
                                self.publish_pose_and_tf(msg.header)

                # Store current frame data
                self.prev_image = current_image
                self.prev_keypoints = current_keypoints
                self.prev_descriptors = current_descriptors

            except Exception as e:
                self.get_logger().error(f'VISLAM error: {str(e)}')

    def publish_pose_and_tf(self, header):
        """Publish pose and transform"""
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = float(self.current_pose[0, 3])
        pose_msg.pose.position.y = float(self.current_pose[1, 3])
        pose_msg.pose.position.z = float(self.current_pose[2, 3])

        # Convert rotation matrix to quaternion
        R = self.current_pose[:3, :3]
        qw, qx, qy, qz = self.rotation_matrix_to_quaternion(R)
        pose_msg.pose.orientation.w = qw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz

        self.pose_pub.publish(pose_msg)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(self.current_pose[0, 3])
        t.transform.translation.y = float(self.current_pose[1, 3])
        t.transform.translation.z = float(self.current_pose[2, 3])

        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz

        self.tf_broadcaster.sendTransform(t)

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return qw, qx, qy, qz

def main(args=None):
    rclpy.init(args=args)
    node = VISLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Scene Understanding and Object Recognition

### Scene Understanding Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from collections import Counter

class SceneUnderstandingNode(Node):
    def __init__(self):
        super().__init__('scene_understanding_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to sensors
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.scene_desc_pub = self.create_publisher(String, '/scene_description', 10)
        self.object_map_pub = self.create_publisher(String, '/object_map', 10)

        # Scene understanding state
        self.scene_objects = []
        self.scene_layout = {}

        self.get_logger().info('Scene Understanding Node initialized')

    def image_callback(self, msg):
        """Process image for scene understanding"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Analyze scene content
            scene_analysis = self.analyze_scene(cv_image)

            # Publish scene description
            scene_desc_msg = String()
            scene_desc_msg.data = scene_analysis
            self.scene_desc_pub.publish(scene_desc_msg)

            self.get_logger().info(f'Scene analysis: {scene_analysis}')

        except Exception as e:
            self.get_logger().error(f'Scene understanding error: {str(e)}')

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D scene understanding"""
        try:
            # Convert PointCloud2 to numpy array (simplified)
            # In practice, use sensor_msgs_py.point_cloud2.read_points
            points_3d = self.pointcloud_to_array(msg)

            if points_3d is not None:
                # Analyze 3D scene structure
                scene_structure = self.analyze_3d_structure(points_3d)

                # Publish object map
                object_map_msg = String()
                object_map_msg.data = str(scene_structure)
                self.object_map_pub.publish(object_map_msg)

        except Exception as e:
            self.get_logger().error(f'Point cloud processing error: {str(e)}')

    def analyze_scene(self, image):
        """Analyze 2D scene content"""
        height, width = image.shape[:2]

        # Use simple color and texture analysis for scene understanding
        # In practice, use deep learning models for scene segmentation

        # Convert to different color spaces for analysis
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Analyze dominant colors
        dominant_colors = self.get_dominant_colors(image)

        # Analyze texture
        texture_analysis = self.analyze_texture(gray)

        # Analyze scene layout based on color distribution
        scene_layout = self.analyze_layout(hsv)

        # Combine analyses
        scene_description = {
            'dominant_colors': dominant_colors,
            'texture_type': texture_analysis,
            'layout': scene_layout,
            'estimated_objects': self.estimate_objects(image),
            'scene_type': self.classify_scene(dominant_colors, texture_analysis)
        }

        return str(scene_description)

    def get_dominant_colors(self, image, k=5):
        """Extract dominant colors from image"""
        # Reshape image to be a list of pixels
        pixels = image.reshape((-1, 3))

        # Convert to float
        pixels = np.float32(pixels)

        # Apply K-means clustering
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 0.1)
        _, labels, centers = cv2.kmeans(pixels, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        # Convert back to uint8
        centers = np.uint8(centers)

        # Count the frequency of each color
        unique, counts = np.unique(labels, return_counts=True)
        color_freq = dict(zip(centers, counts))

        # Sort by frequency
        sorted_colors = sorted(color_freq.items(), key=lambda x: x[1], reverse=True)

        return [color.tolist() for color, freq in sorted_colors[:3]]

    def analyze_texture(self, gray_image):
        """Analyze texture properties of image"""
        # Use Local Binary Pattern for texture analysis
        # Simplified version - in practice, use more sophisticated methods

        # Calculate gradients
        grad_x = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3)

        # Calculate gradient magnitude
        grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)

        # Analyze texture based on gradient statistics
        mean_grad = np.mean(grad_magnitude)
        std_grad = np.std(grad_magnitude)

        if mean_grad < 20:
            return "smooth"
        elif mean_grad < 50:
            return "moderate"
        else:
            return "rough"

    def analyze_layout(self, hsv_image):
        """Analyze scene layout based on color distribution"""
        height, width = hsv_image.shape[:2]

        # Divide image into regions
        regions = {
            'top': hsv_image[:height//3, :],
            'middle': hsv_image[height//3:2*height//3, :],
            'bottom': hsv_image[2*height//3:, :],
            'left': hsv_image[:, :width//3],
            'center': hsv_image[:, width//3:2*width//3],
            'right': hsv_image[:, 2*width//3:]
        }

        layout_analysis = {}
        for region_name, region in regions.items():
            if region.size > 0:
                # Analyze dominant hue in region
                dominant_hue = np.mean(region[:, :, 0])
                saturation_mean = np.mean(region[:, :, 1])
                value_mean = np.mean(region[:, :, 2])

                layout_analysis[region_name] = {
                    'dominant_hue': float(dominant_hue),
                    'saturation': float(saturation_mean),
                    'brightness': float(value_mean)
                }

        return layout_analysis

    def estimate_objects(self, image):
        """Estimate objects in scene using simple methods"""
        # In practice, use object detection models
        # For simulation, use simple shape detection

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        object_types = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter small contours
                # Approximate contour to get shape
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

                # Estimate shape based on number of vertices
                if len(approx) == 3:
                    object_types.append("triangle")
                elif len(approx) == 4:
                    # Check if it's square or rectangle
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.75 <= aspect_ratio <= 1.25:
                        object_types.append("square")
                    else:
                        object_types.append("rectangle")
                elif len(approx) > 4:
                    object_types.append("circle")  # Approximate as circle
                else:
                    object_types.append("other")

        # Count object types
        object_counts = Counter(object_types)
        return dict(object_counts)

    def classify_scene(self, dominant_colors, texture_type):
        """Classify scene type based on features"""
        # Simple classification based on dominant colors and texture
        # In practice, use trained models

        blue_count = sum(1 for color in dominant_colors if color[0] < 100 and color[1] < 150 and color[2] > 100)  # Blue-like
        green_count = sum(1 for color in dominant_colors if color[1] > color[0] and color[1] > color[2])  # Green-like
        brown_count = sum(1 for color in dominant_colors if all(50 < c < 150 for c in color))  # Brown-like

        if blue_count > 0:
            return "outdoor/nature"
        elif green_count > 0:
            return "indoor/garden"
        elif brown_count > 0 and texture_type == "rough":
            return "indoor/office"
        else:
            return "unknown"

    def pointcloud_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # This is a simplified version
        # In practice, use sensor_msgs_py.point_cloud2.read_points
        try:
            # For simulation purposes, return a dummy array
            # In real implementation, extract x, y, z coordinates
            return np.random.rand(100, 3) * 10  # Random points for simulation
        except:
            return None

    def analyze_3d_structure(self, points_3d):
        """Analyze 3D structure from point cloud"""
        if len(points_3d) == 0:
            return {}

        # Use DBSCAN for clustering to identify objects
        clustering = DBSCAN(eps=0.2, min_samples=10).fit(points_3d)
        labels = clustering.labels_

        # Count clusters (potential objects)
        unique_labels = set(labels)
        n_clusters = len(unique_labels) - (1 if -1 in labels else 0)  # Don't count noise

        # Analyze ground plane (lowest Z values)
        z_values = points_3d[:, 2]
        ground_z = np.percentile(z_values, 5)  # Approximate ground level

        # Analyze height distribution
        height_distribution = {
            'min_height': float(np.min(z_values)),
            'max_height': float(np.max(z_values)),
            'avg_height': float(np.mean(z_values)),
            'ground_level': float(ground_z)
        }

        return {
            'n_objects': n_clusters,
            'height_distribution': height_distribution,
            'estimated_room_size': self.estimate_room_size(points_3d)
        }

    def estimate_room_size(self, points_3d):
        """Estimate room dimensions from point cloud"""
        if len(points_3d) == 0:
            return {}

        x_range = [float(np.min(points_3d[:, 0])), float(np.max(points_3d[:, 0]))]
        y_range = [float(np.min(points_3d[:, 1])), float(np.max(points_3d[:, 1]))]
        z_range = [float(np.min(points_3d[:, 2])), float(np.max(points_3d[:, 2]))]

        return {
            'x_range': x_range,
            'y_range': y_range,
            'z_range': z_range,
            'width': float(x_range[1] - x_range[0]),
            'depth': float(y_range[1] - y_range[0]),
            'height': float(z_range[1] - z_range[0])
        }

def main(args=None):
    rclpy.init(args=args)
    node = SceneUnderstandingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Complete Perception System

In this lab, you'll integrate all perception components into a complete system.

### Step 1: Create the Perception System Launch File

Create `perception_system_launch.py`:

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

    # Perception system nodes
    multi_sensor_fusion = Node(
        package='ai_robo_learning',
        executable='multi_sensor_fusion_node',
        name='multi_sensor_fusion',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    object_detection = Node(
        package='ai_robo_learning',
        executable='object_detection_node',
        name='object_detection',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    depth_processing = Node(
        package='ai_robo_learning',
        executable='depth_processing_node',
        name='depth_processing',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    vislam = Node(
        package='ai_robo_learning',
        executable='vislam_node',
        name='vislam',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    scene_understanding = Node(
        package='ai_robo_learning',
        executable='scene_understanding_node',
        name='scene_understanding',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return launch description
    ld = LaunchDescription()

    # Add all nodes
    ld.add_action(multi_sensor_fusion)
    ld.add_action(object_detection)
    ld.add_action(depth_processing)
    ld.add_action(vislam)
    ld.add_action(scene_understanding)

    return ld
```

### Step 2: Create the Complete Perception Node

Create `complete_perception_system.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2, Imu, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import deque
import threading

class CompletePerceptionSystem(Node):
    def __init__(self):
        super().__init__('complete_perception_system')

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to all sensors
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.environment_map_pub = self.create_publisher(OccupancyGrid, '/environment_map', 10)
        self.obstacle_map_pub = self.create_publisher(OccupancyGrid, '/obstacle_map', 10)
        self.status_pub = self.create_publisher(Bool, '/perception_status', 10)

        # Perception system state
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None
        self.depth_data = None
        self.pointcloud_data = None
        self.camera_info = None

        # Perception buffers
        self.perception_buffer = deque(maxlen=100)
        self.fusion_lock = threading.Lock()

        # System status
        self.system_active = True
        self.perception_quality = 0.0

        self.get_logger().info('Complete Perception System initialized')

    def camera_callback(self, msg):
        """Process camera data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = {
                'image': cv_image,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            self.update_perception_quality()
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {str(e)}')

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        try:
            self.lidar_data = {
                'ranges': np.array(msg.ranges),
                'intensities': np.array(msg.intensities) if msg.intensities else None,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            }
            self.update_perception_quality()
        except Exception as e:
            self.get_logger().error(f'LiDAR callback error: {str(e)}')

    def imu_callback(self, msg):
        """Process IMU data"""
        try:
            self.imu_data = {
                'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                'timestamp': msg.header.stamp
            }
            self.update_perception_quality()
        except Exception as e:
            self.get_logger().error(f'IMU callback error: {str(e)}')

    def depth_callback(self, msg):
        """Process depth data"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.depth_data = {
                'image': depth_image,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            self.update_perception_quality()
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {str(e)}')

    def info_callback(self, msg):
        """Process camera info"""
        try:
            self.camera_info = msg
        except Exception as e:
            self.get_logger().error(f'Camera info callback error: {str(e)}')

    def pointcloud_callback(self, msg):
        """Process point cloud data"""
        try:
            self.pointcloud_data = msg
            self.update_perception_quality()
        except Exception as e:
            self.get_logger().error(f'Point cloud callback error: {str(e)}')

    def update_perception_quality(self):
        """Update overall perception system quality"""
        with self.fusion_lock:
            # Calculate perception quality based on available sensor data
            quality_score = 0.0

            if self.camera_data is not None:
                quality_score += 0.25
            if self.lidar_data is not None:
                quality_score += 0.25
            if self.imu_data is not None:
                quality_score += 0.15
            if self.depth_data is not None:
                quality_score += 0.25
            if self.pointcloud_data is not None:
                quality_score += 0.10

            self.perception_quality = quality_score

            # Publish system status
            status_msg = Bool()
            status_msg.data = (quality_score > 0.5)  # System is active if quality > 50%
            self.status_pub.publish(status_msg)

    def process_environment_mapping(self):
        """Process all sensor data to create environment map"""
        with self.fusion_lock:
            if not self.system_active:
                return

            # Create environment map by combining sensor data
            if self.lidar_data is not None:
                # Create occupancy grid from LiDAR data
                occupancy_grid = self.create_occupancy_grid_from_lidar()
                self.environment_map_pub.publish(occupancy_grid)

            if self.lidar_data is not None and self.camera_data is not None:
                # Create obstacle map combining LiDAR and vision data
                obstacle_map = self.create_obstacle_map()
                self.obstacle_map_pub.publish(obstacle_map)

    def create_occupancy_grid_from_lidar(self):
        """Create occupancy grid from LiDAR data"""
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import Point

        # Create a simple occupancy grid
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"

        # Define grid parameters
        resolution = 0.1  # 10cm resolution
        width = 200  # 20m x 20m area
        height = 200
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = -10.0
        grid.info.origin.position.y = -10.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Initialize grid with unknown (-1)
        grid.data = [-1] * (width * height)

        # Process LiDAR ranges
        ranges = self.lidar_data['ranges']
        angle_min = self.lidar_data['angle_min']
        angle_increment = self.lidar_data['angle_increment']

        for i, range_val in enumerate(ranges):
            if range_val > self.lidar_data['range_min'] and range_val < self.lidar_data['range_max']:
                angle = angle_min + i * angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)

                # Convert to grid coordinates
                grid_x = int((x - grid.info.origin.position.x) / resolution)
                grid_y = int((y - grid.info.origin.position.y) / resolution)

                # Check bounds
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    # Mark as occupied (100)
                    grid.data[grid_y * width + grid_x] = 100

        return grid

    def create_obstacle_map(self):
        """Create obstacle map combining multiple sensors"""
        # This would combine LiDAR, camera, and depth data
        # For simplicity, we'll use the same approach as occupancy grid
        return self.create_occupancy_grid_from_lidar()

def main(args=None):
    rclpy.init(args=args)
    node = CompletePerceptionSystem()

    try:
        # Create a timer to periodically process environment mapping
        timer = node.create_timer(0.1, node.process_environment_mapping)  # 10 Hz
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down perception system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test the Perception System

1. Run the complete perception system:
```bash
python3 complete_perception_system.py
```

2. Monitor the outputs in RViz:
```bash
# Visualize the environment map
ros2 run rviz2 rviz2
```

## Optimization Techniques

### Real-time Perception Optimization

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import threading

class PerceptionOptimizer(Node):
    def __init__(self):
        super().__init__('perception_optimizer')

        # Publishers for performance metrics
        self.fps_pub = self.create_publisher(Float32, '/perception_fps', 10)
        self.cpu_usage_pub = self.create_publisher(Float32, '/cpu_usage', 10)

        # Performance tracking
        self.frame_times = []
        self.max_frame_times = 100
        self.last_frame_time = time.time()

        # Optimization parameters
        self.target_fps = 10.0
        self.current_processing_rate = 1.0  # Process every Nth frame

        # Timer for performance monitoring
        self.timer = self.create_timer(1.0, self.update_performance_metrics)

        self.get_logger().info('Perception Optimizer initialized')

    def update_performance_metrics(self):
        """Update and publish performance metrics"""
        if len(self.frame_times) > 1:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

            fps_msg = Float32()
            fps_msg.data = fps
            self.fps_pub.publish(fps_msg)

            # Adjust processing rate based on performance
            if fps < self.target_fps * 0.8:  # Too slow, reduce processing
                self.current_processing_rate = min(5.0, self.current_processing_rate + 0.1)
            elif fps > self.target_fps * 1.2:  # Too fast, can process more
                self.current_processing_rate = max(1.0, self.current_processing_rate - 0.1)

            self.get_logger().info(f'Perception FPS: {fps:.2f}, Processing rate: {self.current_processing_rate:.1f}')

    def record_frame_time(self):
        """Record time taken for frame processing"""
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time

        if frame_time > 0:
            self.frame_times.append(frame_time)
            if len(self.frame_times) > self.max_frame_times:
                self.frame_times.pop(0)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionOptimizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

1. **Multi-Sensor Fusion**: Combine multiple sensor inputs for robust perception
2. **Real-time Processing**: Optimize algorithms for real-time performance
3. **Calibration**: Ensure proper calibration of all sensors
4. **Noise Filtering**: Apply appropriate filtering to reduce sensor noise
5. **Validation**: Continuously validate perception results against ground truth
6. **Fallback Systems**: Implement backup perception methods for critical functions

## Next Steps

After completing this chapter, you'll have a comprehensive understanding of perception systems for humanoid robots. In Module 4, you'll learn about Vision-Language-Action (VLA) pipelines that integrate these perception capabilities with AI decision-making.