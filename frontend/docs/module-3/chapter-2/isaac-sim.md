---
sidebar_position: 2
title: 'NVIDIA Isaac Sim'
---

# NVIDIA Isaac Sim

This chapter covers NVIDIA Isaac Sim, a comprehensive robotics simulation platform that provides photorealistic simulation, synthetic data generation, and AI training capabilities for humanoid robots and other robotic systems.

## What You'll Learn

In this chapter, you'll explore:
- NVIDIA Isaac Sim architecture and capabilities
- Setting up Isaac Sim for humanoid robot simulation
- Creating photorealistic environments
- Synthetic data generation for AI training
- Integration with ROS 2 and other frameworks
- Performance optimization techniques

## Prerequisites

- Completion of Module 1 and 2
- NVIDIA GPU with CUDA support
- Isaac Sim installed (Omniverse-based)
- Basic understanding of computer graphics and AI concepts
- ROS 2 Humble installed

## Understanding NVIDIA Isaac Sim

NVIDIA Isaac Sim is built on the Omniverse platform and provides:
- **Photorealistic rendering**: Using RTX technology for realistic lighting and materials
- **Physics simulation**: PhysX engine for accurate physics
- **Synthetic data generation**: Large-scale data generation for AI training
- **ROS 2 integration**: Native ROS 2 support for robotics workflows
- **AI training environment**: Reinforcement learning and imitation learning support

### Key Features

1. **High-Fidelity Graphics**: RTX-accelerated rendering for photorealistic environments
2. **Large-Scale Simulation**: Ability to run thousands of parallel simulations
3. **Synthetic Data Generation**: Generate labeled training data for computer vision
4. **Robot Simulation**: Support for various robot types including humanoid robots
5. **ROS 2 Bridge**: Seamless integration with ROS 2 ecosystem
6. **AI Training Framework**: Built-in support for reinforcement learning

## Setting Up Isaac Sim

### Installation Requirements

- **GPU**: NVIDIA RTX series GPU with CUDA support
- **OS**: Ubuntu 20.04 or 22.04 (recommended)
- **RAM**: 32GB or more recommended
- **Storage**: 100GB+ free space for assets
- **Omniverse**: Isaac Sim runs on NVIDIA Omniverse platform

### Basic Setup

Isaac Sim can be launched through Omniverse Launcher or directly:

```bash
# Launch Isaac Sim
isaac-sim

# Or launch with specific configuration
isaac-sim --config=robot_config.yaml
```

## Creating Robot Models in Isaac Sim

### USD Format for Robot Models

Isaac Sim uses Universal Scene Description (USD) format for 3D assets:

```usd
# Example USD file for a simple robot (robot.usd)
#usda 1.0

def Xform "Robot" (
    prepend references = @./base_link.usd@
)
{
    def Xform "LeftLeg" (
        prepend references = @./leg.usd@
    )
    {
        # Joint constraints and properties
    }

    def Xform "RightLeg" (
        prepend references = @./leg.usd@
    )
    {
        # Joint constraints and properties
    }
}
```

### Robot Configuration File

```yaml
# robot_config.yaml
robot:
  usd_path: "/path/to/robot.usd"
  position: [0, 0, 1.0]
  orientation: [0, 0, 0, 1]  # quaternion (x, y, z, w)

joints:
  left_hip:
    type: "revolute"
    limits: [-1.57, 1.57]
    drive: "position"
  left_knee:
    type: "revolute"
    limits: [0, 1.57]
    drive: "position"
  right_hip:
    type: "revolute"
    limits: [-1.57, 1.57]
    drive: "position"
  right_knee:
    type: "revolute"
    limits: [0, 1.57]
    drive: "position"

sensors:
  camera:
    type: "rgb"
    position: [0.1, 0, 0.8]
    orientation: [0, 0, 0, 1]
    resolution: [640, 480]
    fov: 60
  lidar:
    type: "lidar"
    position: [0.1, 0, 0.5]
    resolution: [720, 1]
    range: [0.1, 30.0]
```

## Programming Isaac Sim with Python

### Basic Robot Control Script

```python
#!/usr/bin/env python3

import omni
import carb
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import _sensor
import numpy as np
import asyncio

class IsaacSimRobotController:
    def __init__(self):
        self.world = None
        self.robot = None
        self.lidar_sensor = None
        self.camera_sensor = None

    async def setup_simulation(self):
        """Initialize Isaac Sim world and load robot"""
        # Create world instance
        self.world = World(stage_units_in_meters=1.0)

        # Load robot from USD file
        robot_path = "/path/to/your/robot.usd"
        add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")

        # Add robot to world
        self.robot = self.world.scene.add(
            Articulation(
                prim_path="/World/Robot",
                name="humanoid_robot",
                position=np.array([0, 0, 1.0]),
                orientation=np.array([0, 0, 0, 1])
            )
        )

        # Add sensors
        self.add_sensors()

        # Play the simulation
        self.world.play()

    def add_sensors(self):
        """Add sensors to the robot"""
        # Add camera sensor
        from omni.isaac.sensor import Camera
        self.camera_sensor = Camera(
            prim_path="/World/Robot/Camera",
            name="camera_sensor",
            position=np.array([0.1, 0, 0.8]),
            frequency=30
        )

        # Add LiDAR sensor
        from omni.isaac.sensor import RotatingLidarPhysX
        self.lidar_sensor = RotatingLidarPhysX(
            prim_path="/World/Robot/Lidar",
            name="lidar_sensor",
            translation=np.array([0.1, 0, 0.5]),
            orientation=np.array([0, 0, 0, 1])
        )

    async def run_robot_control(self):
        """Main robot control loop"""
        for i in range(1000):  # Run for 1000 simulation steps
            if i % 100 == 0:  # Print every 100 steps
                print(f"Simulation step: {i}")

            # Get sensor data
            if self.lidar_sensor:
                lidar_data = self.lidar_sensor.get_sensor_data()
                if lidar_data:
                    ranges = lidar_data.get('ranges', [])
                    if ranges is not None and len(ranges) > 0:
                        min_range = np.min(ranges)
                        print(f"Min LiDAR range: {min_range:.2f}")

            # Control robot based on sensor data
            await self.control_robot()

            # Step simulation
            self.world.step(render=True)

            # Small delay to prevent overwhelming the system
            await asyncio.sleep(0.01)

    async def control_robot(self):
        """Robot control logic"""
        # Simple walking pattern
        current_positions = self.robot.get_joints_state().position
        target_positions = current_positions.copy()

        # Apply walking gait (simplified)
        time_step = self.world.current_time_step_index
        phase = (time_step % 200) / 200.0  # 0 to 1 phase

        # Left leg
        target_positions[0] = 0.5 * np.sin(2 * np.pi * phase)  # hip
        target_positions[1] = 0.3 * np.sin(2 * np.pi * phase + np.pi)  # knee

        # Right leg
        target_positions[2] = 0.5 * np.sin(2 * np.pi * phase + np.pi)  # hip
        target_positions[3] = 0.3 * np.sin(2 * np.pi * phase)  # knee

        # Apply joint positions
        self.robot.set_joints_state(positions=target_positions)

    async def cleanup(self):
        """Clean up simulation"""
        if self.world:
            self.world.stop()
            await self.world.reset_async()

# Main execution
async def main():
    controller = IsaacSimRobotController()

    try:
        await controller.setup_simulation()
        await controller.run_robot_control()
    except Exception as e:
        print(f"Error during simulation: {e}")
    finally:
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
```

## Creating Photorealistic Environments

### Environment Configuration

```python
#!/usr/bin/env python3

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.utils.semantics import add_semantics
from pxr import UsdGeom, Gf
import numpy as np

class IsaacSimEnvironment:
    def __init__(self):
        self.world = None

    def create_office_environment(self):
        """Create a photorealistic office environment"""
        # Add ground plane
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Environments/Simple_Room/simple_room.usd",
            prim_path="/World/simple_room"
        )

        # Add furniture
        self.add_furniture()

        # Add lighting
        self.add_lighting()

        # Add textures and materials
        self.add_materials()

    def add_furniture(self):
        """Add office furniture"""
        # Add desk
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Props/Materials/metal_desk.usd",
            prim_path="/World/Desk"
        )

        # Add chair
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Props/Chair/chair.usd",
            prim_path="/World/Chair"
        )

        # Position furniture
        from omni.isaac.core.utils.transforms import set_local_pose
        set_local_pose("/World/Desk", position=np.array([2, 0, 0]))
        set_local_pose("/World/Chair", position=np.array([1.5, 0, 0]))

    def add_lighting(self):
        """Add realistic lighting"""
        # Add dome light for environment lighting
        from omni.isaac.core.utils.prims import create_prim
        create_prim(
            prim_path="/World/DomeLight",
            prim_type="DomeLight",
            position=np.array([0, 0, 0]),
            attributes={"color": (0.5, 0.5, 0.5), "intensity": 3000}
        )

        # Add key light
        create_prim(
            prim_path="/World/KeyLight",
            prim_type="DistantLight",
            position=np.array([5, 5, 10]),
            attributes={"color": (1, 1, 1), "intensity": 1000}
        )

    def add_materials(self):
        """Add realistic materials"""
        # Create and assign materials
        from omni.isaac.core.utils.materials import create_diffuse_material
        create_diffuse_material(
            prim_path="/World/Looks/wood_material",
            color=(0.6, 0.4, 0.2)
        )

def setup_environment():
    """Main environment setup function"""
    env = IsaacSimEnvironment()
    env.create_office_environment()
```

## Synthetic Data Generation

### Data Generation Script

```python
#!/usr/bin/env python3

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import cv2
import os
from PIL import Image
import json

class SyntheticDataGenerator:
    def __init__(self, output_dir="synthetic_data"):
        self.world = None
        self.camera = None
        self.output_dir = output_dir
        self.data_helper = SyntheticDataHelper()

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(f"{output_dir}/images", exist_ok=True)
        os.makedirs(f"{output_dir}/labels", exist_ok=True)

    async def setup_data_generation(self):
        """Setup Isaac Sim for data generation"""
        self.world = World(stage_units_in_meters=1.0)

        # Add robot
        add_reference_to_stage(
            usd_path="/path/to/robot.usd",
            prim_path="/World/Robot"
        )

        # Add camera for data capture
        self.camera = Camera(
            prim_path="/World/Robot/Camera",
            name="data_camera",
            position=np.array([0.1, 0, 0.8]),
            frequency=10  # 10 Hz for data collection
        )

        # Add various objects for training
        self.add_training_objects()

        self.world.play()

    def add_training_objects(self):
        """Add various objects for synthetic data generation"""
        object_types = [
            "omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Props/Blocks/block_20cm.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Props/YCB/Axis_Aligned/002_master_chef_can.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
            "omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Props/YCB/Axis_Aligned/004_sugar_box.usd"
        ]

        positions = [
            np.array([1, 1, 0.1]),
            np.array([-1, 1, 0.1]),
            np.array([1, -1, 0.1]),
            np.array([-1, -1, 0.1])
        ]

        for i, (obj_path, pos) in enumerate(zip(object_types, positions)):
            add_reference_to_stage(
                usd_path=obj_path,
                prim_path=f"/World/Object_{i}"
            )
            from omni.isaac.core.utils.transforms import set_local_pose
            set_local_pose(f"/World/Object_{i}", position=pos)

    async def generate_dataset(self, num_samples=1000):
        """Generate synthetic dataset"""
        data_samples = []

        for i in range(num_samples):
            # Move objects randomly for variation
            self.randomize_scene()

            # Step simulation to let objects settle
            for _ in range(10):
                self.world.step(render=True)

            # Capture data
            sample_data = await self.capture_sample(i)
            data_samples.append(sample_data)

            if i % 100 == 0:
                print(f"Generated {i}/{num_samples} samples")

        # Save dataset metadata
        self.save_metadata(data_samples)

    def randomize_scene(self):
        """Randomize object positions and lighting"""
        import random

        # Randomize object positions
        for i in range(4):  # 4 objects
            new_pos = np.array([
                random.uniform(-2, 2),
                random.uniform(-2, 2),
                random.uniform(0.1, 0.5)
            ])
            from omni.isaac.core.utils.transforms import set_local_pose
            set_local_pose(f"/World/Object_{i}", position=new_pos)

        # Randomize lighting
        # (Implementation would depend on specific lighting setup)

    async def capture_sample(self, sample_id):
        """Capture a single data sample"""
        # Get RGB image
        rgb_data = self.camera.get_rgb()
        if rgb_data is not None:
            rgb_image = Image.fromarray((rgb_data * 255).astype(np.uint8))
            rgb_path = f"{self.output_dir}/images/rgb_{sample_id:06d}.png"
            rgb_image.save(rgb_path)

        # Get depth image
        depth_data = self.camera.get_depth()
        if depth_data is not None:
            depth_image = Image.fromarray((depth_data * 1000).astype(np.uint16))  # Scale for 16-bit
            depth_path = f"{self.output_dir}/images/depth_{sample_id:06d}.png"
            depth_image.save(depth_path)

        # Get segmentation
        seg_data = self.camera.get_semantic_segmentation()
        if seg_data is not None:
            seg_image = Image.fromarray((seg_data).astype(np.uint16))
            seg_path = f"{self.output_dir}/images/seg_{sample_id:06d}.png"
            seg_image.save(seg_path)

        # Create sample metadata
        sample_metadata = {
            "id": sample_id,
            "rgb_path": f"images/rgb_{sample_id:06d}.png",
            "depth_path": f"images/depth_{sample_id:06d}.png",
            "seg_path": f"images/seg_{sample_id:06d}.png",
            "timestamp": self.world.current_time_step_index,
            "robot_pose": self.get_robot_pose(),
            "object_poses": self.get_object_poses()
        }

        return sample_metadata

    def get_robot_pose(self):
        """Get current robot pose"""
        # Implementation would depend on robot setup
        return {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}

    def get_object_poses(self):
        """Get poses of all objects in scene"""
        poses = {}
        for i in range(4):
            # Implementation would get actual object poses
            poses[f"object_{i}"] = {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
        return poses

    def save_metadata(self, data_samples):
        """Save dataset metadata"""
        metadata = {
            "dataset_name": "Humanoid Robot Perception Dataset",
            "num_samples": len(data_samples),
            "created": "2024-01-01",
            "description": "Synthetic dataset for humanoid robot perception",
            "samples": data_samples
        }

        with open(f"{self.output_dir}/metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)

# Main execution
async def generate_synthetic_data():
    generator = SyntheticDataGenerator()
    await generator.setup_data_generation()
    await generator.generate_dataset(num_samples=1000)
    print("Synthetic dataset generation completed!")
```

## ROS 2 Integration

### ROS 2 Bridge Setup

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, JointState, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import asyncio
from omni.isaac.core import World
from omni.isaac.sensor import Camera, RotatingLidarPhysX
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation

class IsaacSimROSBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_ros_bridge')

        # ROS publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # ROS subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self.joint_cmd_callback, 10)

        # CV bridge for image conversion
        self.cv_bridge = CvBridge()

        # Isaac Sim components
        self.world = None
        self.robot = None
        self.camera = None
        self.lidar = None

        # Robot command storage
        self.current_cmd_vel = Twist()
        self.current_joint_cmd = JointState()

        # Setup Isaac Sim
        self.setup_isaac_sim()

        # Timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def setup_isaac_sim(self):
        """Setup Isaac Sim environment"""
        self.world = World(stage_units_in_meters=1.0)

        # Load robot
        add_reference_to_stage(
            usd_path="/path/to/robot.usd",
            prim_path="/World/Robot"
        )

        self.robot = self.world.scene.add(
            Articulation(
                prim_path="/World/Robot",
                name="humanoid_robot",
                position=np.array([0, 0, 1.0])
            )
        )

        # Add sensors
        self.camera = self.world.scene.add(
            Camera(
                prim_path="/World/Robot/Camera",
                name="camera_sensor",
                position=np.array([0.1, 0, 0.8]),
                frequency=30
            )
        )

        self.lidar = self.world.scene.add(
            RotatingLidarPhysX(
                prim_path="/World/Robot/Lidar",
                name="lidar_sensor",
                translation=np.array([0.1, 0, 0.5])
            )
        )

        self.world.play()

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS"""
        self.current_cmd_vel = msg
        # Process velocity command for robot movement
        self.process_velocity_command(msg)

    def joint_cmd_callback(self, msg):
        """Handle joint commands from ROS"""
        self.current_joint_cmd = msg
        # Apply joint commands to robot
        self.apply_joint_commands(msg)

    def process_velocity_command(self, cmd_vel):
        """Process velocity command and convert to joint movements"""
        # Simple differential drive to joint conversion (example)
        linear_x = cmd_vel.linear.x
        angular_z = cmd_vel.angular.z

        # Convert to joint velocities for humanoid (simplified)
        # In reality, this would involve inverse kinematics
        pass

    def apply_joint_commands(self, joint_cmd):
        """Apply joint commands to the robot"""
        if self.robot and joint_cmd.position:
            try:
                self.robot.set_joints_state(positions=np.array(joint_cmd.position))
            except Exception as e:
                self.get_logger().error(f"Error setting joint states: {e}")

    def publish_sensor_data(self):
        """Publish sensor data to ROS topics"""
        if not self.world:
            return

        # Step simulation
        self.world.step(render=False)

        # Publish joint states
        self.publish_joint_states()

        # Publish camera data
        self.publish_camera_data()

        # Publish LiDAR data
        self.publish_lidar_data()

    def publish_joint_states(self):
        """Publish joint state information"""
        if not self.robot:
            return

        try:
            joint_positions = self.robot.get_joints_state().position
            joint_velocities = self.robot.get_joints_state().velocity
            joint_efforts = self.robot.get_joints_state().effort

            msg = JointState()
            msg.name = [f"joint_{i}" for i in range(len(joint_positions))]
            msg.position = list(joint_positions)
            msg.velocity = list(joint_velocities)
            msg.effort = list(joint_efforts)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing joint states: {e}")

    def publish_camera_data(self):
        """Publish camera image data"""
        if not self.camera:
            return

        try:
            rgb_data = self.camera.get_rgb()
            if rgb_data is not None:
                # Convert to ROS Image message
                ros_image = self.cv_bridge.cv2_to_imgmsg(rgb_data, "rgb8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_link"

                self.image_pub.publish(ros_image)

                # Publish camera info
                self.publish_camera_info()
        except Exception as e:
            self.get_logger().error(f"Error publishing camera data: {e}")

    def publish_camera_info(self):
        """Publish camera info"""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = "camera_link"
        info_msg.height = 480
        info_msg.width = 640
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients
        info_msg.k = [320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0]  # Camera matrix
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rectification matrix
        info_msg.p = [320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix

        self.camera_info_pub.publish(info_msg)

    def publish_lidar_data(self):
        """Publish LiDAR scan data"""
        if not self.lidar:
            return

        try:
            lidar_data = self.lidar.get_sensor_data()
            if lidar_data and 'ranges' in lidar_data:
                ranges = lidar_data['ranges']

                if ranges is not None and len(ranges) > 0:
                    scan_msg = LaserScan()
                    scan_msg.header.stamp = self.get_clock().now().to_msg()
                    scan_msg.header.frame_id = "lidar_link"
                    scan_msg.angle_min = -np.pi
                    scan_msg.angle_max = np.pi
                    scan_msg.angle_increment = (2 * np.pi) / len(ranges)
                    scan_msg.time_increment = 0.0
                    scan_msg.scan_time = 0.1
                    scan_msg.range_min = 0.1
                    scan_msg.range_max = 30.0
                    scan_msg.ranges = list(ranges)
                    scan_msg.intensities = []  # Empty for now

                    self.scan_pub.publish(scan_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing lidar data: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Initialize Isaac Sim in a separate thread
    import threading

    def run_isaac_sim():
        # Run Isaac Sim loop
        import asyncio
        async def sim_loop():
            bridge = IsaacSimROSBridge()

            try:
                # Keep running the bridge
                while rclpy.ok():
                    # Process ROS callbacks
                    rclpy.spin_once(bridge, timeout_sec=0.01)

                    # Update Isaac Sim
                    if bridge.world:
                        bridge.world.step(render=False)

                    # Small delay to prevent overwhelming
                    await asyncio.sleep(0.01)
            except Exception as e:
                bridge.get_logger().error(f"Isaac Sim loop error: {e}")
            finally:
                bridge.destroy_node()

        asyncio.run(sim_loop())

    # Start Isaac Sim in background thread
    sim_thread = threading.Thread(target=run_isaac_sim, daemon=True)
    sim_thread.start()

    # Keep main thread alive
    try:
        sim_thread.join()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Isaac Sim Humanoid Robot Simulation

In this lab, you'll create a complete Isaac Sim environment with a humanoid robot and ROS integration.

### Step 1: Create the Environment Setup Script

Create `isaac_sim_humanoid.py`:

```python
#!/usr/bin/env python3

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import Camera, RotatingLidarPhysX
from omni.isaac.core.utils.semantics import add_semantics
import numpy as np
import asyncio

class IsaacSimHumanoidLab:
    def __init__(self):
        self.world = None
        self.robot = None
        self.camera = None
        self.lidar = None

    async def setup_environment(self):
        """Setup complete Isaac Sim environment"""
        print("Setting up Isaac Sim environment...")

        # Initialize world
        self.world = World(stage_units_in_meters=1.0)

        # Add ground plane
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Environments/Simple_Room/simple_room.usd",
            prim_path="/World/defaultGroundPlane"
        )

        # Add robot
        # For this example, we'll use a simple robot USD
        # In practice, you would use your own humanoid robot model
        robot_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Robots/Franka/franka.usd"
        add_reference_to_stage(
            usd_path=robot_path,
            prim_path="/World/Robot"
        )

        # Add robot to scene
        self.robot = self.world.scene.add(
            Articulation(
                prim_path="/World/Robot",
                name="humanoid_robot",
                position=np.array([0, 0, 0.5]),
                orientation=np.array([0, 0, 0, 1])
            )
        )

        # Add sensors
        self.camera = self.world.scene.add(
            Camera(
                prim_path="/World/Robot/panda_hand/Camera",
                name="camera_sensor",
                position=np.array([0.1, 0, 0.1]),
                frequency=30
            )
        )

        self.lidar = self.world.scene.add(
            RotatingLidarPhysX(
                prim_path="/World/Robot/panda_hand/Lidar",
                name="lidar_sensor",
                translation=np.array([0.15, 0, 0.1])
            )
        )

        print("Environment setup complete!")

    async def run_simulation(self):
        """Run the simulation loop"""
        print("Starting simulation...")

        # Play the simulation
        self.world.play()

        # Run for a number of steps
        for step in range(10000):
            # Step the world
            self.world.step(render=True)

            # Every 100 steps, print status
            if step % 100 == 0:
                print(f"Simulation step: {step}")

                # Get and print sensor data
                if self.lidar:
                    lidar_data = self.lidar.get_sensor_data()
                    if lidar_data and 'ranges' in lidar_data:
                        ranges = lidar_data['ranges']
                        if ranges is not None and len(ranges) > 0:
                            min_range = np.min(ranges)
                            print(f"  Min LiDAR range: {min_range:.2f}m")

            # Small delay to prevent overwhelming the system
            await asyncio.sleep(0.01)

    async def cleanup(self):
        """Clean up the simulation"""
        if self.world:
            self.world.stop()
            await self.world.reset_async()
        print("Simulation cleaned up!")

async def main():
    lab = IsaacSimHumanoidLab()

    try:
        await lab.setup_environment()
        await lab.run_simulation()
    except Exception as e:
        print(f"Error during simulation: {e}")
    finally:
        await lab.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
```

### Step 2: Run the Simulation

1. Launch Isaac Sim:
```bash
isaac-sim
```

2. Run your Python script:
```bash
python3 isaac_sim_humanoid.py
```

## Performance Optimization

### Optimization Techniques

1. **Level of Detail (LOD)**: Use simpler models when the camera is far away
2. **Culling**: Don't render objects outside the camera's view
3. **Batch Processing**: Process multiple simulation instances in parallel
4. **Texture Streaming**: Load textures on demand rather than all at once
5. **Physics Simplification**: Use simpler collision meshes than visual meshes

### Configuration for Performance

```python
# Performance optimization settings
def configure_performance_settings():
    """Configure Isaac Sim for optimal performance"""

    # Set rendering quality
    carb.settings.get_settings().set("/app/window/spp", 1)  # Samples per pixel
    carb.settings.get_settings().set("/rtx/ambientOcclusion/enabled", False)
    carb.settings.get_settings().set("/rtx/dlss/enable", True)  # If DLSS is available

    # Physics settings
    carb.settings.get_settings().set("/physics_solver_frequency", 60)
    carb.settings.get_settings().set("/physics_max_substeps", 1)

    # Rendering settings
    carb.settings.get_settings().set("/app/renderer/enabled", True)
    carb.settings.get_settings().set("/app/renderer/resolution/width", 640)
    carb.settings.get_settings().set("/app/renderer/resolution/height", 480)
```

## Best Practices

1. **Asset Management**: Organize your 3D assets efficiently
2. **Scene Complexity**: Balance visual quality with performance
3. **Data Pipeline**: Create efficient workflows for synthetic data generation
4. **Validation**: Always validate simulation results against real-world data
5. **Documentation**: Document your simulation setups and configurations
6. **Version Control**: Use version control for your USD scenes and configurations

## Next Steps

After completing this chapter, you'll be ready to learn about Isaac ROS and hardware-accelerated perception in Chapter 3.