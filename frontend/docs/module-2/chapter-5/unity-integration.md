---
sidebar_position: 5
title: 'Unity Integration'
---

# Unity Integration

This chapter covers integrating Unity for high-fidelity rendering and human-robot interaction in simulation. Unity provides photorealistic environments and advanced graphics capabilities that complement Gazebo's physics simulation.

## What You'll Learn

In this chapter, you'll explore:
- Unity robotics simulation concepts
- Setting up Unity for robotics simulation
- ROS 2 integration with Unity
- Creating photorealistic environments
- Human-robot interaction in Unity
- Comparing Unity vs Gazebo for different use cases

## Prerequisites

- Completion of Module 1 and 2, Chapters 1-4
- Basic Unity knowledge (optional but helpful)
- ROS 2 Humble installed
- Understanding of 3D graphics concepts

## Understanding Unity for Robotics

Unity is a powerful game engine that can be adapted for robotics simulation. It excels at:
- Photorealistic rendering
- Complex visual environments
- Human-robot interaction scenarios
- VR/AR applications
- Advanced lighting and materials

### Unity vs Gazebo Comparison

| Feature | Unity | Gazebo |
|---------|-------|--------|
| Rendering Quality | Photorealistic | Good for visualization |
| Physics Simulation | Basic (Unity Physics) | Advanced (ODE, Bullet, DART) |
| ROS Integration | Unity Robotics Package | Native integration |
| Performance | Optimized for graphics | Optimized for physics |
| Learning Curve | Steeper | Moderate |

## Setting Up Unity for Robotics

### Installing Unity Robotics Package

The Unity Robotics Package provides:
- ROS communication via TCP/IP
- Sensor simulation (camera, LiDAR, IMU)
- Robot control interfaces
- Sample scenes and examples

1. Install Unity Hub and Unity 2021.3 LTS or later
2. Create a new 3D project
3. Install the Unity Robotics Package via Package Manager
4. Install the ROS TCP Connector

### ROS TCP Connector

The ROS TCP Connector enables communication between Unity and ROS 2:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

public class UnityRobotController : MonoBehaviour
{
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    private RosSocket rosSocket;

    void Start()
    {
        // Connect to ROS
        rosSocket = new RosSocket(new RosSharp.Communication.Protocol.WebSocket(rosIPAddress, rosPort));
    }

    void OnDestroy()
    {
        rosSocket.Close();
    }
}
```

## Creating a Unity Robot Model

### Importing Robot Models

Unity can import robot models in various formats:
- **FBX**: Most common format for 3D models
- **URDF**: Through Unity Robotics Package
- **STL**: Simple 3D models
- **OBJ**: Wavefront OBJ format

### Basic Robot Controller in Unity

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

public class UnityRobotController : MonoBehaviour
{
    [Header("Joint Control")]
    public Transform leftHip;
    public Transform leftKnee;
    public Transform rightHip;
    public Transform rightKnee;
    public Transform leftShoulder;
    public Transform leftElbow;
    public Transform rightShoulder;
    public Transform rightElbow;

    [Header("ROS Connection")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    private RosSocket rosSocket;
    private JointStatePublisher jointStatePublisher;
    private JointStateSubscriber jointStateSubscriber;

    void Start()
    {
        // Initialize ROS connection
        rosSocket = new RosSocket(new RosSharp.Communication.Protocol.WebSocket(rosIPAddress, rosPort));

        // Initialize publishers and subscribers
        jointStatePublisher = new JointStatePublisher(rosSocket, "/joint_states");
        jointStateSubscriber = new JointStateSubscriber(rosSocket, "/joint_commands", OnJointCommandsReceived);
    }

    void Update()
    {
        // Publish current joint states
        PublishJointStates();
    }

    void OnJointCommandsReceived(float[] positions)
    {
        // Apply joint commands to robot
        if (positions.Length >= 8)
        {
            leftHip.localRotation = Quaternion.Euler(0, 0, positions[0] * Mathf.Rad2Deg);
            leftKnee.localRotation = Quaternion.Euler(0, 0, positions[1] * Mathf.Rad2Deg);
            rightHip.localRotation = Quaternion.Euler(0, 0, positions[2] * Mathf.Rad2Deg);
            rightKnee.localRotation = Quaternion.Euler(0, 0, positions[3] * Mathf.Rad2Deg);
            leftShoulder.localRotation = Quaternion.Euler(positions[4] * Mathf.Rad2Deg, 0, 0);
            leftElbow.localRotation = Quaternion.Euler(positions[5] * Mathf.Rad2Deg, 0, 0);
            rightShoulder.localRotation = Quaternion.Euler(positions[6] * Mathf.Rad2Deg, 0, 0);
            rightElbow.localRotation = Quaternion.Euler(positions[7] * Mathf.Rad2Deg, 0, 0);
        }
    }

    void PublishJointStates()
    {
        // Get current joint positions and publish
        float[] currentPositions = {
            leftHip.localEulerAngles.z * Mathf.Deg2Rad,
            leftKnee.localEulerAngles.z * Mathf.Deg2Rad,
            rightHip.localEulerAngles.z * Mathf.Deg2Rad,
            rightKnee.localEulerAngles.z * Mathf.Deg2Rad,
            leftShoulder.localEulerAngles.x * Mathf.Deg2Rad,
            leftElbow.localEulerAngles.x * Mathf.Deg2Rad,
            rightShoulder.localEulerAngles.x * Mathf.Deg2Rad,
            rightElbow.localEulerAngles.x * Mathf.Deg2Rad
        };

        jointStatePublisher.Publish(currentPositions);
    }

    void OnDestroy()
    {
        rosSocket?.Close();
    }
}
```

## Sensor Simulation in Unity

### Camera Sensor

```csharp
using UnityEngine;
using RosSharp;

public class UnityCameraSensor : MonoBehaviour
{
    public Camera cameraComponent;
    public string topicName = "/unity_camera/image_raw";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float updateRate = 30.0f;

    private SensorPublisher sensorPublisher;
    private float updateInterval;
    private float lastUpdateTime;

    void Start()
    {
        updateInterval = 1.0f / updateRate;
        lastUpdateTime = 0;

        // Initialize sensor publisher
        sensorPublisher = new SensorPublisher(rosSocket, topicName);
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            CaptureAndPublishImage();
            lastUpdateTime = Time.time;
        }
    }

    void CaptureAndPublishImage()
    {
        // Capture image from camera
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cameraComponent.targetTexture;

        cameraComponent.Render();

        Texture2D imageTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        imageTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        imageTexture.Apply();

        RenderTexture.active = currentRT;

        // Convert to ROS image format and publish
        byte[] imageData = imageTexture.EncodeToJPG();
        sensorPublisher.PublishImage(imageData, imageWidth, imageHeight, "rgb8");

        Destroy(imageTexture);
    }
}
```

### IMU Sensor

```csharp
using UnityEngine;
using RosSharp;

public class UnityImuSensor : MonoBehaviour
{
    public string topicName = "/unity_imu/data";
    public float updateRate = 100.0f;

    private SensorPublisher sensorPublisher;
    private float updateInterval;
    private float lastUpdateTime;

    void Start()
    {
        updateInterval = 1.0f / updateRate;
        lastUpdateTime = 0;

        sensorPublisher = new SensorPublisher(rosSocket, topicName);
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            PublishImuData();
            lastUpdateTime = Time.time;
        }
    }

    void PublishImuData()
    {
        // Get linear acceleration (from physics or simulated)
        Vector3 linearAcc = GetComponent<Rigidbody>().velocity / Time.fixedDeltaTime;

        // Get angular velocity
        Vector3 angularVel = GetComponent<Rigidbody>().angularVelocity;

        // Create and publish IMU message
        sensorPublisher.PublishImu(
            transform.rotation,
            angularVel,
            linearAcc
        );
    }
}
```

## Creating Photorealistic Environments

### Environment Setup

Unity excels at creating photorealistic environments:

```csharp
using UnityEngine;

public class EnvironmentManager : MonoBehaviour
{
    [Header("Lighting")]
    public Light sunLight;
    public float dayNightCycleSpeed = 0.1f;
    private float timeOfDay = 0.5f; // 0 = midnight, 0.5 = noon, 1 = midnight

    [Header("Weather")]
    public GameObject rainSystem;
    public GameObject fogSystem;

    void Update()
    {
        UpdateDayNightCycle();
    }

    void UpdateDayNightCycle()
    {
        timeOfDay += dayNightCycleSpeed * Time.deltaTime;
        if (timeOfDay > 1) timeOfDay = 0;

        // Rotate sun based on time of day
        float sunAngle = timeOfDay * 360f - 90f;
        sunLight.transform.rotation = Quaternion.Euler(sunAngle, 0, 0);

        // Adjust lighting intensity based on sun angle
        float intensity = Mathf.Clamp01(Mathf.Cos(Mathf.Deg2Rad * sunAngle));
        sunLight.intensity = Mathf.Lerp(0.1f, 1f, intensity);
    }

    public void SetWeather(string weatherType)
    {
        switch (weatherType)
        {
            case "rain":
                rainSystem.SetActive(true);
                fogSystem.SetActive(false);
                break;
            case "fog":
                rainSystem.SetActive(false);
                fogSystem.SetActive(true);
                break;
            case "clear":
                rainSystem.SetActive(false);
                fogSystem.SetActive(false);
                break;
        }
    }
}
```

### Material and Lighting Setup

For photorealistic rendering, configure materials properly:

```csharp
using UnityEngine;

public class MaterialSetup : MonoBehaviour
{
    [Header("Robot Materials")]
    public Material robotBodyMaterial;
    public Material robotJointMaterial;
    public Material sensorMaterial;

    [Header("Environment Materials")]
    public Material floorMaterial;
    public Material wallMaterial;

    void Start()
    {
        SetupRobotMaterials();
        SetupEnvironmentMaterials();
    }

    void SetupRobotMaterials()
    {
        // Robot body material (metallic surface)
        robotBodyMaterial.SetFloat("_Metallic", 0.7f);
        robotBodyMaterial.SetFloat("_Smoothness", 0.8f);

        // Robot joint material (slightly different for visual distinction)
        robotJointMaterial.SetFloat("_Metallic", 0.5f);
        robotJointMaterial.SetFloat("_Smoothness", 0.6f);

        // Sensor material (transparent for visual identification)
        sensorMaterial.SetFloat("_Metallic", 0.2f);
        sensorMaterial.SetFloat("_Smoothness", 0.9f);
        sensorMaterial.SetFloat("_Surface", 1); // Transparent
        sensorMaterial.SetColor("_BaseColor", new Color(0, 0.8f, 1, 0.5f));
    }

    void SetupEnvironmentMaterials()
    {
        // Floor material (concrete or tile)
        floorMaterial.SetFloat("_Metallic", 0.1f);
        floorMaterial.SetFloat("_Smoothness", 0.3f);

        // Wall material (paint or plaster)
        wallMaterial.SetFloat("_Metallic", 0.0f);
        wallMaterial.SetFloat("_Smoothness", 0.2f);
    }
}
```

## Human-Robot Interaction in Unity

### VR/AR Interaction Setup

```csharp
using UnityEngine;
using UnityEngine.XR;

public class HumanRobotInteraction : MonoBehaviour
{
    [Header("Interaction Setup")]
    public Transform humanHead;
    public Transform[] interactionPoints;
    public LayerMask interactionLayer;

    [Header("Gestures")]
    public bool detectGestures = true;
    public float gestureDetectionRadius = 2.0f;

    void Update()
    {
        if (detectGestures)
        {
            DetectGestures();
        }

        HandleInteractions();
    }

    void DetectGestures()
    {
        // Detect hand gestures in the interaction radius
        Collider[] nearbyObjects = Physics.OverlapSphere(humanHead.position, gestureDetectionRadius, interactionLayer);

        foreach (Collider col in nearbyObjects)
        {
            if (IsGestureDetected(col))
            {
                TriggerInteraction(col);
            }
        }
    }

    bool IsGestureDetected(Collider target)
    {
        // Simple gesture detection logic
        // In practice, this would use more sophisticated hand tracking
        Vector3 directionToTarget = (target.transform.position - humanHead.position).normalized;
        float angleToTarget = Vector3.Angle(humanHead.forward, directionToTarget);

        // Detect if user is looking at and gesturing toward the target
        return angleToTarget < 30f; // Within 30-degree cone
    }

    void TriggerInteraction(Collider target)
    {
        // Send interaction command to ROS
        // This could be a custom message type
        string interactionCommand = $"INTERACT:{target.name}";

        // Publish to ROS topic
        // rosSocket.Publish("/interaction_commands", interactionCommand);
    }

    void HandleInteractions()
    {
        // Handle direct interactions (keyboard, mouse, VR controllers)
        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Send command to robot
            SendRobotCommand("STOP");
        }
        else if (Input.GetKeyDown(KeyCode.UpArrow))
        {
            SendRobotCommand("FORWARD");
        }
        else if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            SendRobotCommand("LEFT");
        }
        else if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            SendRobotCommand("RIGHT");
        }
    }

    void SendRobotCommand(string command)
    {
        // Send command to ROS system
        // rosSocket.Publish("/robot_commands", command);
    }
}
```

## Unity-Gazebo Hybrid Simulation

For the best of both worlds, you can combine Unity's rendering with Gazebo's physics:

### Architecture Overview

```
[Unity (Rendering)] <---> [ROS Bridge] <---> [Gazebo (Physics)]
      |                       |                     |
[High-quality visuals]  [Real-time data]  [Accurate physics]
```

### Synchronization Code

```csharp
using UnityEngine;
using RosSharp;

public class UnityGazeboSync : MonoBehaviour
{
    [Header("Synchronization")]
    public string gazeboJointStatesTopic = "/joint_states";
    public string unityCommandTopic = "/unity_robot_commands";

    private JointStateSubscriber gazeboJointStateSubscriber;
    private JointStatePublisher unityJointStatePublisher;

    private Dictionary<string, Transform> jointMap;

    void Start()
    {
        InitializeJointMap();
        InitializeRosConnections();
    }

    void InitializeJointMap()
    {
        // Map joint names to Unity transforms
        jointMap = new Dictionary<string, Transform>
        {
            {"left_hip", FindJoint("LeftHip")},
            {"left_knee", FindJoint("LeftKnee")},
            {"right_hip", FindJoint("RightHip")},
            {"right_knee", FindJoint("RightKnee")},
            {"left_shoulder", FindJoint("LeftShoulder")},
            {"left_elbow", FindJoint("LeftElbow")},
            {"right_shoulder", FindJoint("RightShoulder")},
            {"right_elbow", FindJoint("RightElbow")}
        };
    }

    Transform FindJoint(string jointName)
    {
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            if (child.name.Contains(jointName, System.StringComparison.OrdinalIgnoreCase))
                return child;
        }
        return null;
    }

    void InitializeRosConnections()
    {
        // Subscribe to Gazebo joint states
        gazeboJointStateSubscriber = new JointStateSubscriber(rosSocket, gazeboJointStatesTopic, OnGazeboJointStatesReceived);

        // Publish Unity joint states
        unityJointStatePublisher = new JointStatePublisher(rosSocket, unityCommandTopic);
    }

    void OnGazeboJointStatesReceived(string[] jointNames, float[] positions, float[] velocities, float[] efforts)
    {
        // Update Unity robot model based on Gazebo physics
        for (int i = 0; i < jointNames.Length && i < positions.Length; i++)
        {
            if (jointMap.ContainsKey(jointNames[i]))
            {
                Transform joint = jointMap[jointNames[i]];

                // Apply position based on joint type
                // For revolute joints, apply rotation
                joint.localRotation = Quaternion.Euler(0, 0, positions[i] * Mathf.Rad2Deg);
            }
        }
    }

    void Update()
    {
        // Publish current Unity state (for visualization)
        PublishUnityState();
    }

    void PublishUnityState()
    {
        // Get current joint states from Unity model
        List<string> jointNames = new List<string>();
        List<float> positions = new List<float>();
        List<float> velocities = new List<float>();
        List<float> efforts = new List<float>();

        foreach (var kvp in jointMap)
        {
            jointNames.Add(kvp.Key);

            // Convert Unity rotation back to joint position
            float position = kvp.Value.localEulerAngles.z * Mathf.Deg2Rad;
            positions.Add(position);

            // Simplified velocity and effort (in practice, get from physics)
            velocities.Add(0.0f);
            efforts.Add(0.0f);
        }

        unityJointStatePublisher.Publish(jointNames.ToArray(), positions.ToArray(), velocities.ToArray(), efforts.ToArray());
    }
}
```

## Hands-on Lab: Unity Humanoid Robot Simulation

In this lab, you'll set up a basic Unity scene with a humanoid robot and ROS integration.

### Step 1: Create Unity Project

1. Open Unity Hub and create a new 3D project
2. Install the Unity Robotics Package via Package Manager
3. Import the ROS TCP Connector

### Step 2: Create Robot Model

Create a simple humanoid robot in Unity:

1. Create an empty GameObject named "HumanoidRobot"
2. Add child objects for each body part:
   - Base (Cube)
   - Torso (Capsule)
   - Head (Sphere)
   - Arms and legs (Capsules)

### Step 3: Add Robot Controller Script

Attach the UnityRobotController script to your robot:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

public class UnityRobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot Joints")]
    public Transform leftHip;
    public Transform leftKnee;
    public Transform rightHip;
    public Transform rightKnee;
    public Transform leftShoulder;
    public Transform leftElbow;
    public Transform rightShoulder;
    public Transform rightElbow;

    private RosSocket rosSocket;
    private JointStatePublisher jointStatePublisher;
    private JointStateSubscriber jointStateSubscriber;

    void Start()
    {
        // Initialize ROS connection
        rosSocket = new RosSocket(new RosSharp.Communication.Protocol.WebSocket(rosIPAddress, rosPort));

        // Initialize publishers and subscribers
        jointStatePublisher = new JointStatePublisher(rosSocket, "/unity_joint_states");
        jointStateSubscriber = new JointStateSubscriber(rosSocket, "/unity_joint_commands", OnJointCommandsReceived);
    }

    void Update()
    {
        // Publish current joint states
        PublishJointStates();
    }

    void OnJointCommandsReceived(float[] positions)
    {
        // Apply joint commands to robot
        if (positions.Length >= 8)
        {
            if (leftHip != null) leftHip.localRotation = Quaternion.Euler(0, 0, positions[0] * Mathf.Rad2Deg);
            if (leftKnee != null) leftKnee.localRotation = Quaternion.Euler(0, 0, positions[1] * Mathf.Rad2Deg);
            if (rightHip != null) rightHip.localRotation = Quaternion.Euler(0, 0, positions[2] * Mathf.Rad2Deg);
            if (rightKnee != null) rightKnee.localRotation = Quaternion.Euler(0, 0, positions[3] * Mathf.Rad2Deg);
            if (leftShoulder != null) leftShoulder.localRotation = Quaternion.Euler(positions[4] * Mathf.Rad2Deg, 0, 0);
            if (leftElbow != null) leftElbow.localRotation = Quaternion.Euler(positions[5] * Mathf.Rad2Deg, 0, 0);
            if (rightShoulder != null) rightShoulder.localRotation = Quaternion.Euler(positions[6] * Mathf.Rad2Deg, 0, 0);
            if (rightElbow != null) rightElbow.localRotation = Quaternion.Euler(positions[7] * Mathf.Rad2Deg, 0, 0);
        }
    }

    void PublishJointStates()
    {
        if (rosSocket == null) return;

        // Get current joint positions
        float[] positions = new float[8];
        positions[0] = leftHip != null ? leftHip.localEulerAngles.z * Mathf.Deg2Rad : 0;
        positions[1] = leftKnee != null ? leftKnee.localEulerAngles.z * Mathf.Deg2Rad : 0;
        positions[2] = rightHip != null ? rightHip.localEulerAngles.z * Mathf.Deg2Rad : 0;
        positions[3] = rightKnee != null ? rightKnee.localEulerAngles.z * Mathf.Deg2Rad : 0;
        positions[4] = leftShoulder != null ? leftShoulder.localEulerAngles.x * Mathf.Deg2Rad : 0;
        positions[5] = leftElbow != null ? leftElbow.localEulerAngles.x * Mathf.Deg2Rad : 0;
        positions[6] = rightShoulder != null ? rightShoulder.localEulerAngles.x * Mathf.Deg2Rad : 0;
        positions[7] = rightElbow != null ? rightElbow.localEulerAngles.x * Mathf.Deg2Rad : 0;

        jointStatePublisher.Publish(positions);
    }

    void OnDestroy()
    {
        rosSocket?.Close();
    }
}
```

### Step 4: Create a ROS Bridge Node

Create `unity_bridge.py` to connect Unity with ROS 2:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import socket
import json
import threading
import time

class UnityBridge(Node):

    def __init__(self):
        super().__init__('unity_bridge')

        # ROS publishers and subscribers
        self.joint_state_publisher = self.create_publisher(JointState, '/unity_joint_states', 10)
        self.joint_command_subscriber = self.create_subscription(
            JointState, '/unity_joint_commands', self.joint_command_callback, 10)

        # Unity connection
        self.unity_socket = None
        self.unity_connected = False
        self.host = 'localhost'
        self.port = 5005

        # Start Unity connection thread
        self.unity_thread = threading.Thread(target=self.unity_connection_loop)
        self.unity_thread.daemon = True
        self.unity_thread.start()

        self.get_logger().info('Unity Bridge initialized')

    def unity_connection_loop(self):
        """Main loop for Unity communication"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        self.get_logger().info(f'Unity Bridge listening on {self.host}:{self.port}')

        while rclpy.ok():
            try:
                self.unity_socket, addr = server_socket.accept()
                self.get_logger().info(f'Unity connected from {addr}')
                self.unity_connected = True

                while self.unity_connected and rclpy.ok():
                    data = self.unity_socket.recv(1024).decode('utf-8')
                    if data:
                        self.process_unity_data(data)
                    else:
                        break

            except Exception as e:
                self.get_logger().error(f'Unity connection error: {str(e)}')
                self.unity_connected = False
                if self.unity_socket:
                    self.unity_socket.close()

            time.sleep(0.1)

    def process_unity_data(self, data):
        """Process data received from Unity"""
        try:
            unity_data = json.loads(data)

            if unity_data.get('type') == 'joint_states':
                self.publish_joint_states(unity_data['data'])
        except Exception as e:
            self.get_logger().error(f'Error processing Unity data: {str(e)}')

    def joint_command_callback(self, msg):
        """Send joint commands to Unity"""
        if self.unity_connected and self.unity_socket:
            try:
                unity_cmd = {
                    'type': 'joint_commands',
                    'data': {
                        'joint_names': list(msg.name),
                        'positions': list(msg.position),
                        'velocities': list(msg.velocity),
                        'efforts': list(msg.effort)
                    }
                }
                self.unity_socket.send(json.dumps(unity_cmd).encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Error sending to Unity: {str(e)}')

    def publish_joint_states(self, joint_data):
        """Publish joint states to ROS"""
        msg = JointState()
        msg.name = joint_data.get('joint_names', [])
        msg.position = joint_data.get('positions', [])
        msg.velocity = joint_data.get('velocities', [])
        msg.effort = joint_data.get('efforts', [])
        msg.header.stamp = self.get_clock().now().to_msg()

        self.joint_state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    unity_bridge = UnityBridge()

    try:
        rclpy.spin(unity_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        unity_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Test the Integration

1. Start the Unity Bridge:
```bash
python3 unity_bridge.py
```

2. Run your Unity scene
3. Verify that joint states are being published and commands are being received

## Best Practices

1. **Performance**: Balance visual quality with simulation performance
2. **Synchronization**: Ensure Unity and ROS states are properly synchronized
3. **Modularity**: Keep Unity and ROS components modular and independent
4. **Error Handling**: Implement robust error handling for network connections
5. **Validation**: Validate that Unity simulations match real-world expectations
6. **Documentation**: Document the Unity-ROS interface clearly

## Next Steps

After completing this chapter, you'll be ready to learn about environment building and scene creation in Chapter 6, where you'll explore creating complex simulation environments for humanoid robots.