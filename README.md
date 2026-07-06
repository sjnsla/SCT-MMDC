# Single-Controller Teleoperation and Multimodal Dataset Generation for a Mobile Manipulator

Welcome to the official repository for the research project **"Single-Controller Teleoperation and Multimodal Dataset Generation for a Mobile Manipulator"** conducted at the Department of Cognitive Robotics, TU Delft.

This project introduces an integrated framework for efficient, teleoperated multimodal data collection using a single handheld controller (PlayStation DualShock 4). The system enables a single operator to execute tasks, record streams, filter out low-quality inputs, and automatically export structured data into machine-learning-ready formats.

---
## Framework Schematic

<img width="1997" height="949" alt="datapipeline_schematic drawio" src="https://github.com/user-attachments/assets/156829c6-6d1f-49ce-8b57-99ba0c81c7e2" />

---

## Hardware Setup & Specifications
The framework is fully integrated and validated on a specialized mobile manipulation platform consisting of the following hardware components:

* **Mobile Base:** **Clearpath Dingo-O** 
  * An indoor robotic base equipped with omnidirectional movement capabilities, ideal for navigating dynamic, restrictive, or unstructured environments.
  * link: https://clearpathrobotics.com/dingo-indoor-mobile-robot/
* **Robotic Manipulator:** **Kinova Gen3 Lite**
  * A lightweight robotic arm featuring 6 degrees of freedom (DoF), allowing highly accurate 3D object interaction. 
  * Explicitly configured to use a High-Level Controller (HLC) velocity mode via the Kinova API to ensure smooth, jitter-free movements during teleoperation.
  * link: https://www.kinovarobotics.com/product/gen3-lite-robots
* **Vision Sensor:** **Intel RealSense Depth Camera D455**
  * Mounted directly on top of the robot's end-effector via a custom, single-bolt quick-assembly mount.
  * Captures egocentric RGB-D (color and depth) data streams, providing a dynamic, detailed view of targeted objects from multiple angles.
  * link: https://www.realsenseai.com/products/real-sense-depth-camera-d455f/
* **Input Device:** **Sony PlayStation DualShock 4 Wireless Controller**
  * Connected via Bluetooth to orchestrate the base navigation, arm adjustments, and data collection routines from a single hardware interface.
  * link: https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/


<img width="2048" height="1536" alt="Robot task 4 2026-06-12 at 10 26 25" src="https://github.com/user-attachments/assets/5ed093ca-85c4-4ee2-b1d7-b66f1e7fd62f" />
---

## Demonstration Videos
The videos directory in this repository contains recorded demonstrations validating the framework:
* **visual synchronization assessment.mov** Video of the synchronisation between camerastreams and a visualisation of the robot arm based on the sensors.
* **controller mapping guide.mp4** A demonstration of the operator showcasing the mapping of the controller in both base controlling mode and arm controlling mode.
* **Experiment_recording_2026_06_04.MOV** Video capturing the robot being teleoperated to navigate, grasp a bottle, transport it between surfaces, and release it upright.
* **Mujoco simulation video 22-5-2026 (1).mp4** Video of a simulation of the robot arm, where inputs were tested before running them on the real arm.
* **pictures photoshoot-20260703T1214620Z-3-001.zip** Visuals of the robot, used in the project.

---

## System Architecture & Key Features

The framework addresses the challenges of fragmented teleoperation interfaces and heavy post-processing manual workflows through:
1. **Unified Teleoperation:** Full omnidirectional control of the Dingo-O base and joint/Cartesian movement of the Kinova arm using a single controller, eliminating the need to switch input devices.
2. **Automated ROS Pipeline:** A lightweight ROS Noetic stack running on Ubuntu 20.04 that records wheel encoders, joint states, end-effector positions, and raw RGB-D imagery from a realsense D455 camera into a unified, system-clock-timestamped rosbag via SSH.
3. **Visual Quality Filtering:** An automated script using OpenCV to apply a Laplacian filter for blur detection and a grayscale mean filter for brightness check, discarding corrupted frames.
4. **Temporal Synchronization:** Computationally efficient linear interpolation that aligns heterogeneous sensor frequencies using the camera feed as the primary temporal reference.
5. **Learning-Ready Storage:** Direct conversion of processed trajectories into chunked Zarr arrays, organizing images, joint states, and actions into structures optimized for Python-based deep learning pipelines.

---

#
