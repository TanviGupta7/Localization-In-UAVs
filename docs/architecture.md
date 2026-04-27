# System Architecture: UAV Localization with ORB-SLAM3 + VIO

## Overview

This document describes the complete system architecture for GPS-denied UAV localization using ORB-SLAM3 with Visual-Inertial Odometry (VIO), integrated into an autonomous landing pipeline.

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              UAV Platform                                    │
│                                                                               │
│  ┌─────────────────────┐         ┌──────────────────────────────────────┐   │
│  │    SENSOR SUITE      │         │         ORB-SLAM3 VIO ENGINE          │   │
│  │                      │         │                                        │   │
│  │  ┌───────────────┐   │  Frames │  ┌────────────┐   ┌────────────────┐  │   │
│  │  │  Stereo Cam   │───┼────────►│  │  Tracking  │──►│  Local Mapping │  │   │
│  │  │  (20 Hz)      │   │         │  │  Thread    │   │  Thread        │  │   │
│  │  └───────────────┘   │         │  │            │   │  (Local BA)    │  │   │
│  │                      │         │  │ - ORB Ext  │   └───────┬────────┘  │   │
│  │  ┌───────────────┐   │  IMU    │  │ - PnP+RANSAC│          │           │   │
│  │  │  IMU          │───┼────────►│  │ - KF dec.  │   ┌───────▼────────┐  │   │
│  │  │  (200 Hz)     │   │         │  └─────┬──────┘   │  Loop Closing  │  │   │
│  │  └───────────────┘   │         │        │Pose       │  Thread        │  │   │
│  └─────────────────────┘         │        │           │  (Full BA)     │  │   │
│                                   │  ┌─────▼──────────►┌──────────────┐ │  │   │
│                                   │  │  IMU Pre-Integr  │  Atlas /     │ │  │   │
│                                   │  │  + MAP Init      │  Multi-Map   │ │  │   │
│                                   │  └──────────────────└──────────────┘ │  │   │
│                                   │             │ 6-DoF Pose @ 200Hz      │   │
│                                   └─────────────┼──────────────────────────┘   │
│                                                 │                               │
│  ┌──────────────────────────────────────────────▼─────────────────────────┐   │
│  │                     AUTONOMY STACK                                       │   │
│  │                                                                           │   │
│  │  ┌─────────────────┐   ┌─────────────────┐   ┌───────────────────────┐  │   │
│  │  │  Path Planner   │   │ Platform Detect. │   │  Landing Controller   │  │   │
│  │  │  (A* / MPC)     │   │ (ArUco/DNN)     │   │  (PX4 + MAVROS)      │  │   │
│  │  └────────┬────────┘   └────────┬────────┘   └──────────┬────────────┘  │   │
│  │           │                     │                         │               │   │
│  │  ┌────────▼─────────────────────▼─────────────────────────▼────────────┐ │   │
│  │  │                     Flight Controller (PX4)                          │ │   │
│  │  │              Attitude / Velocity / Position Control                  │ │   │
│  │  └─────────────────────────────────────────────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## ORB-SLAM3 Internal Architecture

```
Input Sensors
     │
     ▼
┌──────────────────────────────────────────────────────────────────┐
│                    ORB-SLAM3 System                               │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │                   TRACKING THREAD                         │    │
│  │                                                            │    │
│  │  Camera Frame ──► ORB Extraction ──► Feature Matching     │    │
│  │                            │                              │    │
│  │  IMU Data ──► Pre-Integration ──► Initial Pose Estimate   │    │
│  │                            │                              │    │
│  │                     PnP + RANSAC                          │    │
│  │                            │                              │    │
│  │                    Pose Refinement ──► Current Pose        │    │
│  │                            │                              │    │
│  │                   Keyframe Decision                        │    │
│  │                    YES │    NO │                           │    │
│  └────────────────────────┼───────┼───────────────────────────┘   │
│                           │       └──► Continue tracking           │
│                           ▼                                        │
│  ┌────────────────────────────────────────────────────────┐      │
│  │                LOCAL MAPPING THREAD                     │      │
│  │                                                          │      │
│  │  New Keyframe ──► Map Point Creation ──► Local BA        │      │
│  │                         │                               │      │
│  │                  Keyframe Culling                        │      │
│  │                  Map Point Culling                       │      │
│  └───────────────────────────────────────────────────────┘       │
│                           │                                        │
│                    New Keyframe?                                   │
│                           ▼                                        │
│  ┌────────────────────────────────────────────────────────┐      │
│  │              LOOP CLOSING THREAD                        │      │
│  │                                                          │      │
│  │  New KF ──► DBoW2 Detection ──► Loop Candidate         │      │
│  │                      │                                   │      │
│  │              Geometric Verification                      │      │
│  │                      │                                   │      │
│  │              Loop Correction + Pose Graph Opt.           │      │
│  │                      │                                   │      │
│  │              Full Bundle Adjustment (async)              │      │
│  └────────────────────────────────────────────────────────┘      │
│                                                                    │
│  ┌────────────────────────────────────────────────────────┐      │
│  │              ATLAS (Multi-Map Manager)                  │      │
│  │  Active Map │ Stored Maps │ Place Recognition DB        │      │
│  └────────────────────────────────────────────────────────┘      │
└──────────────────────────────────────────────────────────────────┘
```

---

## Data Flow Diagram

```
┌──────────────┐     ┌──────────────┐     ┌──────────────────────────┐
│  Camera L    │────►│              │────►│                          │
└──────────────┘     │  Sensor      │     │    ORB-SLAM3 System       │
                     │  Sync &      │     │                          │
┌──────────────┐     │  Timestamping│     │  Input: Stereo frames    │
│  Camera R    │────►│              │     │         IMU stream        │
└──────────────┘     │              │     │                          │
                     │              │     │  Output:                 │
┌──────────────┐     │              │────►│    T_WC (4x4 matrix)     │
│  IMU         │────►│              │     │    Map (optional)        │
└──────────────┘     └──────────────┘     │    Status flags          │
                                          └──────────┬───────────────┘
                                                     │
                                    ┌────────────────▼──────────────────┐
                                    │           Pose Publisher           │
                                    │   ROS: geometry_msgs/PoseStamped   │
                                    │   File: CameraTrajectory.txt       │
                                    └────────────────┬──────────────────┘
                                                     │
                          ┌──────────────────────────┼──────────────────────────┐
                          │                          │                          │
                ┌─────────▼──────┐       ┌──────────▼──────┐       ┌──────────▼──────┐
                │ Path Planner   │       │  Visualization  │       │ Landing Control │
                │ (Navigation)   │       │  (Pangolin/RViz)│       │ (Precision Land)│
                └────────────────┘       └─────────────────┘       └─────────────────┘
```

---

## Coordinate Frame Definitions

```
World Frame (W):          Body Frame (B):           Camera Frame (C):
     Z (up)                    Z (forward)                Z (into scene)
     │                         │                          │
     └── Y                     └── X (right)              └── X (right)
    /                         /                          /
   X (East)                  Y (down)                  Y (down)

Camera-to-World transform:
    T_WC = T_WB * T_BC   (extrinsic calibration from IMU-Camera)

IMU-to-Camera transform T_BC is fixed and provided in YAML config.
```

---

## Software Stack

```
┌─────────────────────────────────────────────────────┐
│                  Application Layer                   │
│  - visualize_trajectory.py   (Python / matplotlib)  │
│  - evaluate_trajectory.sh    (evo tool)              │
│  - run_orbslam3.sh            (wrapper script)       │
└────────────────────────┬────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────┐
│                 SLAM Framework Layer                  │
│  - ORB-SLAM3         (C++ / CMake)                   │
│  - DBoW2             (Bag of Words)                  │
│  - g2o               (Graph Optimization)            │
│  - Sophus            (Lie Group Math)                │
└────────────────────────┬────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────┐
│                Middleware / OS Layer                  │
│  - ROS Noetic        (message passing)               │
│  - Ubuntu 20.04 LTS                                  │
│  - POSIX threads (pthreads)                          │
└────────────────────────┬────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────┐
│                   Hardware Layer                      │
│  - Stereo Camera         (USB3 / CSI)                │
│  - IMU                   (USB / SPI)                 │
│  - CPU: x86_64 or ARM    (Jetson Xavier / NX)        │
└─────────────────────────────────────────────────────┘
```

---

## Key Parameters and Tuning

| Parameter | Default | Effect |
|-----------|---------|--------|
| nFeatures | 1000 | More features = better tracking, slower |
| scaleFactor | 1.2 | Image pyramid scale |
| nLevels | 8 | Pyramid levels for scale invariance |
| iniThFAST | 20 | FAST threshold (initial) |
| minThFAST | 7 | FAST threshold (minimum, used if too few features) |
| IMU.NoiseGyro | 1.7e-4 | Gyroscope noise density |
| IMU.NoiseAcc | 2.0e-3 | Accelerometer noise density |
| IMU.GyroWalk | 1.9e-5 | Gyroscope bias random walk |
| IMU.AccWalk | 3.0e-3 | Accelerometer bias random walk |

---

## Landing Pipeline Architecture

```
                    ┌─────────────────────────────────┐
                    │         Mission Manager          │
                    │   State: TAKEOFF → EXPLORE →    │
                    │          APPROACH → LAND         │
                    └────────────────┬────────────────┘
                                     │
          ┌──────────────────────────┼──────────────────────────┐
          │                          │                          │
┌─────────▼──────────┐   ┌──────────▼──────────┐   ┌──────────▼──────────┐
│  ORB-SLAM3 VIO     │   │  Platform Detector  │   │  State Estimator    │
│                    │   │                     │   │                     │
│  T_WB: UAV pose    │   │  ArUco detector     │   │  Kalman Filter      │
│  in world frame    │   │  T_CP: pad pose     │   │  Platform pose      │
│                    │   │  in camera frame    │   │  prediction         │
└────────┬───────────┘   └──────────┬──────────┘   └──────────┬──────────┘
         │                          │                          │
         └──────────────────────────┼──────────────────────────┘
                                    │
                          ┌─────────▼─────────┐
                          │  Landing Controller│
                          │                   │
                          │  PID / MPC        │
                          │  Velocity setpoints│
                          └─────────┬─────────┘
                                    │
                          ┌─────────▼─────────┐
                          │   PX4 FCU         │
                          │   (MAVROS)        │
                          │                   │
                          │  Motor commands   │
                          └───────────────────┘
```
