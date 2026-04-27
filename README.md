# 🚁 UAV Localization using ORB-SLAM3 + Visual-Inertial Odometry (VIO)

> **GPS-Denied Autonomous Drone Navigation & Landing on a Moving Platform**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://python.org)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](http://wiki.ros.org/noetic)
[![Dataset: EuRoC](https://img.shields.io/badge/Dataset-EuRoC%20MAV-orange.svg)](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

---

## 📋 Table of Contents

1. [Problem Explanation](#-problem-explanation)
2. [Why Localization Matters](#-why-localization-matters)
3. [ORB-SLAM3 + VIO Overview](#-orb-slam3--vio-overview)
4. [Architecture](#-architecture)
5. [Datasets](#-datasets)
6. [Repository Structure](#-repository-structure)
7. [Setup Instructions](#-setup-instructions)
8. [Running the System](#-running-the-system)
9. [Evaluating Results](#-evaluating-results)
10. [Sample Outputs](#-sample-outputs)
11. [Simulation Integration](#-simulation-integration)
12. [Reproducibility Checklist](#-reproducibility-checklist)

---

## 🎯 Problem Explanation

### Simple Explanation

Imagine a drone flying indoors or in a tunnel — there is no GPS signal. How does it know *where it is*? Without position feedback, the drone cannot navigate safely, stabilize itself, or land on a target.

**This project answers that question** by replacing GPS with a combination of:
- A **camera** (sees visual features in the environment)
- An **IMU** (Inertial Measurement Unit — measures acceleration and rotation)

Together, these sensors allow the drone to build a map of its surroundings and track its own position within that map in real time. This is called **Simultaneous Localization and Mapping (SLAM)**.

### Technical Explanation

This project implements **Visual-Inertial Odometry (VIO)** using **ORB-SLAM3** to estimate the 6-DoF (Degrees of Freedom) pose of a UAV (Unmanned Aerial Vehicle) in GPS-denied environments. The system:

1. Extracts **ORB (Oriented FAST and Rotated BRIEF)** keypoints from camera frames
2. Tracks features across consecutive frames to estimate camera motion
3. Fuses IMU pre-integration data with visual measurements using a **tightly-coupled** optimization backend
4. Maintains a sparse 3D map of the environment (keyframes + map points)
5. Performs **loop closure** to correct accumulated drift
6. Outputs a **6-DoF trajectory** (position + orientation) that can be used for navigation and landing

The final goal is **autonomous landing on a moving platform** where GPS is unavailable, relying entirely on onboard sensors.

---

## 🌍 Why Localization Matters

| Scenario | GPS Available? | Challenge |
|----------|---------------|-----------|
| Indoor inspection | ❌ No | Walls block satellite signals |
| Urban canyon flight | ⚠️ Degraded | Buildings cause multipath errors |
| Underground / tunnel | ❌ No | Complete signal loss |
| Military operations | ❌ Jammed | Adversarial GPS jamming |
| Autonomous landing on ship | ⚠️ Unreliable | Moving platform + vibration noise |

Accurate localization is the **foundation of every autonomous task**: navigation, obstacle avoidance, path planning, and precision landing. Without it, the drone is essentially blind.

---

## 🤖 ORB-SLAM3 + VIO

### What is ORB-SLAM3?

[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) is the state-of-the-art open-source SLAM library that supports:
- **Monocular**, **Stereo**, and **RGB-D** cameras
- **IMU integration** (Visual-Inertial SLAM)
- **Multi-map** management and atlas
- **Loop closing** with DBoW2
- **Map reuse** across sessions

### What is Visual-Inertial Odometry (VIO)?

VIO combines two complementary sensors:

| Sensor | Strength | Weakness |
|--------|----------|----------|
| Camera | Rich texture, no drift | Fails in dark/featureless scenes, scale ambiguous |
| IMU | Fast, works in dark | High drift over time |

**Fusing them** (tightly-coupled) gives:
- Metric scale estimation (fixes monocular ambiguity)
- Robustness to fast motion and texture-poor environments
- High-rate pose estimates at IMU frequency (~200 Hz)

### System Pipeline

```
Camera Frames ──────► Feature Extraction (ORB) ──────►
                                                        Tightly-Coupled
IMU Measurements ──► Pre-integration ───────────────►  Graph Optimization ──► Pose Estimate
                                                        (g2o / Lie Algebra)
```

### Key Algorithms

- **ORB feature detection + matching** — rotation/scale invariant keypoints
- **IMU pre-integration** on SO(3) manifold (Forster et al., 2015)
- **Bundle Adjustment** — joint optimization of poses + map points
- **DBoW2 bag-of-words** — place recognition for loop closure
- **IMU initialization** — gravity alignment + bias estimation

---

## 🏗️ Architecture

See [`docs/architecture.md`](docs/architecture.md) for the full system architecture diagram.

```
┌──────────────────────────────────────────────────────────────────┐
│                        UAV Onboard System                         │
│                                                                    │
│  ┌─────────┐    ┌──────────────────────────────────────────────┐  │
│  │ Camera  │───►│              ORB-SLAM3 VIO                   │  │
│  └─────────┘    │  ┌──────────┐  ┌──────────┐  ┌──────────┐   │  │
│  ┌─────────┐    │  │ Tracking │─►│Local Map │─►│   Loop   │   │  │
│  │   IMU   │───►│  │ Thread   │  │ Thread   │  │  Closer  │   │  │
│  └─────────┘    │  └──────────┘  └──────────┘  └──────────┘   │  │
│                 └──────────────────────┬─────────────────────┘  │
│                                        │ 6-DoF Pose              │
│                 ┌──────────────────────▼─────────────────────┐  │
│                 │           Flight Controller (PX4)           │  │
│                 │    Path Planning │ Stabilization │ Landing  │  │
│                 └──────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 📊 Datasets

### EuRoC MAV Dataset (Recommended)
- **Source**: [ETH Zurich ASL](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- **Sensors**: Stereo camera (Aptina MT9V034) + IMU (ADIS16448)
- **Sequences**: MH_01 through MH_05, V1_01 through V2_03
- **Ground Truth**: Vicon motion capture system

```bash
# Download MH_01_easy sequence
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
```

### TUM VI Dataset
- **Source**: [TUM](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
- **Sensors**: Fisheye stereo camera + IMU
- **Environments**: Corridors, rooms, outdoors

```bash
# Download corridor1_512_16 sequence
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-corridor1_512_16.tar.gz
```

### KITTI Odometry Dataset
- **Source**: [KITTI Vision Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- **Sensors**: Stereo camera + Velodyne LiDAR + GPS/IMU
- **Note**: Primarily for autonomous driving, but useful for stereo baseline experiments

### Dataset Comparison

| Dataset | Camera | IMU | GT Quality | Best For |
|---------|--------|-----|-----------|----------|
| EuRoC MAV | Stereo | ✅ | Vicon (mm accuracy) | UAV/Drone testing |
| TUM VI | Fisheye Stereo | ✅ | Motion Capture | Indoor navigation |
| KITTI | Stereo | ✅ | GPS+IMU | Outdoor/vehicle |

---

## 📁 Repository Structure

```
Localization-In-UAVs/
│
├── README.md                          ← This file
│
├── docs/
│   ├── report.md                      ← Detailed research report
│   ├── architecture.md                ← System architecture diagram
│   └── reproducibility_checklist.md  ← Step-by-step checklist
│
├── configs/
│   ├── euroc_mav_mono_imu.yaml        ← EuRoC Mono+IMU camera config
│   └── euroc_mav_stereo_imu.yaml      ← EuRoC Stereo+IMU camera config
│
├── scripts/
│   ├── run_orbslam3.sh                ← Run ORB-SLAM3 on EuRoC dataset
│   ├── evaluate_trajectory.sh         ← Evaluate with evo (ATE + RPE)
│   └── visualize_trajectory.py        ← Python 2D/3D trajectory visualizer
│
├── notebooks/
│   └── uav_localization_visualization.ipynb  ← Colab/Jupyter notebook
│
└── results/
    ├── CameraTrajectory.txt           ← Example trajectory output (TUM format)
    └── sample_metrics.txt             ← Example ATE/RPE metrics
```

---

## ⚙️ Setup Instructions

### Prerequisites

| Component | Version | Notes |
|-----------|---------|-------|
| Ubuntu | 20.04 LTS | Recommended |
| ROS | Noetic | For bag file playback |
| Python | 3.8+ | For visualization scripts |
| OpenCV | 4.x | Usually installed with ROS |
| Eigen3 | 3.3+ | Linear algebra |
| Pangolin | Latest | Visualization (optional) |

### Step 1: Install ROS Noetic

```bash
# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
sudo apt install -y ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Dependencies

```bash
sudo apt install -y \
    libopencv-dev \
    libeigen3-dev \
    libglew-dev \
    cmake \
    git \
    python3-pip

pip3 install numpy matplotlib scipy evo
```

### Step 3: Build Pangolin (ORB-SLAM3 viewer)

```bash
git clone https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin
cd /opt/Pangolin
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### Step 4: Build ORB-SLAM3

```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3
cd /opt/ORB_SLAM3
chmod +x build.sh
./build.sh
```

### Step 5: Clone This Repository

```bash
git clone https://github.com/TanviGupta7/Localization-In-UAVs.git
cd Localization-In-UAVs
pip3 install -r requirements.txt   # if present
```

### Optional: Google Colab Setup

Open the notebook directly in Colab — no local setup required:

[![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/TanviGupta7/Localization-In-UAVs/blob/main/notebooks/uav_localization_visualization.ipynb)

---

## 🚀 Running the System

### Method 1: Using the Run Script (Recommended)

```bash
# Make executable
chmod +x scripts/run_orbslam3.sh

# Run on EuRoC MH_01 sequence (Stereo-IMU mode)
./scripts/run_orbslam3.sh stereo_imu /path/to/MH_01_easy.bag

# Run on EuRoC MH_01 sequence (Mono-IMU mode)
./scripts/run_orbslam3.sh mono_imu /path/to/MH_01_easy.bag
```

### Method 2: Direct Commands

#### Stereo-IMU Mode (Best accuracy on EuRoC)

```bash
cd /opt/ORB_SLAM3

# Run ORB-SLAM3 Stereo-IMU on EuRoC
./Examples/Stereo-Inertial/stereo_inertial_euroc \
    Vocabulary/ORBvoc.txt \
    /path/to/Localization-In-UAVs/configs/euroc_mav_stereo_imu.yaml \
    /path/to/MH_01_easy \
    /path/to/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt \
    dataset-MH01_stereo_imu
```

#### Monocular-IMU Mode

```bash
./Examples/Monocular-Inertial/mono_inertial_euroc \
    Vocabulary/ORBvoc.txt \
    /path/to/Localization-In-UAVs/configs/euroc_mav_mono_imu.yaml \
    /path/to/MH_01_easy \
    /path/to/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt \
    dataset-MH01_mono_imu
```

#### Using ROS Bag

```bash
# Terminal 1: Start ROS core
roscore

# Terminal 2: Run ORB-SLAM3 ROS node
rosrun ORB_SLAM3 Stereo_Inertial \
    /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /path/to/Localization-In-UAVs/configs/euroc_mav_stereo_imu.yaml \
    true

# Terminal 3: Play the bag file
rosbag play /path/to/MH_01_easy.bag \
    /cam0/image_raw:=/camera/left/image_raw \
    /cam1/image_raw:=/camera/right/image_raw \
    /imu0:=/imu
```

### Expected Output Files

After running, ORB-SLAM3 will save:
- `CameraTrajectory.txt` — estimated trajectory in TUM format
- `KeyFrameTrajectory.txt` — keyframe poses only

---

## 📈 Evaluating Results

### Using the Evaluation Script

```bash
chmod +x scripts/evaluate_trajectory.sh

# Full evaluation (ATE + RPE)
./scripts/evaluate_trajectory.sh \
    results/CameraTrajectory.txt \
    /path/to/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv
```

### Manual evo Commands

#### ATE (Absolute Trajectory Error)

```bash
# Compute ATE RMSE
evo_ape tum \
    /path/to/groundtruth.txt \
    results/CameraTrajectory.txt \
    --align \
    --correct_scale \
    --plot \
    --save_results results/ate_results.zip

# Plot trajectory comparison
evo_traj tum \
    results/CameraTrajectory.txt \
    --ref /path/to/groundtruth.txt \
    --align \
    --plot
```

#### RPE (Relative Pose Error)

```bash
# Compute RPE over 1-meter segments
evo_rpe tum \
    /path/to/groundtruth.txt \
    results/CameraTrajectory.txt \
    --align \
    --delta 1 \
    --delta_unit m \
    --plot \
    --save_results results/rpe_results.zip
```

#### Batch Evaluation (Multiple Sequences)

```bash
# Evaluate multiple sequences and compare
evo_ape tum \
    groundtruth_MH01.txt estimated_MH01.txt \
    groundtruth_MH02.txt estimated_MH02.txt \
    --align --plot
```

### Understanding Metrics

| Metric | What It Measures | Good Value (EuRoC) |
|--------|-----------------|-------------------|
| ATE RMSE | Overall global accuracy | < 0.05 m |
| ATE Max | Worst-case error | < 0.15 m |
| RPE Trans | Local translation consistency | < 0.02 m/m |
| RPE Rot | Local rotation consistency | < 1.0 °/m |

---

## 📊 Sample Outputs

### Example Metrics (MH_01_easy, Stereo-IMU)

```
Absolute Trajectory Error (ATE):
  RMSE:   0.0312 m
  Mean:   0.0278 m
  Median: 0.0251 m
  Std:    0.0143 m
  Min:    0.0021 m
  Max:    0.0891 m

Relative Pose Error (RPE, delta=1m):
  RMSE trans:   0.0148 m/m
  RMSE rot:     0.4231 deg/m
```

### Trajectory Visualization

To generate trajectory plots:

```bash
# 2D + 3D trajectory visualization
python3 scripts/visualize_trajectory.py \
    --estimated results/CameraTrajectory.txt \
    --groundtruth /path/to/groundtruth.txt \
    --output results/
```

This produces:
- `results/trajectory_2d.png` — top-down 2D trajectory
- `results/trajectory_3d.png` — full 3D trajectory
- `results/trajectory_comparison.png` — estimated vs ground truth

---

## 🎮 Simulation Integration

### Gazebo (ROS-based)

```bash
# Install Gazebo + UAV simulation packages
sudo apt install ros-noetic-gazebo-ros-pkgs
sudo apt install ros-noetic-rotors-simulator  # Firefly UAV model

# Launch simulation with camera + IMU sensors
roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly

# Bridge sensor topics to ORB-SLAM3
rosrun ORB_SLAM3 Mono_Inertial \
    /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    configs/euroc_mav_mono_imu.yaml
```

### AirSim (Microsoft)

```python
# AirSim Python API for UAV + camera data
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Get camera image
responses = client.simGetImages([
    airsim.ImageRequest("front_center", airsim.ImageType.Scene)
])

# Get IMU data
imu_data = client.getImuData()
```

See [`docs/report.md`](docs/report.md) for detailed simulation setup instructions.

### Landing Module Placeholder

The landing module would use the pose estimate from ORB-SLAM3 to:
1. Detect the landing platform (ArUco marker or visual pattern)
2. Compute relative pose between UAV and platform
3. Generate descent trajectory
4. Execute precision landing via PX4 MAVROS

```python
# Placeholder: Landing controller interface
class LandingController:
    def __init__(self, slam_pose_topic, platform_detector):
        self.pose = None
        self.platform_pose = None

    def update_pose(self, slam_pose):
        self.pose = slam_pose

    def compute_landing_trajectory(self):
        # TODO: Implement landing trajectory generation
        raise NotImplementedError("Landing module — coming soon")
```

---

## ✅ Reproducibility Checklist

See [`docs/reproducibility_checklist.md`](docs/reproducibility_checklist.md) for the complete checklist.

**Quick checklist:**
- [ ] Ubuntu 20.04 installed
- [ ] ROS Noetic installed and sourced
- [ ] ORB-SLAM3 built successfully
- [ ] EuRoC dataset downloaded
- [ ] Python dependencies installed (`pip3 install evo matplotlib numpy`)
- [ ] Run script executed without errors
- [ ] CameraTrajectory.txt generated
- [ ] ATE RMSE computed with `evo`
- [ ] Trajectory plots generated

---

## 📚 References

1. Campos, C., et al. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multimap SLAM**. *IEEE Transactions on Robotics*.
2. Burri, M., et al. (2016). **The EuRoC Micro Aerial Vehicle Datasets**. *The International Journal of Robotics Research*.
3. Forster, C., et al. (2015). **IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation**. *RSS*.
4. Qin, T., et al. (2018). **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**. *IEEE Transactions on Robotics*.
5. Mur-Artal, R., & Tardós, J. D. (2017). **ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras**. *IEEE Transactions on Robotics*.

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Please read the contributing guidelines and feel free to open a pull request.

---

*Built for academic research in autonomous UAV systems. Suitable for course projects, thesis demonstrations, and research prototyping.*