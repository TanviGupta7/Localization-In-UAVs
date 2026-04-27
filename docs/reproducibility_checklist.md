# Reproducibility Checklist: UAV Localization with ORB-SLAM3 + VIO

This checklist guides you through every step needed to reproduce the results in this repository from scratch.

---

## ✅ Environment Setup

### System Requirements
- [ ] **OS**: Ubuntu 20.04 LTS (recommended) or 18.04
- [ ] **RAM**: Minimum 8 GB, recommended 16 GB
- [ ] **Disk**: At least 20 GB free (ORB-SLAM3 + EuRoC datasets)
- [ ] **CPU**: Multi-core recommended (build uses all cores)
- [ ] **GPU**: Optional (not required for ORB-SLAM3)

### ROS Installation
- [ ] ROS Noetic installed (`sudo apt install ros-noetic-desktop-full`)
- [ ] ROS sourced in bashrc (`source /opt/ros/noetic/setup.bash`)
- [ ] `rosdep` initialized and updated
- [ ] `catkin_make` or `catkin build` available

### System Dependencies
```bash
sudo apt install -y \
    libopencv-dev \
    libeigen3-dev \
    libglew-dev \
    cmake \
    git \
    build-essential \
    python3-pip
```
- [ ] `libopencv-dev` installed
- [ ] `libeigen3-dev` installed
- [ ] `cmake` version ≥ 3.10
- [ ] `git` installed

### Python Dependencies
```bash
pip3 install numpy matplotlib scipy evo jupyter pandas
```
- [ ] `numpy` installed
- [ ] `matplotlib` installed
- [ ] `scipy` installed
- [ ] `evo` installed (`pip3 install evo`)
- [ ] `jupyter` installed (for notebooks)
- [ ] `pandas` installed (for ground truth CSV conversion)

---

## ✅ Building Pangolin

```bash
git clone https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin
cd /opt/Pangolin && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

- [ ] Pangolin cloned to `/opt/Pangolin`
- [ ] CMake configuration successful (no errors)
- [ ] Build completed without errors
- [ ] `sudo make install` ran successfully
- [ ] Verify: `ls /usr/local/lib/libpango*` shows library files

---

## ✅ Building ORB-SLAM3

```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3
cd /opt/ORB_SLAM3
chmod +x build.sh
./build.sh
```

- [ ] ORB-SLAM3 cloned to `/opt/ORB_SLAM3`
- [ ] `build.sh` is executable
- [ ] Build completed without errors (takes ~10 minutes)
- [ ] Verify executables exist:
  - [ ] `/opt/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc`
  - [ ] `/opt/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc`
- [ ] Vocabulary file exists: `/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt`

**Troubleshooting:**
- If OpenCV error: Ensure `libopencv-dev` is installed, or set `OpenCV_DIR`
- If Eigen error: Ensure `libeigen3-dev` is installed
- If Pangolin error: Rebuild Pangolin and re-run `./build.sh`

---

## ✅ Dataset Download

### EuRoC MAV (Primary Dataset)

```bash
mkdir -p ~/datasets/euroc
cd ~/datasets/euroc

# MH_01_easy (recommended first test)
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag

# Optional: additional sequences
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_02_easy/MH_02_easy.bag
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag
```

- [ ] `MH_01_easy.bag` downloaded (2.0 GB)
- [ ] MD5 checksum verified (optional but recommended)
- [ ] Dataset path noted: `~/datasets/euroc/`

### Alternative: ASL format (without ROS)

```bash
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
unzip MH_01_easy.zip -d ~/datasets/euroc/MH_01_easy
```

- [ ] Dataset extracted and structure verified:
  ```
  MH_01_easy/
  ├── mav0/
  │   ├── cam0/data/       ← Left camera images
  │   ├── cam1/data/       ← Right camera images
  │   ├── imu0/data.csv    ← IMU measurements
  │   └── state_groundtruth_estimate0/data.csv  ← Ground truth
  ```

---

## ✅ Repository Setup

```bash
git clone https://github.com/TanviGupta7/Localization-In-UAVs.git
cd Localization-In-UAVs
chmod +x scripts/run_orbslam3.sh
chmod +x scripts/evaluate_trajectory.sh
```

- [ ] Repository cloned
- [ ] Scripts made executable
- [ ] Config files present in `configs/`

---

## ✅ Running ORB-SLAM3

### Method A: Using provided script

```bash
./scripts/run_orbslam3.sh stereo_imu ~/datasets/euroc/MH_01_easy.bag
```

### Method B: Direct execution

```bash
cd /opt/ORB_SLAM3
./Examples/Stereo-Inertial/stereo_inertial_euroc \
    Vocabulary/ORBvoc.txt \
    /path/to/Localization-In-UAVs/configs/euroc_mav_stereo_imu.yaml \
    ~/datasets/euroc/MH_01_easy \
    Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt \
    dataset-MH01
```

**Expected behavior:**
- [ ] Viewer window opens (Pangolin) showing feature tracking
- [ ] Feature points visible as colored dots on frame
- [ ] 3D map builds progressively
- [ ] Terminal shows: `Tracking: OK` messages

**Expected output files (after completion):**
- [ ] `CameraTrajectory.txt` exists in working directory
- [ ] `KeyFrameTrajectory.txt` exists
- [ ] File format check (TUM format):
  ```
  # timestamp tx ty tz qx qy qz qw
  1403636579.763555527 0.0 0.0 0.0 0.0 0.0 0.0 1.0
  ...
  ```

---

## ✅ Trajectory Evaluation

### Install evo (if not already installed)
```bash
pip3 install evo --upgrade --no-binary evo
```
- [ ] `evo_ape --version` runs without error

### Convert ground truth to TUM format
```bash
# EuRoC ground truth is in CSV format, convert to TUM
python3 - <<'EOF'
import pandas as pd, numpy as np

df = pd.read_csv("~/datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv")
# Columns: #timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
#          q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []
tum = df[['#timestamp [ns]',
          ' p_RS_R_x [m]', ' p_RS_R_y [m]', ' p_RS_R_z [m]',
          ' q_RS_x []', ' q_RS_y []', ' q_RS_z []', ' q_RS_w []']].copy()
tum['#timestamp [ns]'] = tum['#timestamp [ns]'] / 1e9  # ns to s
tum.to_csv("groundtruth_MH01.txt", sep=' ', header=False, index=False)
print("Saved groundtruth_MH01.txt")
EOF
```
- [ ] `groundtruth_MH01.txt` created in TUM format

### Run ATE evaluation
```bash
evo_ape tum groundtruth_MH01.txt CameraTrajectory.txt \
    --align --correct_scale \
    --save_results results/ate_MH01.zip \
    --plot --plot_mode xy
```
- [ ] ATE computation completes without error
- [ ] RMSE value printed to terminal
- [ ] Plot displayed/saved
- [ ] Results archive saved

### Run RPE evaluation
```bash
evo_rpe tum groundtruth_MH01.txt CameraTrajectory.txt \
    --align --delta 1 --delta_unit m \
    --save_results results/rpe_MH01.zip \
    --plot --plot_mode xy
```
- [ ] RPE computation completes without error
- [ ] RMSE translation and rotation printed

### Expected results (MH_01_easy, Stereo-IMU)
- [ ] ATE RMSE < 0.10 m (good result is < 0.05 m)
- [ ] RPE translation RMSE < 0.03 m/m

---

## ✅ Visualization

### Run Python visualization script
```bash
python3 scripts/visualize_trajectory.py \
    --estimated CameraTrajectory.txt \
    --groundtruth groundtruth_MH01.txt \
    --output results/
```
- [ ] Script runs without errors
- [ ] `results/trajectory_2d.png` created
- [ ] `results/trajectory_3d.png` created
- [ ] `results/trajectory_comparison.png` created
- [ ] Plots show reasonable trajectory shapes (not degenerate/all-zeros)

### Open Jupyter notebook
```bash
jupyter notebook notebooks/uav_localization_visualization.ipynb
```
- [ ] Notebook opens in browser
- [ ] All cells run without error
- [ ] Plots display correctly

---

## ✅ Results Validation

### Check trajectory quality
- [ ] Trajectory is continuous (no large jumps > 1 m)
- [ ] Trajectory starts and ends near the same location (loop closure worked)
- [ ] 3D plot shows reasonable flight path shape

### Check metrics
- [ ] ATE RMSE is finite (not NaN or Inf)
- [ ] RPE values are finite and reasonable
- [ ] Results match published ORB-SLAM3 numbers within 20%

---

## 🔧 Troubleshooting

### ORB-SLAM3 crashes immediately
- Check vocabulary file path is correct
- Check config YAML file path is correct
- Ensure dataset path contains expected folder structure

### "Tracking: LOST" frequently
- Increase `nFeatures` in YAML config (try 2000)
- Decrease `iniThFAST` (try 10)
- Check timestamps in EuRoC timestamp file match bag

### evo gives "0 pairs found" error
- Check that timestamp formats match (seconds vs nanoseconds)
- Use `--t_offset` to account for small time offsets
- Verify both files are in TUM format

### Build errors in ORB-SLAM3
```bash
# Common fix: export library paths
export LD_LIBRARY_PATH=/opt/Pangolin/build/src:$LD_LIBRARY_PATH
cd /opt/ORB_SLAM3 && ./build.sh
```

---

## 📋 Summary

To reproduce results in under 30 minutes (assuming Ubuntu 20.04 + ROS already installed):

```bash
# 1. Install deps (5 min)
sudo apt install -y libopencv-dev libeigen3-dev libglew-dev
pip3 install evo matplotlib numpy jupyter

# 2. Build Pangolin + ORB-SLAM3 (15 min)
git clone https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin
cd /opt/Pangolin && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3
cd /opt/ORB_SLAM3 && ./build.sh

# 3. Download dataset (5 min)
wget -P ~/datasets/euroc \
  http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
unzip ~/datasets/euroc/MH_01_easy.zip -d ~/datasets/euroc/

# 4. Clone this repo + run
git clone https://github.com/TanviGupta7/Localization-In-UAVs.git
cd Localization-In-UAVs
chmod +x scripts/*.sh
./scripts/run_orbslam3.sh stereo_imu ~/datasets/euroc/MH_01_easy

# 5. Evaluate + visualize (5 min)
./scripts/evaluate_trajectory.sh CameraTrajectory.txt ~/datasets/euroc/MH_01_easy/...
python3 scripts/visualize_trajectory.py --estimated CameraTrajectory.txt --output results/
```

Total estimated time: **~30 minutes** (excluding download time)
