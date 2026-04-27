#!/usr/bin/env bash
# =============================================================================
# run_orbslam3.sh — Run ORB-SLAM3 on EuRoC MAV dataset
#
# Usage:
#   ./scripts/run_orbslam3.sh <mode> <dataset_path_or_bagfile> [sequence_name]
#
# Arguments:
#   mode          : stereo_imu | mono_imu | stereo | mono
#   dataset_path  : Path to EuRoC dataset directory (ASL format) OR .bag file
#   sequence_name : Optional. Name for output files (default: "dataset")
#
# Examples:
#   ./scripts/run_orbslam3.sh stereo_imu ~/datasets/euroc/MH_01_easy
#   ./scripts/run_orbslam3.sh mono_imu   ~/datasets/euroc/MH_01_easy MH01_mono
#   ./scripts/run_orbslam3.sh stereo_imu ~/datasets/euroc/MH_01_easy.bag MH01
#
# Output:
#   CameraTrajectory.txt    — Full camera trajectory (TUM format)
#   KeyFrameTrajectory.txt  — Keyframe-only trajectory (TUM format)
#
# Requirements:
#   - ORB-SLAM3 built at $ORB_SLAM3_DIR (default: /opt/ORB_SLAM3)
#   - EuRoC dataset downloaded
#   - This repo cloned (for config files)
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
ORB_SLAM3_DIR="${ORB_SLAM3_DIR:-/opt/ORB_SLAM3}"
RESULTS_DIR="${REPO_DIR}/results"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

log_info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

usage() {
    cat <<EOF
Usage: $(basename "$0") <mode> <dataset_path> [sequence_name]

Modes:
  stereo_imu   Stereo + IMU (most accurate, recommended)
  mono_imu     Monocular + IMU
  stereo       Stereo only (no IMU)
  mono         Monocular only (scale ambiguous)

Examples:
  $(basename "$0") stereo_imu ~/datasets/euroc/MH_01_easy
  $(basename "$0") mono_imu   ~/datasets/euroc/MH_01_easy MH01_mono

Environment variables:
  ORB_SLAM3_DIR  Path to ORB-SLAM3 build directory (default: /opt/ORB_SLAM3)
EOF
    exit 1
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

if [ $# -lt 2 ]; then
    log_error "Too few arguments."
    usage
fi

MODE="$1"
DATASET_PATH="$2"
SEQ_NAME="${3:-dataset}"

# Validate mode
case "$MODE" in
    stereo_imu|mono_imu|stereo|mono) ;;
    *) log_error "Unknown mode: $MODE"; usage ;;
esac

# Validate paths
if [ ! -e "$DATASET_PATH" ]; then
    log_error "Dataset path not found: $DATASET_PATH"
    exit 1
fi

if [ ! -d "$ORB_SLAM3_DIR" ]; then
    log_error "ORB-SLAM3 directory not found: $ORB_SLAM3_DIR"
    log_error "Set ORB_SLAM3_DIR environment variable or build ORB-SLAM3 at /opt/ORB_SLAM3"
    exit 1
fi

VOC_FILE="${ORB_SLAM3_DIR}/Vocabulary/ORBvoc.txt"
if [ ! -f "$VOC_FILE" ]; then
    log_error "Vocabulary file not found: $VOC_FILE"
    exit 1
fi

# ---------------------------------------------------------------------------
# Determine timestamps file (for ASL format)
# ---------------------------------------------------------------------------

get_timestamps_file() {
    local dataset_name
    dataset_name="$(basename "$DATASET_PATH" | sed 's/_easy$//;s/_medium$//;s/_difficult$//')"
    local ts_dir="${ORB_SLAM3_DIR}/Examples"
    local ts_file=""

    # Map dataset name to timestamp file
    case "$MODE" in
        stereo_imu) ts_dir="${ts_dir}/Stereo-Inertial" ;;
        mono_imu)   ts_dir="${ts_dir}/Monocular-Inertial" ;;
        stereo)     ts_dir="${ts_dir}/Stereo" ;;
        mono)       ts_dir="${ts_dir}/Monocular" ;;
    esac

    # Try to find timestamp file
    ts_file="${ts_dir}/EuRoC_TimeStamps/${dataset_name}.txt"
    if [ ! -f "$ts_file" ]; then
        # Try uppercase
        ts_file="${ts_dir}/EuRoC_TimeStamps/$(echo "$dataset_name" | tr '[:lower:]' '[:upper:]').txt"
    fi

    echo "$ts_file"
}

# ---------------------------------------------------------------------------
# Config file selection
# ---------------------------------------------------------------------------

get_config_file() {
    case "$MODE" in
        stereo_imu) echo "${REPO_DIR}/configs/euroc_mav_stereo_imu.yaml" ;;
        mono_imu)   echo "${REPO_DIR}/configs/euroc_mav_mono_imu.yaml" ;;
        stereo)     echo "${REPO_DIR}/configs/euroc_mav_stereo_imu.yaml" ;;
        mono)       echo "${REPO_DIR}/configs/euroc_mav_mono_imu.yaml" ;;
    esac
}

# ---------------------------------------------------------------------------
# Executable selection
# ---------------------------------------------------------------------------

get_executable() {
    case "$MODE" in
        stereo_imu) echo "${ORB_SLAM3_DIR}/Examples/Stereo-Inertial/stereo_inertial_euroc" ;;
        mono_imu)   echo "${ORB_SLAM3_DIR}/Examples/Monocular-Inertial/mono_inertial_euroc" ;;
        stereo)     echo "${ORB_SLAM3_DIR}/Examples/Stereo/stereo_euroc" ;;
        mono)       echo "${ORB_SLAM3_DIR}/Examples/Monocular/mono_euroc" ;;
    esac
}

# ---------------------------------------------------------------------------
# Main execution
# ---------------------------------------------------------------------------

CONFIG_FILE="$(get_config_file)"
EXECUTABLE="$(get_executable)"
TIMESTAMPS="$(get_timestamps_file)"

log_info "=== ORB-SLAM3 Runner ==="
log_info "Mode:        $MODE"
log_info "Dataset:     $DATASET_PATH"
log_info "Config:      $CONFIG_FILE"
log_info "Executable:  $EXECUTABLE"
log_info "Timestamps:  $TIMESTAMPS"
log_info "Output name: $SEQ_NAME"
log_info "Results dir: $RESULTS_DIR"
echo ""

# Validate executable
if [ ! -f "$EXECUTABLE" ]; then
    log_error "ORB-SLAM3 executable not found: $EXECUTABLE"
    log_error "Please build ORB-SLAM3 first: cd $ORB_SLAM3_DIR && ./build.sh"
    exit 1
fi

# Validate config
if [ ! -f "$CONFIG_FILE" ]; then
    log_error "Config file not found: $CONFIG_FILE"
    exit 1
fi

# Create results directory
mkdir -p "$RESULTS_DIR"

# Change to results directory so output files land there
cd "$RESULTS_DIR"

# Detect if input is .bag file or ASL directory
if [[ "$DATASET_PATH" == *.bag ]]; then
    log_warn "ROS bag input detected. Ensure roscore is running."
    log_warn "This script does not automatically launch ROS nodes for bag playback."
    log_warn "Please use the ROS method described in the README instead."
    exit 0
fi

# Run ORB-SLAM3
log_info "Starting ORB-SLAM3..."
echo ""

if [ -f "$TIMESTAMPS" ]; then
    "$EXECUTABLE" \
        "$VOC_FILE" \
        "$CONFIG_FILE" \
        "$DATASET_PATH" \
        "$TIMESTAMPS" \
        "$SEQ_NAME"
else
    log_warn "Timestamps file not found: $TIMESTAMPS"
    log_warn "Running without explicit timestamp file..."
    "$EXECUTABLE" \
        "$VOC_FILE" \
        "$CONFIG_FILE" \
        "$DATASET_PATH" \
        "$SEQ_NAME"
fi

echo ""
log_ok "ORB-SLAM3 finished!"
log_info "Output files:"
ls -lh "${RESULTS_DIR}"/*.txt 2>/dev/null || log_warn "No .txt output found in $RESULTS_DIR"

echo ""
log_info "Next steps:"
log_info "  1. Evaluate: ./scripts/evaluate_trajectory.sh"
log_info "  2. Visualize: python3 scripts/visualize_trajectory.py --estimated results/CameraTrajectory.txt"
