#!/usr/bin/env bash
# =============================================================================
# evaluate_trajectory.sh — Evaluate ORB-SLAM3 trajectory against ground truth
#
# Uses the 'evo' trajectory evaluation tool to compute:
#   - ATE (Absolute Trajectory Error)
#   - RPE (Relative Pose Error)
#
# Usage:
#   ./scripts/evaluate_trajectory.sh <estimated_traj> <groundtruth_traj> [output_dir]
#
# Arguments:
#   estimated_traj   : Path to ORB-SLAM3 output (CameraTrajectory.txt, TUM format)
#   groundtruth_traj : Path to ground truth trajectory (TUM format)
#                      For EuRoC: convert from data.csv using provided instructions
#   output_dir       : Directory for results (default: results/)
#
# Examples:
#   ./scripts/evaluate_trajectory.sh \
#       results/CameraTrajectory.txt \
#       ~/datasets/euroc/MH_01_easy/groundtruth_tum.txt
#
# TUM trajectory format:
#   # timestamp tx ty tz qx qy qz qw
#   1403636579.763555527 1.788 1.045 1.502 0.012 -0.004 -0.665 0.747
#   ...
#
# Dependencies:
#   pip3 install evo
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_section() { echo -e "\n${CYAN}=== $* ===${NC}"; }

usage() {
    cat <<EOF
Usage: $(basename "$0") <estimated_traj> <groundtruth_traj> [output_dir]

Arguments:
  estimated_traj   ORB-SLAM3 output trajectory in TUM format
  groundtruth_traj Ground truth trajectory in TUM format
  output_dir       Output directory for results and plots (default: results/)

TUM format:  timestamp tx ty tz qx qy qz qw

EuRoC ground truth conversion:
  python3 scripts/convert_euroc_gt.py \\
      ~/datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \\
      groundtruth_MH01.txt
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

EST_TRAJ="$1"
GT_TRAJ="$2"
OUTPUT_DIR="${3:-${REPO_DIR}/results}"

# Validate inputs
for f in "$EST_TRAJ" "$GT_TRAJ"; do
    if [ ! -f "$f" ]; then
        log_error "File not found: $f"
        exit 1
    fi
done

# Check evo is installed
if ! command -v evo_ape &>/dev/null; then
    log_error "'evo' not found. Install with: pip3 install evo"
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

log_info "=== Trajectory Evaluation ==="
log_info "Estimated:    $EST_TRAJ"
log_info "Ground truth: $GT_TRAJ"
log_info "Output dir:   $OUTPUT_DIR"
log_info "evo version:  $(evo_ape --version 2>/dev/null | head -1 || echo 'unknown')"

# ---------------------------------------------------------------------------
# Trajectory statistics
# ---------------------------------------------------------------------------

log_section "Trajectory Info"

log_info "Estimated trajectory:"
evo_traj tum "$EST_TRAJ" 2>/dev/null | grep -E "^(name|duration|path|num_poses|fps)" || true

log_info "Ground truth trajectory:"
evo_traj tum "$GT_TRAJ" 2>/dev/null | grep -E "^(name|duration|path|num_poses|fps)" || true

# ---------------------------------------------------------------------------
# ATE (Absolute Trajectory Error)
# ---------------------------------------------------------------------------

log_section "ATE — Absolute Trajectory Error"

log_info "Computing ATE with SE(3) alignment..."
evo_ape tum \
    "$GT_TRAJ" \
    "$EST_TRAJ" \
    --align \
    --correct_scale \
    --save_results "${OUTPUT_DIR}/ate_results.zip" \
    --plot_mode xy \
    --save_plot "${OUTPUT_DIR}/ate_plot.png" \
    2>&1 | tee "${OUTPUT_DIR}/ate_log.txt"

log_ok "ATE results saved to ${OUTPUT_DIR}/ate_results.zip"
log_ok "ATE plot saved to    ${OUTPUT_DIR}/ate_plot.png"

# ---------------------------------------------------------------------------
# RPE (Relative Pose Error) — distance-based (1 m segments)
# ---------------------------------------------------------------------------

log_section "RPE — Relative Pose Error (delta=1m)"

log_info "Computing RPE with 1-meter segments..."
evo_rpe tum \
    "$GT_TRAJ" \
    "$EST_TRAJ" \
    --align \
    --delta 1 \
    --delta_unit m \
    --save_results "${OUTPUT_DIR}/rpe_1m_results.zip" \
    --plot_mode xy \
    --save_plot "${OUTPUT_DIR}/rpe_1m_plot.png" \
    2>&1 | tee "${OUTPUT_DIR}/rpe_1m_log.txt"

log_ok "RPE (1m) results saved to ${OUTPUT_DIR}/rpe_1m_results.zip"

# ---------------------------------------------------------------------------
# RPE — time-based (1 second segments)
# ---------------------------------------------------------------------------

log_section "RPE — Relative Pose Error (delta=1s)"

log_info "Computing RPE with 1-second segments..."
evo_rpe tum \
    "$GT_TRAJ" \
    "$EST_TRAJ" \
    --align \
    --delta 1 \
    --delta_unit s \
    --save_results "${OUTPUT_DIR}/rpe_1s_results.zip" \
    --plot_mode xy \
    --save_plot "${OUTPUT_DIR}/rpe_1s_plot.png" \
    2>&1 | tee "${OUTPUT_DIR}/rpe_1s_log.txt"

log_ok "RPE (1s) results saved to ${OUTPUT_DIR}/rpe_1s_results.zip"

# ---------------------------------------------------------------------------
# Trajectory comparison plot
# ---------------------------------------------------------------------------

log_section "Trajectory Comparison Plot"

log_info "Generating trajectory comparison plot..."
evo_traj tum \
    "$EST_TRAJ" \
    --ref "$GT_TRAJ" \
    --align \
    --correct_scale \
    --plot_mode xy \
    --save_plot "${OUTPUT_DIR}/trajectory_comparison.png" \
    2>&1 || log_warn "Trajectory comparison plot failed (non-fatal)"

log_ok "Trajectory comparison saved to ${OUTPUT_DIR}/trajectory_comparison.png"

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

log_section "Summary"

echo ""
echo "Results saved in: $OUTPUT_DIR"
echo ""
echo "Files:"
ls -lh "${OUTPUT_DIR}"/*.zip "${OUTPUT_DIR}"/*.png "${OUTPUT_DIR}"/*.txt 2>/dev/null || true
echo ""

# Extract RMSE from ATE log for quick display
ATE_RMSE=$(grep -oP "(?<=rmse\s+)\S+" "${OUTPUT_DIR}/ate_log.txt" 2>/dev/null | head -1 || echo "N/A")
RPE_RMSE=$(grep -oP "(?<=rmse\s+)\S+" "${OUTPUT_DIR}/rpe_1m_log.txt" 2>/dev/null | head -1 || echo "N/A")

echo "╔══════════════════════════════════════╗"
echo "║          EVALUATION SUMMARY           ║"
echo "╠══════════════════════════════════════╣"
echo "║  ATE RMSE:          ${ATE_RMSE} m"
echo "║  RPE Trans RMSE:    ${RPE_RMSE} m/m"
echo "╚══════════════════════════════════════╝"
echo ""
log_info "For detailed analysis, unzip results:"
log_info "  python3 -c \"import zipfile; zipfile.ZipFile('${OUTPUT_DIR}/ate_results.zip').extractall('${OUTPUT_DIR}/ate')\""
