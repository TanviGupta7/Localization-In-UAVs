#!/usr/bin/env python3
"""
visualize_trajectory.py — 2D and 3D trajectory visualization for ORB-SLAM3 output.

Plots estimated trajectory (from ORB-SLAM3 CameraTrajectory.txt) alongside
optional ground truth trajectory, generating publication-quality figures.

Usage:
    # Estimated trajectory only
    python3 scripts/visualize_trajectory.py \\
        --estimated results/CameraTrajectory.txt \\
        --output results/

    # With ground truth comparison
    python3 scripts/visualize_trajectory.py \\
        --estimated results/CameraTrajectory.txt \\
        --groundtruth groundtruth_MH01.txt \\
        --output results/

    # Custom title and sequence name
    python3 scripts/visualize_trajectory.py \\
        --estimated results/CameraTrajectory.txt \\
        --groundtruth groundtruth_MH01.txt \\
        --title "MH_01_easy — Stereo-IMU" \\
        --output results/

Input file format (TUM):
    # timestamp tx ty tz qx qy qz qw
    1403636579.763555527 1.788 1.045 1.502 0.012 -0.004 -0.665 0.747
    ...

Output files:
    trajectory_2d.png         — Top-down (XY) and side (XZ) views
    trajectory_3d.png         — Full 3D trajectory plot
    trajectory_comparison.png — Side-by-side estimated vs ground truth (if GT provided)
    trajectory_error.png      — Per-timestamp translation error (if GT provided)
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless/server environments
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3D projection


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def load_trajectory_tum(filepath: str) -> np.ndarray:
    """
    Load a trajectory from a TUM-format file.

    TUM format:
        # timestamp tx ty tz qx qy qz qw
        1403636579.763 1.788 1.045 1.502 0.012 -0.004 -0.665 0.747

    Returns:
        ndarray of shape (N, 8): [timestamp, tx, ty, tz, qx, qy, qz, qw]
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"Trajectory file not found: {filepath}")

    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    row = (
                    [float(x) for x in parts[:8]]
                    if len(parts) >= 8
                    else [float(x) for x in parts[:4]] + [0.0, 0.0, 0.0, 1.0]
                )
                    data.append(row)
                except ValueError:
                    continue  # Skip malformed lines

    if not data:
        raise ValueError(f"No valid trajectory data found in: {filepath}")

    return np.array(data, dtype=np.float64)


def align_trajectories(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    """
    Align estimated trajectory to ground truth using Umeyama method (similarity transform).
    Matches poses by nearest timestamp.

    Args:
        est: Estimated trajectory (N, 8)
        gt:  Ground truth trajectory (M, 8)

    Returns:
        est_aligned: Aligned estimated positions (K, 3)
        gt_matched:  Matched ground truth positions (K, 3)
    """
    est_ts = est[:, 0]
    gt_ts = gt[:, 0]

    # Match timestamps
    matched_est = []
    matched_gt = []
    for i, ts in enumerate(est_ts):
        idx = np.argmin(np.abs(gt_ts - ts))
        if np.abs(gt_ts[idx] - ts) < 0.05:  # 50ms tolerance
            matched_est.append(est[i, 1:4])
            matched_gt.append(gt[idx, 1:4])

    if len(matched_est) < 3:
        # Fall back to using all points
        matched_est = est[:, 1:4]
        n = min(len(est), len(gt))
        matched_gt = gt[:n, 1:4]
        matched_est = matched_est[:n]

    matched_est = np.array(matched_est)
    matched_gt = np.array(matched_gt)

    # Umeyama similarity transform: minimize ||s*R*p_est + t - p_gt||
    mu_est = matched_est.mean(axis=0)
    mu_gt = matched_gt.mean(axis=0)

    est_centered = matched_est - mu_est
    gt_centered = matched_gt - mu_gt

    sigma_est = np.mean(np.sum(est_centered ** 2, axis=1))

    H = est_centered.T @ gt_centered / len(matched_est)
    U, S, Vt = np.linalg.svd(H)

    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, d])

    R = Vt.T @ D @ U.T
    s = np.sum(S * np.diag(D)) / sigma_est
    t = mu_gt - s * R @ mu_est

    # Apply transform to full estimated trajectory
    est_pos = est[:, 1:4]
    est_aligned = (s * (R @ est_pos.T).T) + t

    return est_aligned, matched_gt


def compute_ate(est_aligned: np.ndarray, gt: np.ndarray, est_ts: np.ndarray, gt_ts: np.ndarray) -> np.ndarray:
    """Compute per-pose Absolute Trajectory Error after alignment."""
    errors = []
    for i, ts in enumerate(est_ts):
        idx = np.argmin(np.abs(gt_ts - ts))
        if np.abs(gt_ts[idx] - ts) < 0.05:
            err = np.linalg.norm(est_aligned[i] - gt[idx, 1:4])
            errors.append(err)
    return np.array(errors) if errors else np.zeros(1)


# ---------------------------------------------------------------------------
# Plotting functions
# ---------------------------------------------------------------------------

def plot_trajectory_2d(est_pos: np.ndarray, gt_pos: np.ndarray | None,
                       title: str, output_path: str) -> None:
    """
    Plot trajectory in 2D: XY (top-down) and XZ (side) views.

    Args:
        est_pos:    Estimated positions (N, 3)
        gt_pos:     Ground truth positions (M, 3) or None
        title:      Plot title
        output_path: Path to save the figure
    """
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(f'{title}\n2D Trajectory Visualization', fontsize=14, fontweight='bold')

    # Color by progress along trajectory
    est_n = len(est_pos)
    est_colors = cm.viridis(np.linspace(0, 1, est_n))

    for ax_idx, (ax, dims, dim_labels) in enumerate(zip(
            axes,
            [(0, 1), (0, 2)],
            [('X (m)', 'Y (m)', 'Top-Down View (XY)'),
             ('X (m)', 'Z (m)', 'Side View (XZ)')])):

        xi, yi = dims

        # Plot estimated trajectory
        for j in range(est_n - 1):
            ax.plot(est_pos[j:j+2, xi], est_pos[j:j+2, yi],
                    color=est_colors[j], linewidth=1.5, alpha=0.8)

        # Start/end markers
        ax.scatter(est_pos[0, xi], est_pos[0, yi],
                   c='lime', s=120, zorder=5, edgecolors='black', linewidth=1.5,
                   label='Start', marker='o')
        ax.scatter(est_pos[-1, xi], est_pos[-1, yi],
                   c='red', s=120, zorder=5, edgecolors='black', linewidth=1.5,
                   label='End', marker='s')

        # Plot ground truth
        if gt_pos is not None and len(gt_pos) > 0:
            ax.plot(gt_pos[:, xi], gt_pos[:, yi],
                    'k--', linewidth=1.5, alpha=0.6, label='Ground Truth', zorder=3)

        ax.set_xlabel(dim_labels[0], fontsize=11)
        ax.set_ylabel(dim_labels[1], fontsize=11)
        ax.set_title(dim_labels[2], fontsize=12)
        ax.legend(fontsize=9, loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.tick_params(labelsize=9)

    # Colorbar for time progress
    sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=0, vmax=1))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=axes.ravel().tolist(), shrink=0.6, aspect=20, pad=0.02)
    cbar.set_label('Progress along trajectory', fontsize=10)
    cbar.set_ticks([0, 0.5, 1])
    cbar.set_ticklabels(['Start', 'Mid', 'End'])

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved 2D trajectory plot: {output_path}")


def plot_trajectory_3d(est_pos: np.ndarray, gt_pos: np.ndarray | None,
                       title: str, output_path: str) -> None:
    """
    Plot trajectory in 3D.

    Args:
        est_pos:    Estimated positions (N, 3)
        gt_pos:     Ground truth positions (M, 3) or None
        title:      Plot title
        output_path: Path to save the figure
    """
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Color by progress
    est_n = len(est_pos)
    colors = cm.viridis(np.linspace(0, 1, est_n))

    for j in range(est_n - 1):
        ax.plot(est_pos[j:j+2, 0], est_pos[j:j+2, 1], est_pos[j:j+2, 2],
                color=colors[j], linewidth=1.5, alpha=0.85)

    # Start/end markers
    ax.scatter(*est_pos[0], c='lime', s=150, zorder=5, edgecolors='black',
               linewidth=1.5, label='Start')
    ax.scatter(*est_pos[-1], c='red', s=150, zorder=5, edgecolors='black',
               linewidth=1.5, label='End')

    # Ground truth
    if gt_pos is not None and len(gt_pos) > 0:
        ax.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2],
                'k--', linewidth=1.5, alpha=0.6, label='Ground Truth')

    ax.set_xlabel('X (m)', fontsize=11, labelpad=8)
    ax.set_ylabel('Y (m)', fontsize=11, labelpad=8)
    ax.set_zlabel('Z (m)', fontsize=11, labelpad=8)
    ax.set_title(f'{title}\n3D Trajectory', fontsize=13, fontweight='bold', pad=15)
    ax.legend(fontsize=10, loc='upper left')
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=8)

    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=0, vmax=1))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.5, aspect=15, pad=0.1)
    cbar.set_label('Progress', fontsize=9)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved 3D trajectory plot: {output_path}")


def plot_trajectory_comparison(est_pos: np.ndarray, gt_pos: np.ndarray,
                                est_aligned: np.ndarray,
                                title: str, output_path: str) -> None:
    """
    Plot side-by-side comparison: estimated (before/after alignment) vs ground truth.
    """
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(f'{title}\nEstimated vs Ground Truth Comparison', fontsize=14, fontweight='bold')

    views = [
        (0, 1, 'Top-Down XY'),
        (0, 2, 'Side XZ'),
    ]

    for ax, (xi, yi, view_name) in zip(axes, views):
        # Ground truth
        ax.plot(gt_pos[:, xi], gt_pos[:, yi],
                color='black', linewidth=2.0, alpha=0.8, label='Ground Truth', zorder=4)
        # Aligned estimate
        ax.plot(est_aligned[:, xi], est_aligned[:, yi],
                color='royalblue', linewidth=1.8, alpha=0.85, label='Estimated (aligned)', zorder=5)

        # Start markers
        ax.scatter(gt_pos[0, xi], gt_pos[0, yi],
                   c='lime', s=120, zorder=6, edgecolors='black', linewidth=1.5)
        ax.scatter(est_aligned[0, xi], est_aligned[0, yi],
                   c='deepskyblue', s=120, zorder=6, edgecolors='black', linewidth=1.5)

        ax.set_xlabel('X (m)', fontsize=11)
        ax.set_ylabel('Y (m)' if yi == 1 else 'Z (m)', fontsize=11)
        ax.set_title(view_name, fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved trajectory comparison: {output_path}")


def plot_error_over_time(errors: np.ndarray, timestamps: np.ndarray,
                          title: str, output_path: str) -> None:
    """
    Plot ATE over time.
    """
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle(f'{title}\nTranslation Error Over Time', fontsize=13, fontweight='bold')

    t = timestamps - timestamps[0]  # Relative time

    # Error timeseries
    ax = axes[0]
    ax.plot(t[:len(errors)], errors, color='royalblue', linewidth=1.5, alpha=0.8)
    ax.fill_between(t[:len(errors)], 0, errors, alpha=0.2, color='royalblue')
    ax.axhline(y=errors.mean(), color='red', linestyle='--', linewidth=1.5,
               label=f'Mean: {errors.mean():.4f} m')
    ax.axhline(y=np.sqrt(np.mean(errors**2)), color='orange', linestyle='--', linewidth=1.5,
               label=f'RMSE: {np.sqrt(np.mean(errors**2)):.4f} m')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('ATE (m)', fontsize=11)
    ax.set_title('Absolute Trajectory Error', fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # Histogram
    ax2 = axes[1]
    ax2.hist(errors, bins=50, color='steelblue', alpha=0.7, edgecolor='white')
    ax2.axvline(x=errors.mean(), color='red', linestyle='--', linewidth=1.5,
                label=f'Mean: {errors.mean():.4f} m')
    ax2.axvline(x=np.percentile(errors, 95), color='orange', linestyle='--', linewidth=1.5,
                label=f'95th pct: {np.percentile(errors, 95):.4f} m')
    ax2.set_xlabel('ATE (m)', fontsize=11)
    ax2.set_ylabel('Count', fontsize=11)
    ax2.set_title('ATE Distribution', fontsize=12)
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved error plot: {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Visualize ORB-SLAM3 trajectory (2D + 3D plots)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--estimated', '-e', required=True,
                        help='Path to estimated trajectory (TUM format)')
    parser.add_argument('--groundtruth', '-g', default=None,
                        help='Path to ground truth trajectory (TUM format, optional)')
    parser.add_argument('--output', '-o', default='results/',
                        help='Output directory for plots (default: results/)')
    parser.add_argument('--title', '-t', default='UAV Trajectory — ORB-SLAM3 VIO',
                        help='Title for plots')
    parser.add_argument('--show', action='store_true', default=False,
                        help='Display plots interactively in addition to saving')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"  UAV Trajectory Visualizer")
    print(f"{'='*60}")
    print(f"  Estimated:    {args.estimated}")
    print(f"  Ground truth: {args.groundtruth or 'Not provided'}")
    print(f"  Output dir:   {args.output}")
    print(f"  Title:        {args.title}")
    print(f"{'='*60}\n")

    # Load estimated trajectory
    print("[1/5] Loading estimated trajectory...")
    est = load_trajectory_tum(args.estimated)
    est_pos = est[:, 1:4]
    print(f"      Loaded {len(est)} poses, duration: {est[-1,0]-est[0,0]:.1f}s")

    # Load ground truth (optional)
    gt = None
    gt_pos = None
    if args.groundtruth:
        print("[2/5] Loading ground truth trajectory...")
        gt = load_trajectory_tum(args.groundtruth)
        gt_pos = gt[:, 1:4]
        print(f"      Loaded {len(gt)} poses, duration: {gt[-1,0]-gt[0,0]:.1f}s")
    else:
        print("[2/5] No ground truth provided — skipping alignment and error analysis")

    # Align trajectories
    est_aligned = est_pos.copy()
    gt_matched = None
    if gt is not None:
        print("[3/5] Aligning trajectories (Umeyama similarity transform)...")
        try:
            est_aligned, gt_matched = align_trajectories(est, gt)
            print(f"      Alignment successful ({len(est_aligned)} matched poses)")
        except Exception as ex:
            print(f"      Warning: Alignment failed ({ex}), using raw positions")
            est_aligned = est_pos
    else:
        print("[3/5] Skipping alignment (no ground truth)")

    print("[4/5] Generating plots...")

    # 2D plot
    plot_trajectory_2d(
        est_pos=est_aligned,
        gt_pos=gt_pos,
        title=args.title,
        output_path=os.path.join(args.output, 'trajectory_2d.png')
    )

    # 3D plot
    plot_trajectory_3d(
        est_pos=est_aligned,
        gt_pos=gt_pos,
        title=args.title,
        output_path=os.path.join(args.output, 'trajectory_3d.png')
    )

    # Comparison plot (only if GT available)
    if gt_pos is not None:
        plot_trajectory_comparison(
            est_pos=est_pos,
            gt_pos=gt_pos,
            est_aligned=est_aligned,
            title=args.title,
            output_path=os.path.join(args.output, 'trajectory_comparison.png')
        )

        # Error over time
        errors = compute_ate(est_aligned, gt, est[:, 0], gt[:, 0])
        if len(errors) > 0:
            plot_error_over_time(
                errors=errors,
                timestamps=est[:len(errors), 0],
                title=args.title,
                output_path=os.path.join(args.output, 'trajectory_error.png')
            )
            print(f"\n[5/5] Error statistics:")
            print(f"      ATE RMSE:   {np.sqrt(np.mean(errors**2)):.4f} m")
            print(f"      ATE Mean:   {errors.mean():.4f} m")
            print(f"      ATE Median: {np.median(errors):.4f} m")
            print(f"      ATE Std:    {errors.std():.4f} m")
            print(f"      ATE Max:    {errors.max():.4f} m")
            print(f"      ATE Min:    {errors.min():.4f} m")
        else:
            print("[5/5] Could not compute ATE (timestamp mismatch)")
    else:
        print("[5/5] Skipping error analysis (no ground truth)")

    print(f"\n{'='*60}")
    print(f"  Done! Plots saved to: {args.output}")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()
