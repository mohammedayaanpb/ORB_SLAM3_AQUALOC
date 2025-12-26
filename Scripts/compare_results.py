#!/usr/bin/env python3
"""Compare Raw vs CLAHE SLAM results"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for VirtualBox
import matplotlib.pyplot as plt
from pathlib import Path

def load_trajectory(filepath):
    data = []
    if not Path(filepath).exists():
        return np.array([])
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 4:
                ts = float(parts[0])
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                data.append([ts, tx, ty, tz])
    return np.array(data) if data else np.array([])

def compute_length(traj):
    if len(traj) < 2:
        return 0
    diffs = np.diff(traj[:, 1:4], axis=0)
    return np.sum(np.linalg.norm(diffs, axis=1))

def main():
    results_dir = Path.home() / 'ORB_SLAM3_AQUALOC' / 'Results'
    
    raw_file = results_dir / 'Raw/Harbor_seq01/KeyFrameTrajectory.txt'
    clahe_file = results_dir / 'CLAHE/Harbor_seq01/KeyFrameTrajectory.txt'
    
    raw_traj = load_trajectory(raw_file)
    clahe_traj = load_trajectory(clahe_file)
    
    print("\n" + "="*65)
    print("       COMPARISON: Raw vs CLAHE Preprocessing")
    print("="*65)
    
    # Basic stats
    raw_kf = len(raw_traj)
    clahe_kf = len(clahe_traj)
    
    raw_dur = (raw_traj[-1, 0] - raw_traj[0, 0]) if raw_kf > 1 else 0
    clahe_dur = (clahe_traj[-1, 0] - clahe_traj[0, 0]) if clahe_kf > 1 else 0
    
    raw_len = compute_length(raw_traj)
    clahe_len = compute_length(clahe_traj)
    
    print(f"\n{'Metric':<30} {'Raw':<15} {'CLAHE':<15} {'Diff':<15}")
    print("-"*65)
    print(f"{'Keyframes Tracked':<30} {raw_kf:<15} {clahe_kf:<15} {clahe_kf - raw_kf:+d}")
    print(f"{'Tracking Duration (s)':<30} {raw_dur:<15.1f} {clahe_dur:<15.1f} {clahe_dur - raw_dur:+.1f}")
    print(f"{'Trajectory Length (m)':<30} {raw_len:<15.3f} {clahe_len:<15.3f} {clahe_len - raw_len:+.3f}")
    
    if raw_kf > 0:
        raw_kf_rate = raw_kf / raw_dur if raw_dur > 0 else 0
        clahe_kf_rate = clahe_kf / clahe_dur if clahe_dur > 0 else 0
        print(f"{'Keyframe Rate (kf/s)':<30} {raw_kf_rate:<15.2f} {clahe_kf_rate:<15.2f}")
    
    print("="*65)
    
    # Determine winner
    print("\n>>> ANALYSIS:")
    if clahe_kf > raw_kf:
        print(f"    CLAHE tracked {clahe_kf - raw_kf} MORE keyframes ({(clahe_kf/raw_kf - 1)*100:.1f}% improvement)")
    elif raw_kf > clahe_kf:
        print(f"    Raw tracked {raw_kf - clahe_kf} MORE keyframes ({(raw_kf/clahe_kf - 1)*100:.1f}% better)")
    else:
        print("    Both methods tracked the same number of keyframes")
    
    if clahe_dur > raw_dur:
        print(f"    CLAHE maintained tracking {clahe_dur - raw_dur:.1f}s LONGER")
    elif raw_dur > clahe_dur:
        print(f"    Raw maintained tracking {raw_dur - clahe_dur:.1f}s LONGER")
    
    # Plot trajectories
    if len(raw_traj) > 0 or len(clahe_traj) > 0:
        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        fig.suptitle('ORB-SLAM3 Trajectory Comparison: Raw vs CLAHE\nAQUALOC Harbor Sequence 01', fontsize=14)
        
        # XY plot
        ax = axes[0, 0]
        if len(raw_traj) > 0:
            ax.plot(raw_traj[:, 1], raw_traj[:, 2], 'b-', label=f'Raw ({raw_kf} kf)', linewidth=1.5, alpha=0.8)
            ax.scatter(raw_traj[0, 1], raw_traj[0, 2], c='blue', s=100, marker='o', zorder=5, edgecolors='black')
        if len(clahe_traj) > 0:
            ax.plot(clahe_traj[:, 1], clahe_traj[:, 2], 'r-', label=f'CLAHE ({clahe_kf} kf)', linewidth=1.5, alpha=0.8)
            ax.scatter(clahe_traj[0, 1], clahe_traj[0, 2], c='red', s=100, marker='o', zorder=5, edgecolors='black')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Top-Down View (XY Plane)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # XZ plot
        ax = axes[0, 1]
        if len(raw_traj) > 0:
            ax.plot(raw_traj[:, 1], raw_traj[:, 3], 'b-', label='Raw', linewidth=1.5, alpha=0.8)
        if len(clahe_traj) > 0:
            ax.plot(clahe_traj[:, 1], clahe_traj[:, 3], 'r-', label='CLAHE', linewidth=1.5, alpha=0.8)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Z (m)')
        ax.set_title('Side View (XZ Plane)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # YZ plot
        ax = axes[1, 0]
        if len(raw_traj) > 0:
            ax.plot(raw_traj[:, 2], raw_traj[:, 3], 'b-', label='Raw', linewidth=1.5, alpha=0.8)
        if len(clahe_traj) > 0:
            ax.plot(clahe_traj[:, 2], clahe_traj[:, 3], 'r-', label='CLAHE', linewidth=1.5, alpha=0.8)
        ax.set_xlabel('Y (m)')
        ax.set_ylabel('Z (m)')
        ax.set_title('Front View (YZ Plane)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Statistics bar chart
        ax = axes[1, 1]
        metrics = ['Keyframes', 'Duration (s)', 'Length (m)']
        raw_vals = [raw_kf, raw_dur, raw_len]
        clahe_vals = [clahe_kf, clahe_dur, clahe_len]
        
        x = np.arange(len(metrics))
        width = 0.35
        
        bars1 = ax.bar(x - width/2, raw_vals, width, label='Raw', color='blue', alpha=0.7)
        bars2 = ax.bar(x + width/2, clahe_vals, width, label='CLAHE', color='red', alpha=0.7)
        
        ax.set_ylabel('Value')
        ax.set_title('Performance Metrics')
        ax.set_xticks(x)
        ax.set_xticklabels(metrics)
        ax.legend()
        
        # Add value labels on bars
        for bar, val in zip(bars1, raw_vals):
            ax.annotate(f'{val:.1f}', xy=(bar.get_x() + bar.get_width()/2, bar.get_height()),
                       ha='center', va='bottom', fontsize=9)
        for bar, val in zip(bars2, clahe_vals):
            ax.annotate(f'{val:.1f}', xy=(bar.get_x() + bar.get_width()/2, bar.get_height()),
                       ha='center', va='bottom', fontsize=9)
        
        plt.tight_layout()
        output_path = results_dir / 'comparison_raw_vs_clahe.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\n>>> Plot saved: {output_path}")
        plt.close()

if __name__ == '__main__':
    main()
