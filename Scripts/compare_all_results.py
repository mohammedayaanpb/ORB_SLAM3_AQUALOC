#!/usr/bin/env python3
"""Compare Raw vs CLAHE vs FUnIE-GAN SLAM results"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

def load_trajectory(filepath):
    data = []
    if not Path(filepath).exists():
        print(f"Warning: {filepath} not found")
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

def main():
    results_dir = Path.home() / 'ORB_SLAM3_AQUALOC' / 'Results'
    
    raw_traj = load_trajectory(results_dir / 'Raw/Harbor_seq01/KeyFrameTrajectory.txt')
    clahe_traj = load_trajectory(results_dir / 'CLAHE/Harbor_seq01/KeyFrameTrajectory.txt')
    funie_traj = load_trajectory(results_dir / 'FUnIE_GAN/Harbor_seq01/KeyFrameTrajectory.txt')
    
    methods = ['Raw', 'CLAHE', 'FUnIE-GAN']
    trajs = [raw_traj, clahe_traj, funie_traj]
    colors = ['blue', 'orange', 'green']
    
    print("\n" + "="*70)
    print("       THREE-WAY COMPARISON: Raw vs CLAHE vs FUnIE-GAN")
    print("="*70)
    
    kfs = [len(t) for t in trajs]
    durs = [(t[-1,0] - t[0,0]) if len(t) > 1 else 0 for t in trajs]
    
    print(f"\n{'Metric':<25} {'Raw':<12} {'CLAHE':<12} {'FUnIE-GAN':<12}")
    print("-"*70)
    print(f"{'Keyframes Tracked':<25} {kfs[0]:<12} {kfs[1]:<12} {kfs[2]:<12}")
    print(f"{'Duration (s)':<25} {durs[0]:<12.1f} {durs[1]:<12.1f} {durs[2]:<12.1f}")
    print("="*70)
    
    if kfs[0] > 0:
        print(f"\n>>> PERFORMANCE vs RAW:")
        clahe_pct = (kfs[1]-kfs[0])/kfs[0]*100
        funie_pct = (kfs[2]-kfs[0])/kfs[0]*100
        print(f"    CLAHE:     {clahe_pct:+.1f}% ({kfs[1]-kfs[0]:+d} keyframes)")
        print(f"    FUnIE-GAN: {funie_pct:+.1f}% ({kfs[2]-kfs[0]:+d} keyframes)")
    
    best_idx = np.argmax(kfs)
    print(f"\n>>> BEST METHOD: {methods[best_idx]} ({kfs[best_idx]} keyframes)")
    
    # Create plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle('ORB-SLAM3 Trajectory Comparison\nAQUALOC Harbor Sequence 01', fontsize=14, fontweight='bold')
    
    # XY Plot
    ax = axes[0, 0]
    for traj, method, color in zip(trajs, methods, colors):
        if len(traj) > 0:
            ax.plot(traj[:, 1], traj[:, 2], color=color, label=f'{method} ({len(traj)} kf)', linewidth=1.5)
            ax.scatter(traj[0, 1], traj[0, 2], c=color, s=100, marker='o', edgecolors='black', zorder=5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Top-Down View (XY Plane)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # XZ Plot
    ax = axes[0, 1]
    for traj, method, color in zip(trajs, methods, colors):
        if len(traj) > 0:
            ax.plot(traj[:, 1], traj[:, 3], color=color, label=method, linewidth=1.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Side View (XZ Plane)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # YZ Plot
    ax = axes[1, 0]
    for traj, method, color in zip(trajs, methods, colors):
        if len(traj) > 0:
            ax.plot(traj[:, 2], traj[:, 3], color=color, label=method, linewidth=1.5)
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Front View (YZ Plane)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Bar Chart
    ax = axes[1, 1]
    x = np.arange(len(methods))
    bars = ax.bar(x, kfs, color=colors, alpha=0.7, edgecolor='black', linewidth=2)
    ax.set_ylabel('Keyframes Tracked', fontsize=12)
    ax.set_title('Keyframes by Preprocessing Method')
    ax.set_xticks(x)
    ax.set_xticklabels(methods, fontsize=11)
    for bar, kf in zip(bars, kfs):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 5, 
                str(kf), ha='center', fontweight='bold', fontsize=12)
    
    plt.tight_layout()
    output_path = results_dir / 'comparison_all_methods.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n>>> Plot saved: {output_path}")
    plt.close()

if __name__ == '__main__':
    main()
