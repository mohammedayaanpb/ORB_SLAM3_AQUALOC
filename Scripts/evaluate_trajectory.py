#!/usr/bin/env python3
"""
Trajectory Evaluation Wrapper for ORB-SLAM3 Results
Uses evo package for ATE and RPE computation against ground truth.
Falls back to standalone metrics if evo is not available.
"""

import os
import sys
import argparse
import subprocess
import numpy as np
from pathlib import Path

def check_evo_installed():
    """Check if evo package is available."""
    try:
        subprocess.run(['evo_ape', '--help'], capture_output=True, check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def load_trajectory_tum(filepath):
    """Load trajectory in TUM format: timestamp tx ty tz qx qy qz qw"""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                timestamp = float(parts[0])
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                data.append([timestamp, tx, ty, tz])
    return np.array(data)

def align_trajectories(est_traj, gt_traj, max_time_diff=0.1):
    """
    Align estimated trajectory to ground truth by timestamps.
    Returns aligned pairs.
    """
    aligned_est = []
    aligned_gt = []
    
    for est_point in est_traj:
        est_time = est_point[0]
        
        # Find closest ground truth point
        time_diffs = np.abs(gt_traj[:, 0] - est_time)
        min_idx = np.argmin(time_diffs)
        
        if time_diffs[min_idx] < max_time_diff:
            aligned_est.append(est_point[1:4])
            aligned_gt.append(gt_traj[min_idx, 1:4])
    
    return np.array(aligned_est), np.array(aligned_gt)

def compute_ate_manual(est_traj, gt_traj):
    """
    Compute Absolute Trajectory Error (ATE) manually.
    Uses Umeyama alignment to find best rigid transformation.
    """
    aligned_est, aligned_gt = align_trajectories(est_traj, gt_traj)
    
    if len(aligned_est) < 3:
        return None, None, None
    
    # Umeyama alignment (simplified - translation and scale only)
    est_centered = aligned_est - aligned_est.mean(axis=0)
    gt_centered = aligned_gt - aligned_gt.mean(axis=0)
    
    # Compute scale
    est_scale = np.sqrt(np.sum(est_centered**2) / len(est_centered))
    gt_scale = np.sqrt(np.sum(gt_centered**2) / len(gt_centered))
    scale = gt_scale / est_scale if est_scale > 0 else 1.0
    
    # Apply scale and compute translation
    est_scaled = aligned_est * scale
    translation = aligned_gt.mean(axis=0) - est_scaled.mean(axis=0)
    
    # Aligned estimate
    est_aligned = est_scaled + translation
    
    # Compute errors
    errors = np.linalg.norm(est_aligned - aligned_gt, axis=1)
    
    ate_rmse = np.sqrt(np.mean(errors**2))
    ate_mean = np.mean(errors)
    ate_std = np.std(errors)
    
    return ate_rmse, ate_mean, ate_std

def compute_rpe_manual(est_traj, gt_traj, delta=1):
    """
    Compute Relative Pose Error (RPE) manually.
    Delta specifies the frame interval for relative poses.
    """
    aligned_est, aligned_gt = align_trajectories(est_traj, gt_traj)
    
    if len(aligned_est) < delta + 1:
        return None, None, None
    
    trans_errors = []
    
    for i in range(len(aligned_est) - delta):
        # Relative motion in estimate
        est_rel = aligned_est[i + delta] - aligned_est[i]
        
        # Relative motion in ground truth
        gt_rel = aligned_gt[i + delta] - aligned_gt[i]
        
        # Translation error
        trans_error = np.linalg.norm(est_rel - gt_rel)
        trans_errors.append(trans_error)
    
    trans_errors = np.array(trans_errors)
    
    rpe_rmse = np.sqrt(np.mean(trans_errors**2))
    rpe_mean = np.mean(trans_errors)
    rpe_std = np.std(trans_errors)
    
    return rpe_rmse, rpe_mean, rpe_std

def run_evo_evaluation(est_file, gt_file, output_dir, align=True):
    """Run evaluation using evo package."""
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    results = {}
    
    # ATE evaluation
    ate_cmd = [
        'evo_ape', 'tum', str(gt_file), str(est_file),
        '--save_results', str(output_path / 'ate_results.zip'),
        '--plot_mode', 'xyz',
        '--save_plot', str(output_path / 'ate_plot.png')
    ]
    if align:
        ate_cmd.append('--align')
    
    try:
        result = subprocess.run(ate_cmd, capture_output=True, text=True)
        print("ATE Results (evo):")
        print(result.stdout)
        results['ate_output'] = result.stdout
        
        # Save text output
        with open(output_path / 'ate_results.txt', 'w') as f:
            f.write(result.stdout)
    except Exception as e:
        print(f"ATE evaluation error: {e}")
    
    # RPE evaluation
    rpe_cmd = [
        'evo_rpe', 'tum', str(gt_file), str(est_file),
        '--save_results', str(output_path / 'rpe_results.zip'),
        '--plot_mode', 'xyz',
        '--save_plot', str(output_path / 'rpe_plot.png'),
        '--delta', '1',
        '--delta_unit', 'f'  # frames
    ]
    if align:
        rpe_cmd.append('--align')
    
    try:
        result = subprocess.run(rpe_cmd, capture_output=True, text=True)
        print("\nRPE Results (evo):")
        print(result.stdout)
        results['rpe_output'] = result.stdout
        
        with open(output_path / 'rpe_results.txt', 'w') as f:
            f.write(result.stdout)
    except Exception as e:
        print(f"RPE evaluation error: {e}")
    
    return results

def run_manual_evaluation(est_file, gt_file, output_dir):
    """Run evaluation using manual implementation."""
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Load trajectories
    est_traj = load_trajectory_tum(est_file)
    gt_traj = load_trajectory_tum(gt_file)
    
    print(f"Loaded {len(est_traj)} estimated poses")
    print(f"Loaded {len(gt_traj)} ground truth poses")
    
    if len(est_traj) == 0:
        print("Error: No estimated trajectory data!")
        return None
    
    results = {}
    
    # Compute ATE
    if len(gt_traj) > 0:
        ate_rmse, ate_mean, ate_std = compute_ate_manual(est_traj, gt_traj)
        
        if ate_rmse is not None:
            results['ate'] = {
                'rmse': ate_rmse,
                'mean': ate_mean,
                'std': ate_std
            }
            
            print("\n" + "="*50)
            print("ATE Results (Manual)")
            print("="*50)
            print(f"  RMSE:  {ate_rmse:.4f} m")
            print(f"  Mean:  {ate_mean:.4f} m")
            print(f"  Std:   {ate_std:.4f} m")
        
        # Compute RPE
        rpe_rmse, rpe_mean, rpe_std = compute_rpe_manual(est_traj, gt_traj)
        
        if rpe_rmse is not None:
            results['rpe'] = {
                'rmse': rpe_rmse,
                'mean': rpe_mean,
                'std': rpe_std
            }
            
            print("\n" + "="*50)
            print("RPE Results (Manual)")
            print("="*50)
            print(f"  RMSE:  {rpe_rmse:.4f} m")
            print(f"  Mean:  {rpe_mean:.4f} m")
            print(f"  Std:   {rpe_std:.4f} m")
    else:
        print("No ground truth provided - computing trajectory statistics only")
        
        # Trajectory statistics without ground truth
        traj_length = 0
        if len(est_traj) > 1:
            positions = est_traj[:, 1:4]
            diffs = np.diff(positions, axis=0)
            traj_length = np.sum(np.linalg.norm(diffs, axis=1))
        
        results['stats'] = {
            'num_poses': len(est_traj),
            'trajectory_length': traj_length,
            'duration': est_traj[-1, 0] - est_traj[0, 0] if len(est_traj) > 1 else 0
        }
        
        print("\n" + "="*50)
        print("Trajectory Statistics")
        print("="*50)
        print(f"  Poses:     {results['stats']['num_poses']}")
        print(f"  Length:    {results['stats']['trajectory_length']:.3f} m")
        print(f"  Duration:  {results['stats']['duration']:.2f} s")
    
    # Save results
    results_file = output_path / 'evaluation_results.txt'
    with open(results_file, 'w') as f:
        f.write("Trajectory Evaluation Results\n")
        f.write("="*50 + "\n\n")
        f.write(f"Estimated trajectory: {est_file}\n")
        f.write(f"Ground truth: {gt_file}\n\n")
        
        if 'ate' in results:
            f.write("ATE (Absolute Trajectory Error):\n")
            f.write(f"  RMSE: {results['ate']['rmse']:.4f} m\n")
            f.write(f"  Mean: {results['ate']['mean']:.4f} m\n")
            f.write(f"  Std:  {results['ate']['std']:.4f} m\n\n")
        
        if 'rpe' in results:
            f.write("RPE (Relative Pose Error):\n")
            f.write(f"  RMSE: {results['rpe']['rmse']:.4f} m\n")
            f.write(f"  Mean: {results['rpe']['mean']:.4f} m\n")
            f.write(f"  Std:  {results['rpe']['std']:.4f} m\n\n")
        
        if 'stats' in results:
            f.write("Trajectory Statistics:\n")
            f.write(f"  Poses:    {results['stats']['num_poses']}\n")
            f.write(f"  Length:   {results['stats']['trajectory_length']:.3f} m\n")
            f.write(f"  Duration: {results['stats']['duration']:.2f} s\n")
    
    print(f"\nResults saved to: {results_file}")
    
    return results

def main():
    parser = argparse.ArgumentParser(
        description='Evaluate ORB-SLAM3 trajectory against ground truth'
    )
    parser.add_argument('--estimated', '-e', required=True,
                        help='Estimated trajectory file (TUM format)')
    parser.add_argument('--groundtruth', '-g', default=None,
                        help='Ground truth trajectory file (TUM format)')
    parser.add_argument('--output', '-o', required=True,
                        help='Output directory for results')
    parser.add_argument('--no-align', action='store_true',
                        help='Disable trajectory alignment')
    parser.add_argument('--force-manual', action='store_true',
                        help='Use manual evaluation even if evo is available')
    
    args = parser.parse_args()
    
    est_file = Path(args.estimated).expanduser()
    gt_file = Path(args.groundtruth).expanduser() if args.groundtruth else None
    output_dir = Path(args.output).expanduser()
    
    if not est_file.exists():
        print(f"Error: Estimated trajectory not found: {est_file}")
        sys.exit(1)
    
    if gt_file and not gt_file.exists():
        print(f"Warning: Ground truth not found: {gt_file}")
        gt_file = None
    
    # Check if evo is available
    use_evo = check_evo_installed() and not args.force_manual
    
    if use_evo and gt_file:
        print("Using evo package for evaluation...")
        run_evo_evaluation(est_file, gt_file, output_dir, align=not args.no_align)
    else:
        if not use_evo:
            print("evo package not found, using manual evaluation...")
        run_manual_evaluation(est_file, gt_file, output_dir)

if __name__ == '__main__':
    main()
