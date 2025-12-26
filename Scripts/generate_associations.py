#!/usr/bin/env python3
"""
Generate association files for AQUALOC sequences.
"""

import argparse
from pathlib import Path

def generate_association(sequence_dir, image_dir, output_file):
    sequence_path = Path(sequence_dir).expanduser().resolve()
    image_path = Path(image_dir).expanduser().resolve()
    
    print(f"Sequence dir: {sequence_path}")
    print(f"Image dir: {image_path}")
    
    # List all CSV files
    all_csv = list(sequence_path.glob('*.csv'))
    print(f"Found CSV files: {[f.name for f in all_csv]}")
    
    # Find the image timestamps CSV (contains 'img' in name)
    ts_file = None
    for csv_file in all_csv:
        if 'img' in csv_file.name.lower():
            ts_file = csv_file
            break
    
    if not ts_file:
        print("ERROR: No image timestamps CSV found!")
        return 0
    
    print(f"Using timestamps file: {ts_file}")
    
    associations = []
    with open(ts_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split(',')
            if len(parts) < 2:
                continue
            
            timestamp_ns = parts[0].strip()
            frame_name = parts[1].strip()
            
            try:
                timestamp_sec = float(timestamp_ns) / 1e9
            except ValueError:
                continue
            
            img_path = image_path / frame_name
            if img_path.exists():
                associations.append((timestamp_sec, str(img_path)))
    
    with open(output_file, 'w') as f:
        for ts, img in associations:
            f.write(f"{ts:.6f} {img}\n")
    
    print(f"Generated {len(associations)} associations -> {output_file}")
    if associations:
        print(f"Time range: {associations[0][0]:.3f}s to {associations[-1][0]:.3f}s")
    return len(associations)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sequence', '-s', required=True)
    parser.add_argument('--images', '-i', required=True)
    parser.add_argument('--output', '-o', required=True)
    args = parser.parse_args()
    generate_association(args.sequence, args.images, args.output)
