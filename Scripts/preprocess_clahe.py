#!/usr/bin/env python3
"""
CLAHE Preprocessing for AQUALOC Dataset
Applies Contrast Limited Adaptive Histogram Equalization to underwater images
"""

import cv2
import os
import argparse
from pathlib import Path

def apply_clahe(image, clip_limit=2.0, tile_grid_size=(8, 8)):
    """Apply CLAHE to a grayscale image."""
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    return clahe.apply(image)

def process_sequence(input_dir, output_dir, clip_limit=2.0, tile_size=8):
    """Process all images in a sequence directory."""
    
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Find all images (AQUALOC uses .png)
    image_extensions = ['.png', '.jpg', '.jpeg', '.PNG', '.JPG']
    image_files = []
    for ext in image_extensions:
        image_files.extend(input_path.glob(f'*{ext}'))
    
    image_files = sorted(image_files)
    
    if not image_files:
        print(f"No images found in {input_dir}")
        return 0
    
    print(f"Processing {len(image_files)} images...")
    print(f"CLAHE parameters: clip_limit={clip_limit}, tile_size={tile_size}x{tile_size}")
    
    processed_count = 0
    for img_path in image_files:
        # Read image (grayscale for AQUALOC)
        img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        
        if img is None:
            print(f"Warning: Could not read {img_path}")
            continue
        
        # Apply CLAHE
        enhanced = apply_clahe(img, clip_limit, (tile_size, tile_size))
        
        # Save with same filename
        output_file = output_path / img_path.name
        cv2.imwrite(str(output_file), enhanced)
        processed_count += 1
        
        if processed_count % 100 == 0:
            print(f"  Processed {processed_count}/{len(image_files)} images")
    
    print(f"Completed: {processed_count} images saved to {output_dir}")
    return processed_count

def main():
    parser = argparse.ArgumentParser(description='Apply CLAHE to AQUALOC sequences')
    parser.add_argument('--input', '-i', required=True, help='Input image directory')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--clip-limit', '-c', type=float, default=2.0,
                        help='CLAHE clip limit (default: 2.0)')
    parser.add_argument('--tile-size', '-t', type=int, default=8,
                        help='CLAHE tile grid size (default: 8)')
    
    args = parser.parse_args()
    
    process_sequence(args.input, args.output, args.clip_limit, args.tile_size)

if __name__ == '__main__':
    main()
