# Underwater Visual SLAM with Image Enhancement

[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange)](https://ubuntu.com/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.4.0-green)](https://opencv.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository presents a comparative study of image enhancement techniques for improving visual SLAM performance in challenging underwater environments. We integrate **CLAHE (Contrast Limited Adaptive Histogram Equalization)** and **FUnIE-GAN** preprocessing with **ORB-SLAM3** to address the fundamental limitations of feature-based SLAM systems underwater.

## Overview

Underwater visual SLAM faces unique challenges including poor visibility, turbidity, light absorption, backscattering, and color distortion. Rather than modifying SLAM algorithms or relying on expensive sensor fusion, this project tackles the root cause by enhancing image quality as a preprocessing step before SLAM processing.

### Key Contributions

- **Systematic comparison** of raw, CLAHE, and FUnIE-GAN preprocessing approaches on the AQUALOC dataset
- **Custom ORB-SLAM3 integration** with AQUALOC timestamp formats and camera configurations
- **Quantitative evaluation framework** using keyframe tracking metrics and trajectory accuracy (ATE)
- **Optimized ORB parameters** for underwater imagery (4000 features, FAST thresholds 10/3)

### Results Summary

| Preprocessing | Keyframes Tracked | Improvement | Initial Map Points |
|--------------|-------------------|-------------|-------------------|
| Raw          | 502               | Baseline    | 423               |
| CLAHE        | 626               | **+24.7%**  | 825 (+95%)        |
| FUnIE-GAN    | 565               | +12.5%      | 687 (+62%)        |

*Evaluated on AQUALOC Harbor Sequence 01 (4,586 frames)*

## Project Structure

```
ORB_SLAM3_AQUALOC/
├── Configs/                          # Camera configuration files
│   ├── archaeological_pinhole.yaml   # Archaeological environment config
│   └── harbor_fisheye.yaml           # Harbor environment config
│
├── Datasets/
│   └── AQUALOC/
│       └── Harbor/
│           └── sequence_01/
│               ├── images/           # Grayscale frames (640x512)
│               ├── harbor_depth_sequence_01.csv
│               ├── harbor_img_sequence_01.csv
│               ├── harbor_imu_sequence_01.csv
│               └── harbor_mag_sequence_01.csv
│
├── ORB_SLAM3/                        # ORB-SLAM3 (aliaxam153 fork)
│   ├── src/                          # Core SLAM implementation
│   ├── include/                      # Header files
│   ├── lib/                          # Compiled library
│   ├── Examples/                     # Executable examples
│   ├── Vocabulary/                   # ORB vocabulary
│   ├── evaluation/                   # Evaluation scripts
│   │   ├── evaluate_ate_scale.py    # ATE calculation
│   │   └── associate.py             # Timestamp association
│   ├── build.sh                      # Build script
│   └── build_ros.sh                  # ROS workspace build
│
├── Preprocessed/
│   └── CLAHE/
│       └── Harbor/
│           └── sequence_01/          # CLAHE-enhanced images
│
├── Results/
│   ├── CLAHE/
│   │   └── Harbor_seq01/             # CLAHE experiment results
│   └── Raw/
│       └── Harbor_seq01/             # Raw baseline results
│
└── Scripts/
    ├── preprocess_clahe.py           # CLAHE preprocessing
    ├── generate_associations.py      # Create association files
    ├── run_slam_raw.sh               # Run SLAM on raw images
    ├── run_slam_clahe.sh             # Run SLAM on CLAHE images
    ├── evaluate_trajectory.py        # Trajectory evaluation
    └── compare_results.py            # Compare preprocessing methods
```

## Prerequisites

### System Requirements
- Ubuntu 20.04 LTS
- 8GB+ RAM recommended
- GPU optional (required for FUnIE-GAN preprocessing)

### Dependencies

```bash
# Core dependencies
sudo apt-get install build-essential cmake git
sudo apt-get install libopencv-dev libopencv-contrib-dev
sudo apt-get install libeigen3-dev libglew-dev libboost-all-dev

# Pangolin (visualization)
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# ROS Noetic (optional, for ROS integration)
# Follow: http://wiki.ros.org/noetic/Installation/Ubuntu
```

### Python Dependencies

```bash
pip install numpy opencv-python matplotlib evo --break-system-packages
```

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR_USERNAME/ORB_SLAM3_AQUALOC.git
cd ORB_SLAM3_AQUALOC
```

### 2. Download AQUALOC Dataset

Download the Harbor sequences from the [AQUALOC dataset](http://www.music-project.it/aqualoc-dataset/) and place them in `Datasets/AQUALOC/Harbor/`.

### 3. Build ORB-SLAM3

```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

### 4. Extract ORB Vocabulary

```bash
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
```

## Usage

### Running SLAM on Raw Images

```bash
cd Scripts
./run_slam_raw.sh
```

Or manually:

```bash
./ORB_SLAM3/Examples/Monocular/mono_aqualoc \
    ./ORB_SLAM3/Vocabulary/ORBvoc.txt \
    ./Configs/harbor_fisheye.yaml \
    ./Datasets/AQUALOC/Harbor/sequence_01
```

### Preprocessing with CLAHE

```bash
python3 Scripts/preprocess_clahe.py \
    --input Datasets/AQUALOC/Harbor/sequence_01/images \
    --output Preprocessed/CLAHE/Harbor/sequence_01 \
    --clip_limit 2.0 \
    --tile_size 8
```

### Running SLAM on Enhanced Images

```bash
./run_slam_clahe.sh
```

### FUnIE-GAN Preprocessing

FUnIE-GAN enhancement is performed via Google Colab due to GPU requirements. See the [FUnIE-GAN repository](https://github.com/xahidbuffon/FUnIE-GAN) for setup instructions.

```python
# In Colab: Load pre-trained PyTorch weights and process images
# Then transfer enhanced images to Preprocessed/FUnIE-GAN/Harbor/sequence_01/
```

## Evaluation

### Trajectory Evaluation

```bash
python3 Scripts/evaluate_trajectory.py \
    --estimated Results/CLAHE/Harbor_seq01/trajectory.txt \
    --ground_truth Datasets/AQUALOC/Harbor/sequence_01/ground_truth.txt
```

### Compare Preprocessing Methods

```bash
python3 Scripts/compare_results.py \
    --raw Results/Raw/Harbor_seq01 \
    --clahe Results/CLAHE/Harbor_seq01 \
    --output Results/comparison_report.txt
```

### Metrics

- **Keyframe Tracking Rate**: Percentage of frames successfully tracked
- **ATE RMSE**: Absolute Trajectory Error (Root Mean Square Error)
- **Initial Map Points**: Number of map points created during initialization

## Configuration

### Camera Parameters (`harbor_fisheye.yaml`)

Key parameters optimized for underwater conditions:

```yaml
# ORB Extractor Parameters (tuned for underwater)
ORBextractor.nFeatures: 4000
ORBextractor.iniThFAST: 10
ORBextractor.minThFAST: 3
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
```

### CLAHE Parameters

```python
clip_limit = 2.0      # Contrast limiting threshold
tile_size = (8, 8)    # Grid size for local histogram equalization
```

## Dataset

This project uses the [AQUALOC Dataset](http://www.music-project.it/aqualoc-dataset/):

- **Environment**: Harbor and Archaeological sites
- **Depth**: Up to 380 meters
- **Resolution**: 640 × 512 pixels (grayscale)
- **Sensors**: Monocular camera, IMU, depth sensor, magnetometer
- **Sequences**: Multiple synchronized sensor streams

## References

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) - Campos et al., IEEE T-RO 2021
- [FUnIE-GAN](https://github.com/xahidbuffon/FUnIE-GAN) - Islam et al., IEEE RA-L 2020
- [AQUALOC Dataset](http://www.music-project.it/aqualoc-dataset/) - Ferrera et al., IROS 2019

## Citation

If you use this work in your research, please cite:

```bibtex
@misc{underwater_slam_enhancement,
  author = {Your Name},
  title = {Underwater Visual SLAM with Image Enhancement},
  year = {2024},
  publisher = {GitHub},
  url = {https://github.com/YOUR_USERNAME/ORB_SLAM3_AQUALOC}
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [ORB-SLAM3 Authors](https://github.com/UZ-SLAMLab/ORB_SLAM3) for the foundational SLAM framework
- [AQUALOC Dataset Authors](http://www.music-project.it/aqualoc-dataset/) for the underwater benchmark
- [FUnIE-GAN Authors](https://github.com/xahidbuffon/FUnIE-GAN) for the underwater image enhancement model
