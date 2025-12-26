#!/bin/bash
BASE_DIR=~/ORB_SLAM3_AQUALOC
SLAM_DIR=$BASE_DIR/ORB_SLAM3
SEQUENCE_DIR=$BASE_DIR/Preprocessed/CLAHE/Harbor/sequence_01
CONFIG=$BASE_DIR/Configs/harbor_fisheye.yaml
RESULTS_DIR=$BASE_DIR/Results/CLAHE/Harbor_seq01
ASSOCIATIONS=$SEQUENCE_DIR/associations.txt

echo "=========================================="
echo "Running ORB-SLAM3 on CLAHE images"
echo "=========================================="

mkdir -p $RESULTS_DIR
cd $SLAM_DIR

./Examples/Monocular/mono_aqualoc \
    Vocabulary/ORBvoc.txt \
    $CONFIG \
    $ASSOCIATIONS

mv -f KeyFrameTrajectory.txt $RESULTS_DIR/ 2>/dev/null
mv -f CameraTrajectory.txt $RESULTS_DIR/ 2>/dev/null

echo "=========================================="
echo "Results saved to: $RESULTS_DIR"
ls -la $RESULTS_DIR/
echo "=========================================="
