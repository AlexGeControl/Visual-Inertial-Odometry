#!/bin/bash
docker run \
  --gpus all \
  --privileged \
  -v ${PWD}/workspace/assignments:/workspace/assignments \
  -v ${PWD}/workspace/data:/workspace/data \
  -p 49001:9001 \
  -p 45901:5900 \
  -p 46006:6006 \
  --name visual-inertial-odometry-gpu shenlanxueyuan/visual-inertial-odometry-gpu:latest