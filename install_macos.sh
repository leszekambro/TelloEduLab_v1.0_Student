#!/usr/bin/env bash
set -e
PYTHON=${PYTHON:-python}
$PYTHON -m pip install --upgrade pip setuptools wheel
if ! $PYTHON -c 'import pybullet' >/dev/null 2>&1; then
  $PYTHON -m pip install pybullet || CFLAGS="-Dfdopen=fdopen" $PYTHON -m pip install pybullet --no-cache-dir
fi
$PYTHON -m pip install numpy scipy opencv-contrib-python matplotlib scikit-learn gymnasium control transforms3d pillow djitellopy
$PYTHON -m pip install 'git+https://github.com/learnsyslab/gym-pybullet-drones.git' --no-deps
$PYTHON api_check.py
printf '\nCore installation complete. Optional DL: python -m pip install torch torchvision\n'
