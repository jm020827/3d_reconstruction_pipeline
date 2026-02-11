#!/bin/bash

# 1. Your Isaac Sim Python Path
ISAAC_PYTHON="/home/worv/isaacsim/python.sh"

# 2. current shell check and source ROS 2
if [[ "$SHELL" == *"zsh"* ]]; then
    echo "[Pipeline] Zsh detected. Sourcing setup.zsh"
    source /opt/ros/jazzy/setup.zsh
else
    echo "[Pipeline] Bash detected. Sourcing setup.bash"
    source /opt/ros/jazzy/setup.bash
fi

export PYTHONPATH=$(echo $PYTHONPATH | sed -e 's|/opt/ros/jazzy/lib/python3.12/site-packages||g')

# 3. start pipeline
if [ -f "$ISAAC_PYTHON" ]; then
    # pass args ($@)
    $ISAAC_PYTHON run_pipeline.py "$@"
else
    echo "[Error] Isaac Sim launcher not found at: $ISAAC_PYTHON"
    exit 1
fi
