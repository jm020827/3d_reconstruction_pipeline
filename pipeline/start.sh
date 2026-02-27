#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "${SCRIPT_DIR}/.env" ]; then
    set -a
    source "${SCRIPT_DIR}/.env"
    set +a
fi

ISAAC_SIM_PYTHON="${ISAAC_SIM_PYTHON:-${ISAAC_PYTHON:-}}"
ROS_SETUP_SCRIPT="${ROS_SETUP_SCRIPT:-}"
ROS_SETUP_ZSH="${ROS_SETUP_ZSH:-}"
ROS_PYTHON_SITE_PACKAGES="${ROS_PYTHON_SITE_PACKAGES:-}"

if [ -z "$ISAAC_SIM_PYTHON" ]; then
    echo "[Error] ISAAC_SIM_PYTHON not set. Configure it in pipeline/.env."
    exit 1
fi

if [ -z "$ROS_SETUP_SCRIPT" ]; then
    echo "[Error] ROS_SETUP_SCRIPT not set. Configure it in pipeline/.env."
    exit 1
fi

if [[ "$SHELL" == *"zsh"* ]] && [ -n "$ROS_SETUP_ZSH" ]; then
    echo "[Pipeline] Zsh detected. Sourcing ROS setup: $ROS_SETUP_ZSH"
    source "$ROS_SETUP_ZSH"
else
    echo "[Pipeline] Bash detected. Sourcing ROS setup: $ROS_SETUP_SCRIPT"
    source "$ROS_SETUP_SCRIPT"
fi

if [ -n "$ROS_PYTHON_SITE_PACKAGES" ]; then
    export PYTHONPATH
    PYTHONPATH=$(echo "$PYTHONPATH" | sed -e "s|${ROS_PYTHON_SITE_PACKAGES}||g")
fi

if [ -f "$ISAAC_SIM_PYTHON" ]; then
    "$ISAAC_SIM_PYTHON" "${SCRIPT_DIR}/run_pipeline.py" "$@"
else
    echo "[Error] Isaac Sim launcher not found at: $ISAAC_SIM_PYTHON"
    exit 1
fi

if [ "${RUN_FIXER_TO_VIDEO:-0}" = "1" ]; then
    if [ -x "${SCRIPT_DIR}/fixer_to_video.sh" ]; then
        "${SCRIPT_DIR}/fixer_to_video.sh"
    else
        echo "[Warn] fixer_to_video.sh not found or not executable."
    fi
fi
