import os
import shlex
import shutil
import subprocess

import carb


def build_clean_ros_env() -> dict:
    keep_keys = [
        "HOME",
        "USER",
        "LOGNAME",
        "SHELL",
        "TERM",
        "DISPLAY",
        "XAUTHORITY",
        "XDG_RUNTIME_DIR",
        "DBUS_SESSION_BUS_ADDRESS",
        "LANG",
        "LC_ALL",
        "ROS_DOMAIN_ID",
        "ROS_LOCALHOST_ONLY",
        "RMW_IMPLEMENTATION",
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "CYCLONEDDS_URI",
    ]
    env = {k: v for k, v in os.environ.items() if k in keep_keys}
    env["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    return env


def build_joystick_teleop_cmd(config_path: str) -> str:
    cfg = shlex.quote(os.path.abspath(config_path))
    return (
        "ros2 run joy joy_node & "
        "ros2 run teleop_twist_joy teleop_node --ros-args --params-file "
        f"{cfg}"
    )


def launch_terminal_teleop(command: str, label: str) -> bool:
    if not command:
        return False

    shell_cmd = f"source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; {command}"
    env = build_clean_ros_env()
    terminal_candidates = [
        ["gnome-terminal", "--", "bash", "-c", shell_cmd],
        ["x-terminal-emulator", "-e", "bash", "-c", shell_cmd],
        ["xterm", "-e", "bash", "-c", shell_cmd],
    ]

    for candidate in terminal_candidates:
        if shutil.which(candidate[0]):
            subprocess.Popen(candidate, env=env)
            carb.log_info(f"[Pipeline] Launched {label} teleop terminal.")
            return True

    carb.log_warn(f"[Pipeline] No terminal emulator found. Run {label} teleop manually.")
    carb.log_warn(f"[Pipeline] Command: {command}")
    return False


def launch_background_teleop(command: str, label: str):
    if not command:
        return None

    shell_cmd = f"source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; {command}"
    env = build_clean_ros_env()
    try:
        proc = subprocess.Popen(["bash", "-c", shell_cmd], env=env, preexec_fn=os.setsid)
        carb.log_info(f"[Pipeline] Started {label} teleop in background.")
        return proc
    except Exception as exc:
        carb.log_error(f"[Pipeline] Failed to start {label} teleop: {exc}")
        return None
