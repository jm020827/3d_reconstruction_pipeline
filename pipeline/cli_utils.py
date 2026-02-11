import argparse
import os


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Isaac Sim ROS2 Pipeline")
    parser.add_argument("--map", type=str, default=os.getenv("DEFAULT_MAP_PATH"), help="Path to map")
    parser.add_argument("--robot", type=str, default=os.getenv("ROBOT_USD_PATH"), help="Path to robot")
    parser.add_argument(
        "--teleop",
        type=str,
        choices=["joystick", "keyboard", "none"],
        default=os.getenv("TELEOP_MODE", "joystick"),
        help="Teleop mode (default: joystick)",
    )
    parser.add_argument(
        "--keyboard-teleop-cmd",
        type=str,
        default=os.getenv("KEYBOARD_TELEOP_CMD", "ros2 run teleop_twist_keyboard teleop_twist_keyboard"),
        help="Command to run in a new terminal for keyboard teleop",
    )
    default_joy_config = os.getenv(
        "JOYSTICK_TELEOP_CONFIG",
        os.path.join(os.path.dirname(__file__), "teleop_twist_joy.yaml"),
    )
    parser.add_argument(
        "--joystick-teleop-config",
        type=str,
        default=default_joy_config,
        help="teleop_twist_joy params file for joystick mapping",
    )
    parser.add_argument(
        "--joystick-teleop-cmd",
        type=str,
        default=os.getenv("JOYSTICK_TELEOP_CMD", ""),
        help="Override joystick teleop command (optional)",
    )
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args, _ = parser.parse_known_args(argv)
    return args
