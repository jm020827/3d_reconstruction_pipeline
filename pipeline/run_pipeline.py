import os
import signal
from dotenv import load_dotenv


# .env 설정 로드
load_dotenv()

# 1. Isaac Sim 앱 시작 (모든 임포트보다 최상단에 위치)
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# 필수 Isaac Sim 모듈 임포트
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import enable_extension
import carb

from cli_utils import parse_args
from path_utils import path_exists
from scene_utils import load_map, parse_spawn_pos, spawn_robot
from teleop_utils import (
    build_joystick_teleop_cmd,
    launch_background_teleop,
    launch_terminal_teleop,
)

ROS2_BRIDGE_EXTENSIONS = ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge")

# 2. ROS 2 브릿지 확장 기능 활성화 (신규 확장 우선, 구버전 폴백)
def _enable_ros2_bridge_extension():
    for ext_name in ROS2_BRIDGE_EXTENSIONS:
        if enable_extension(ext_name):
            carb.log_info(f"[Pipeline] Enabled ROS2 bridge extension: {ext_name}")
            return ext_name
    carb.log_error("[Pipeline] Failed to enable ROS2 bridge extension.")
    return None


ROS2_BRIDGE_EXTENSION = _enable_ros2_bridge_extension()

def main():
    
    args = parse_args()

    # World 생성
    world = World(stage_units_in_meters=1.0)

    # ★ 중요: 확장 기능 로딩을 완료하기 위해 앱을 한 번 업데이트합니다.
    simulation_app.update()

    # 텔레옵 실행
    teleop_proc = None
    teleop_mode = (args.teleop or "joystick").lower()
    if teleop_mode == "joystick":
        if args.joystick_teleop_cmd:
            teleop_cmd = args.joystick_teleop_cmd
        else:
            teleop_cmd = build_joystick_teleop_cmd(args.joystick_teleop_config)
        teleop_proc = launch_background_teleop(teleop_cmd, "joystick")
    elif teleop_mode == "keyboard":
        launch_terminal_teleop(args.keyboard_teleop_cmd, "keyboard")
        carb.log_info("[Pipeline] Keyboard teleop is handled by external terminal.")
    else:
        carb.log_info("[Pipeline] Teleop disabled.")

    # 맵 로드
    if args.map and path_exists(args.map):
        print(f"[Pipeline] Loading Map: {args.map}")
        load_map(args.map)
    
    # 로봇 소환
    spawn_pos = parse_spawn_pos(os.getenv("ROBOT_SPAWN_POS", "0,0,0"))
    if args.robot and path_exists(args.robot):
        print(f"[Pipeline] Spawning Robot: {args.robot}")
        spawn_robot(args.robot, spawn_pos, world)

    world.reset()
    print(f"[Pipeline] Ready! Teleop mode: {teleop_mode}")

    # 메인 시뮬레이션 루프
    try:
        while simulation_app.is_running():
            world.step(render=True)
    finally:
        if teleop_proc is not None and teleop_proc.poll() is None:
            try:
                os.killpg(teleop_proc.pid, signal.SIGTERM)
            except Exception:
                pass
        simulation_app.close()

if __name__ == "__main__":
    main()
