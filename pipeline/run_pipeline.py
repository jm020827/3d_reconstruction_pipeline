import os
import signal
from dotenv import load_dotenv


# .env 설정 로드
load_dotenv()

def _expand_env_vars(keys):
    for key in keys:
        val = os.getenv(key)
        if val:
            os.environ[key] = os.path.expandvars(val)

_expand_env_vars(
    [
        "BASE_DIR",
        "PIPELINE_DIR",
        "ROBOTICS_DATASET_DIR",
        "DEFAULT_MAP_PATH",
        "ROBOT_USD_PATH",
        "ROS_DISTRO",
        "ROS_ROOT",
        "ROS_SETUP_SCRIPT",
        "ROS_SETUP_ZSH",
        "ROS_PYTHON_SITE_PACKAGES",
        "ISAAC_SIM_PYTHON",
        "FIXER_SOURCE_PATH",
        "FIXER_DEMO_DIR",
        "FIXER_CONTAINER_WORKDIR",
        "FIXER_CONTAINER_INPUT_DIR",
        "FIXER_MODEL_PATH",
        "JOYSTICK_BY_ID_DIR",
        "JOYSTICK_DEV_GLOB",
    ]
)

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
    detect_joystick_device,
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

def _nucleus_login_from_env() -> bool:
    server = os.getenv("OMNI_SERVER")
    user = os.getenv("OMNI_USER")
    password = os.getenv("OMNI_PASS")

    if not server or not user or not password:
        carb.log_warn("[Pipeline] OMNI_SERVER/OMNI_USER/OMNI_PASS not set. Skipping Nucleus login.")
        return False

    if not server.startswith("omniverse://"):
        server = f"omniverse://{server}"

    try:
        import omni.client
    except Exception as exc:
        carb.log_error(f"[Pipeline] omni.client import failed: {exc}")
        return False

    try:
        omni.client.initialize()
    except Exception:
        pass

    def _is_ok(result):
        try:
            return result == omni.client.Result.OK
        except Exception:
            return bool(result)

    try:
        if hasattr(omni.client, "login"):
            result = omni.client.login(server, user, password)
        elif hasattr(omni.client, "authenticate"):
            result = omni.client.authenticate(server, user, password)
        elif hasattr(omni.client, "set_credential"):
            result = omni.client.set_credential(server, user, password)
        else:
            carb.log_error("[Pipeline] omni.client has no known login/auth API.")
            return False

        if _is_ok(result):
            carb.log_info(f"[Pipeline] Nucleus login success: {server}")
            return True
        carb.log_error(f"[Pipeline] Nucleus login failed: {server} (result={result})")
        return False
    except Exception as exc:
        carb.log_error(f"[Pipeline] Nucleus login exception: {exc}")
        return False

def _persist_env_value(key: str, value: str) -> bool:
    env_path = os.path.join(os.path.dirname(__file__), ".env")
    if not os.path.exists(env_path):
        return False

    with open(env_path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()

    updated = False
    new_lines = []
    for line in lines:
        if line.startswith(f"{key}="):
            if line != f"{key}={value}":
                new_lines.append(f"{key}={value}")
                updated = True
            else:
                new_lines.append(line)
            continue
        new_lines.append(line)

    if not any(line.startswith(f"{key}=") for line in lines):
        new_lines.append(f"{key}={value}")
        updated = True

    if updated:
        with open(env_path, "w", encoding="utf-8") as f:
            f.write("\n".join(new_lines) + "\n")
    return updated

def main():
    
    args = parse_args()

    # World 생성
    world = World(stage_units_in_meters=1.0)

    # ★ 중요: 확장 기능 로딩을 완료하기 위해 앱을 한 번 업데이트합니다.
    simulation_app.update()

    # Nucleus 자동 로그인
    _nucleus_login_from_env()

    # 텔레옵 실행
    teleop_proc = None
    teleop_mode = (args.teleop or "joystick").lower()
    if teleop_mode == "joystick":
        joystick_device = None
        if not args.joystick_teleop_cmd:
            existing_id = os.getenv("XBOX_ID")
            joystick_device, detected_id = detect_joystick_device(existing_id)
            if detected_id and detected_id != existing_id:
                os.environ["XBOX_ID"] = detected_id
                if _persist_env_value("XBOX_ID", detected_id):
                    carb.log_info("[Pipeline] Updated XBOX_ID in .env")
            if joystick_device:
                carb.log_info(f"[Pipeline] Joystick device: {joystick_device}")

        if args.joystick_teleop_cmd:
            teleop_cmd = args.joystick_teleop_cmd
        else:
            teleop_cmd = build_joystick_teleop_cmd(args.joystick_teleop_config, device=joystick_device)
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
