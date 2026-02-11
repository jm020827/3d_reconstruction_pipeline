# 3d_reconstruction_pipeline

### About
This pipeline runs Isaac Sim locally, loads a map and robot USD, and connects optional ROS2 teleop (joystick/keyboard) either from the host or a Docker container.

Docker build is not required. The Isaac Sim pipeline runs locally, and Docker is only needed when you want ROS2 teleop in a container.

### How to use 

**Prerequisites**
- Isaac Sim installed and `python.sh` path confirmed
- `python-dotenv` installed into Isaac Sim Python
- If using local ROS2: ROS2 Jazzy + `ros-jazzy-joy`, `ros-jazzy-teleop-twist-joy`, `ros-jazzy-teleop-twist-keyboard`
- If using Docker: Docker + joystick access via `/dev/input`

**Run Pipeline With Local ROS2**
1. Update `ISAAC_PYTHON` in `pipeline/start.sh` to your Isaac Sim path
2. Run from the `pipeline` directory

```bash
cd pipeline
./start.sh --map <MAP_USD_OR_OMNI> --robot <ROBOT_USD> --teleop joystick
```

```bash
./start.sh --teleop keyboard
```

```bash
./start.sh --teleop none
```

**Run Teleop With Docker (No Local ROS2)**
1. Start the teleop Docker container

```bash
docker compose -f docker/docker-compose.yaml up --build
```

2. Run the pipeline from the host terminal  
   If you do not have local ROS2, comment out the `/opt/ros/jazzy/setup.bash` source line in `pipeline/start.sh` or run `pipeline/run_pipeline.py` directly with Isaac Sim Python

```bash
cd pipeline
TELEOP_MODE=none ./start.sh --map <MAP_USD_OR_OMNI> --robot <ROBOT_USD>
```

**Configuration Summary**
- CLI options: `--map`, `--robot`, `--teleop`
- Environment variables: `DEFAULT_MAP_PATH`, `ROBOT_USD_PATH`, `ROBOT_SPAWN_POS`, `TELEOP_MODE`, `JOYSTICK_TELEOP_CONFIG`, `KEYBOARD_TELEOP_CMD`, `JOYSTICK_TELEOP_CMD`
- Joystick mapping file: `pipeline/teleop_twist_joy.yaml` (Docker uses this file too)

**Notes**
- If `JOYSTICK_TELEOP_CONFIG` is not set, `pipeline/teleop_twist_joy.yaml` is used by default.
