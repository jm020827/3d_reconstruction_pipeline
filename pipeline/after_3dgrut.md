# 3DGRUT 이후 파이프라인 (USDZ → Isaac Sim → Teleop → Capture → Fixer → Video)

아래 내용은 `pipeline.md` 이후 단계(USDZ 결과물 활용)의 실험/운영 파이프라인을 정리한 문서다.

---

## 0) 사전 준비

```
# 예시 환경 변수
export BASE_DIR=/path/to/3d_reconstruction_pipeline
export PIPELINE_DIR=${BASE_DIR}/pipeline
export ROBOTICS_DATASET_DIR=${BASE_DIR}/robotics_dataset
export TELEOP_FRAMES_DIR=${ROBOTICS_DATASET_DIR}/teleop_frames
export DEFAULT_MAP_PATH=/path/to/3dgrut/export_last.usdz
export ROBOT_USD_PATH=omniverse://<server>/<path>/Segway_E1_ROS2.usdc
export ROBOT_SPAWN_POS="0.0, 0.0, 0.2"
export TELEOP_MODE=joystick
export ROS_SETUP_SCRIPT=/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
export JOYSTICK_BY_ID_DIR=/dev/input/by-id
export JOYSTICK_DEV_GLOB=/dev/input/js*
export FIXER_SOURCE_PATH=${BASE_DIR}/workspaces/Fixer
export FIXER_CONTAINER_WORKDIR=/work
export FIXER_CONTAINER_INPUT_DIR=/input
export FIXER_MODEL_PATH=${FIXER_CONTAINER_WORKDIR}/models/pretrained/pretrained_fixer.pkl
export FIXER_DOCKER_IMAGE=fixer-cosmos-env
export FIXER_GIT_URL=https://github.com/nv-tlabs/Fixer.git
export FIXER_GIT_REF=main
export FIXER_HF_REPO=nvidia/Fixer
export FIXER_MODELS_DIR=${FIXER_SOURCE_PATH}/models
export FIXER_MODEL_HOST_PATH=${FIXER_MODELS_DIR}/pretrained/pretrained_fixer.pkl
export FIXER_DOCKER_CONTEXT=${FIXER_SOURCE_PATH}
export FIXER_DOCKERFILE=Dockerfile.cosmos
export AUTO_FIXER_SETUP=1
```

```
# 파이프라인 실행 (Isaac Sim 런처)
cd ${PIPELINE_DIR}
./start.sh --map "$DEFAULT_MAP_PATH" --robot "$ROBOT_USD_PATH" --teleop "$TELEOP_MODE"
```

설명:
- `--map`은 3DGRUT USDZ 경로.
- `--robot`은 Nucleus(omniverse://) 상의 Segway E1 USD 경로.
- `ROBOT_SPAWN_POS`로 로봇 초기 위치를 정함.

---

## 1) Omniverse(Nucleus) 로그인

`omniverse://` 경로를 사용하려면 Nucleus 로그인이 필요하다.
파이프라인은 `OMNI_SERVER/OMNI_USER/OMNI_PASS`가 있으면 자동 로그인을 시도하고,
실패하면 수동 로그인이 필요하다.

---

## 2) Teleoperation (Joystick)

### 조이스틱 번호 확인
```
ls -l ${JOYSTICK_DEV_GLOB}
ls -l ${JOYSTICK_BY_ID_DIR}/ | grep -i -E "xbox|joy|gamepad"
```

### Teleop 수동 실행 (필요시)
```
source ${ROS_SETUP_SCRIPT}
ros2 run joy joy_node &
ros2 run teleop_twist_joy teleop_node --ros-args --params-file ${PIPELINE_DIR}/teleop_twist_joy.yaml
```

---

## 3) Isaac Sim 카메라(rbg_left) 촬영

Isaac Sim Script Editor에서 실행:

```python
import omni.replicator.core as rep

camera_path = "/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left"
render_product = rep.create.render_product(camera_path, (1920, 1080))

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="${TELEOP_FRAMES_DIR}",
    rgb=True
)
writer.attach([render_product])

for _ in range(1000):
    rep.orchestrator.step()
```

---

## 4) 프레임 → 영상 변환

```
ffmpeg -start_number 0 -i frame.%04d.png -c:v libx264 -pix_fmt yuv420p output.mp4
```

---

## 5) Fixer 적용

### 처음 실행 시 자동 설치/셋업
`AUTO_FIXER_SETUP=1`이면 아래 작업을 자동 수행하고, 이미 되어있으면 스킵한다.

수행 항목:
- Fixer repo가 없으면 `git clone`
- 모델이 없으면 `hf download ${FIXER_HF_REPO} --local-dir ${FIXER_MODELS_DIR}`
- Docker 이미지가 없으면 `docker build -t ${FIXER_DOCKER_IMAGE} -f ${FIXER_DOCKERFILE} ${FIXER_DOCKER_CONTEXT}`
- 여러 폴더를 한번에 추론하려면 상위 경로에서 fixer_to_video.sh 스크립트를 실행하면 된다. 그렇지 않은 경우는 하단 명령어를 추천.

```
export FIXER_INPUT_DIR=${TELEOP_FRAMES_DIR}
export FIXER_OUT_DIR=${FIXER_OUT_DIR:-${TELEOP_FRAMES_DIR}/output}

docker run -it --gpus=all --ipc=host \
  -v ${FIXER_SOURCE_PATH}:${FIXER_CONTAINER_WORKDIR} \
  -v ${FIXER_INPUT_DIR}:${FIXER_CONTAINER_INPUT_DIR} \
  --entrypoint python \
  ${FIXER_DOCKER_IMAGE} \
  ${FIXER_CONTAINER_WORKDIR}/src/inference_pretrained_model.py \
  --model ${FIXER_MODEL_PATH} \
  --input ${FIXER_CONTAINER_INPUT_DIR} \
  --output ${FIXER_OUT_DIR} \
  --timestep 250
```

---

## 6) Fixer 결과 영상 + 비교 영상

```
ffmpeg -start_number 0 -i frame.%04d.png \
       -c:v libx264 -pix_fmt yuv420p before.mp4

ffmpeg -start_number 0 -i output/frame.%04d.png \
       -c:v libx264 -pix_fmt yuv420p after.mp4

ffmpeg -start_number 0 -i frame.%04d.png \
       -start_number 0 -i output/frame.%04d.png \
       -filter_complex hstack \
       -c:v libx264 -pix_fmt yuv420p comparison.mp4
```


---

## USDZ에 mesh가 없을 때 수동 처리 가이드 (권장 절차)

### 케이스 1: `nvblox_mesh.ply`가 있는 경우
1. Isaac Sim에서 **USDZ(gaussian) 로드**
2. PLY에 Create ->`Xform`   
3. `nvblox_mesh.ply`를 Stage에 Import 후 생성한 Xform 아래에 넣고 **gaussian과 일치하도록 회전/이동 정렬**
   (그 PLY Mesh Prim 의 Property 에 나와있는 경로가 빨간 글씨로 떠있다면 절대경로로 수정해주면 된다.) 
4. PLY Mesh Prim에 **Add → Physics → Collider** 추가
5. 그 상태로 **맵 Export 또는 Save**
6. 이후 파이프라인을 그 맵에서 계속 진행

### 케이스 2: `nvblox_mesh.ply`가 없는 경우
1. 바닥용 `Plane` Mesh 생성 후 **gaussian과 높이 / 각도 맞춤**
2. 주요 물체는 수동으로 추가 Mesh 생성 후 위치 정렬
3. 각 Mesh에 **Collider** 동일 적용
4. 맵 Export/Save 후 동일하게 다음 단계 진행
---

## Omniverse 서버 로그인 + E1 자동 소환

`OMNI_SERVER/OMNI_USER/OMNI_PASS`가 설정돼 있으면 자동 로그인을 시도한다.
로그인 성공 시 `ROBOT_USD_PATH`를 통해 E1을 자동 소환하고, 실패 시 수동 로그인 후 재실행하면 된다.
