BASE_DIR=/path/to/3d_reconstruction_pipeline
PIPELINE_DIR=${BASE_DIR}/pipeline
HOST_ROOT=${BASE_DIR}
HOST_WORKSPACE=${BASE_DIR}/robotics_dataset
WORKSPACE=/workspace
HOST_MNT=/mnt
ISAAC_ROS_WS=${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev}
ISAAC_ROS_DEV_WS=${WORKSPACE}/isaac_ros-dev
ISAAC_ROS_ASSETS=${ISAAC_ROS_DEV_WS}/isaac_ros_assets
NVBLOX_WS=${WORKSPACE}/nvblox
ROS_DISTRO=jazzy
ROS_SETUP_SCRIPT=/opt/ros/${ROS_DISTRO}/setup.bash
ISAAC_MAPPING_ROS_PY=/opt/ros/${ROS_DISTRO}/lib/isaac_mapping_ros
TELEOP_CFG=${PIPELINE_DIR}/teleop_twist_joy.yaml
TELEOP_FRAMES_DIR=${HOST_WORKSPACE}/teleop
FIXER_SOURCE_PATH=${BASE_DIR}/workspaces/Fixer
FIXER_CONTAINER_WORKDIR=/work
FIXER_CONTAINER_INPUT_DIR=/input
FIXER_MODEL_PATH=${FIXER_CONTAINER_WORKDIR}/models/pretrained/pretrained_fixer.pkl
FIXER_DOCKER_IMAGE=fixer-cosmos-env

PROJECTDIR=maumai                               # custom
DATADIR=maumai_dataset/zed_nurec_compressed     # custom
POSE_TOPIC_NAME=/zed/zed_node/odom              # custom
BASE_LINK_NAME=zed_camera_link                  # custom


--------------------------------------------------------------------------------
# [Step 1: ROS Bag Converter] 
--------------------------------------------------------------------------------

## Initialization (only for the first time)

```
mkdir -p ${ISAAC_ROS_WS}/src
echo 'export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev/}"' >> ~/.bashrc
source ~/.bashrc
```


```
$ sudo apt-get update
$ k="/usr/share/keyrings/nvidia-isaac-ros.gpg"
$ curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor \
    | sudo tee -a $k > /dev/null
$ f="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
$ sudo touch $f
$ s="deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-4 "
# s="deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-4.0 " # for specified version (4.0)
$ s="${s}noble main"
$ grep -qxF "$s" $f || echo "$s" | sudo tee -a $f
$ sudo apt-get update
```

```
$ pip install termcolor --break-system-packages
$ sudo apt-get install isaac-ros-cli
$ sudo systemctl daemon-reload && sudo systemctl restart docker
$ sudo isaac-ros init docker
```

```
$ echo -e "-v ${WORKSPACE}:${WORKSPACE}" > ~/.isaac_ros_dev-dockerargs
$ isaac-ros activate --build-local
```


## ROS2 BAG -> mapping data 
- rosbag file 이 아니라 이미 cuSFM format 이면 변환할 필요없다.
```
$ isaac-ros activate
$ sudo apt-get update
$ sudo apt-get install -y ros-jazzy-isaac-mapping-ros # it takes a lot of time and disk space
```


### 사전 준비
1. pose_topic_name 을 알아둬야 한다. 
2. base_link_name 을 알아둬야 한다.
3. camera 인식이 안되어 변환에 문제가 생기는 경우, camera_topic_config 를 설정하기 위해 camera_config.yaml 을 만들어둬야한다.
    - camera_config.yaml 예시
        ```
        $ cat <<EOF > camera_config.yaml
        stereo_cameras:
        - name: my_camera
            left: /zed/zed_node/left/image_rect_color/compressed
            left_camera_info: /zed/zed_node/left/camera_info
            right: /zed/zed_node/right/image_rect_color/compressed
            right_camera_info: /zed/zed_node/right/camera_info
        EOF
        ```

### ros2 bag -> mapping data (images/)
```
$ ros2 run isaac_mapping_ros rosbag_to_mapping_data \
    --sensor_data_bag_file ${WORKSPACE}/${PROJECTDIR}/${DATADIR}/zed_nurec_compressed_0.db \
    --output_folder_path ${WORKSPACE}/${PROJECTDIR}/images/ \
    --pose_topic_name ${POSE_TOPIC_NAME} \
    --base_link_name ${BASE_LINK_NAME} \
    --camera_topic_config ${WORKSPACE}/${PROJECTDIR}/${DATADIR}/camera_config.yaml
```

### Expected Output
```
images/
├── <camera_name>/          # e.g., front_stereo_camera_left
│   └── xxxx.jpeg
├── frames_meta.json        # Frame metadata (timestamps, poses, etc.)
└── stereo.edex             # Stereo calibration / extrinsics data
```

## Rtabmap DB 변환 
### TODO
- pointcloud 도 뽑아올 수 있어서 더 정확한 값 기대 가능


--------------------------------------------------------------------------------
# [Step 2: Pose Estimation with cuSFM]
--------------------------------------------------------------------------------

```
$ cd ${HOST_ROOT}/workspaces
$ git clone https://github.com/nvidia-isaac/pyCuSFM
$ cd pyCuSFM
```

```
$ ./run_in_docker.sh --build_docker --install
```


```
$ cusfm_cli --input_dir ${WORKSPACE}/${PROJECTDIR}/images/ \
    --cusfm_base_dir ${WORKSPACE}/${PROJECTDIR}/cusfm \
    --min_inter_frame_distance 0.06 \
    --min_inter_frame_rotation_degrees=1.5
```    

### Expected Output
```
cusfm/
├── cuvgl_map/
├── cuvslam_output/
├── keyframes/
├── kpmap/
├── matches/
├── output_poses/
├── pose_graph/
└── sparse/                 # Contains cameras.txt, images.txt, points3D.txt
```

--------------------------------------------------------------------------------
# [Step 3: Depth Estimation with FoundationStereo]
--------------------------------------------------------------------------------

```
$ isaac-ros activate
```

### FoundationStereo 
- Install FoundationStereo ROS Package
```
$ sudo apt-get update
$ sudo apt-get install -y ros-jazzy-isaac-ros-foundationstereo-models-install
```

- Download pretrained model
```
$ source ${ROS_SETUP_SCRIPT}
$ ros2 run isaac_ros_foundationstereo_models_install install_foundationstereo_models.sh --eula
```

```
# High-resolution model
$ ros2 run isaac_mapping_ros run_foundationstereo_trt_offline.py \
    --image_dir ${WORKSPACE}/${PROJECTDIR}/images \
    --output_dir ${WORKSPACE}/${PROJECTDIR}/depth_foundation \
    --frames_meta_file ${WORKSPACE}/${PROJECTDIR}/cusfm/kpmap/keyframes/frames_meta.json \
      --engine_file_path ${ISAAC_ROS_ASSETS}/models/foundationstereo/deployable_foundation_stereo_small_v1.0/foundationstereo_576x960.engine \
    --verbose

# Low-resolution model
$ ros2 run isaac_mapping_ros run_foundationstereo_trt_offline.py \
    --image_dir ${WORKSPACE}/${PROJECTDIR}/images \
    --output_dir ${WORKSPACE}/${PROJECTDIR}/depth_foundation \
    --frames_meta_file ${WORKSPACE}/${PROJECTDIR}/cusfm/kpmap/keyframes/frames_meta.json \
      --engine_file_path ${ISAAC_ROS_ASSETS}/models/foundationstereo/deployable_foundation_stereo_small_v1.0/foundationstereo_320x736.engine \
    --verbose
```


### ESS(Efficient Selective Stereo) 방법.
- FoundationStereo 에 비해 훨씬 빠름.
- 애초에 depth 성능이 3DGRUT 최종 학습 결과물에 영향을 크게 미치지 않는 것 같음
- 그래서 ESS 로 훨씬 빠르게 수행하는게 더 좋을듯.
- 하지만 pose 문제등이 존재할 수 있으므로 완전히 해결되기 전까지는 FoundationStereo 를 사용할 여지도 남아있음. 

```
$ python3 ${ISAAC_MAPPING_ROS_PY}/run_ess_trt_offline.py \
      --image_dir ${WORKSPACE}/${PROJECTDIR}/images \
      --output_dir ${WORKSPACE}/${PROJECTDIR}/depth_ess \
      --frames_meta_file ${WORKSPACE}/${PROJECTDIR}/cusfm/kpmap/keyframes/frames_meta.json \
      --engine_file_path ${ISAAC_ROS_ASSETS}/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx_trt10.13/ess.engine \
      --verbose
```

### Expected Output
```
depth/
└── <camera_name>/          # Depth maps at original resolution
    └── xxxx.png
```

--------------------------------------------------------------------------------
# [Step 4 : Mesh Generation with nvblox]
--------------------------------------------------------------------------------

```
$ mkdir -p ${NVBLOX_WS}/build
$ cd ${NVBLOX_WS}/build
$ cmake ..
$ make -j$(nproc)
```


```
$ mkdir -p /${WORKSPACE}/${DATADIR}/nvblox_mesh_maumai/
$ ./fuse_cusfm \
     --color_image_dir ${WORKSPACE}/${DATADIR}/images/ \
     --depth_image_dir ${WORKSPACE}/${DATADIR}/depth_ess_maumai/ \
     --frames_meta_file ${WORKSPACE}/${DATADIR}/cusfm_maumai/kpmap/keyframes/frames_meta.json \
     --save_2d_occupancy_map_path ${WORKSPACE}/${DATADIR}/nvblox_mesh_maumai/occupancy_map \
     --mapping_type_dynamic \
     --projective_integrator_max_integration_distance_m=2.5 \
     --esdf_slice_min_height=0.09 \
     --esdf_slice_max_height=0.65 \
     --esdf_slice_height=0.3 \
     --mesh_output_path ${WORKSPACE}/${DATADIR}/nvblox_mesh_maumai/nvblox_mesh.ply \
     --nouse_2d_esdf_mode \
     --fit_to_z0
```

--------------------------------------------------------------------------------
# [Step 5: Neural Reconstruction with 3DGRUT] 
--------------------------------------------------------------------------------

- Start with new terminal

```
# 이미 conda 환경이 만들어져있는 경우
$ conda activate 3dg_blackwell
```

```
# pytorch 버전 확인
$ pip list | grep torch 
```

```
# rtx5080 (sm_120 이상 버전만 호환되는 경우) 
# Pytorch Nightly 버전 설치 2.11.x+cu128
$ pip uninstall -y torch torchvision torchaudio
$ pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
```

```
# reinstall modules
$ export TORCH_CUDA_ARCH_LIST="9.0+PTX"
$ export IGNORE_TORCH_VER=1
$ export CUDA_HOME=$CONDA_PREFIX
$ export MAX_JOBS=$(nproc)

$ pip uninstall -y fused-ssim
$ pip install --no-build-isolation --no-cache-dir git+https://github.com/rahul-goel/fused-ssim@1272e21a282342e89537159e4bad508b19b34157
$ pip uninstall -y ppisp
$ pip install --no-build-isolation --no-cache-dir git+https://github.com/nv-tlabs/ppisp@v1.0.1

$ cd ${HOST_ROOT}/workspaces/3dgrut/thirdparty/kaolin
$ rm -rf build/ dist/
$ python setup.py install

$ cd ${HOST_ROOT}/workspaces/3dgrut
$ pip install --no-build-isolation --no-cache-dir -r requirements.txt
```

## Train 3DGRUT
```
$ mkdir -p /${WORKSPACE}/${PROJECTDIR}/3dgrut/
$ export CUDA_LAUNCH_BLOCKING=1
$ python train.py \
     --config-name apps/cusfm_3dgut.yaml \
     path=${HOST_WORKSPACE}/${PROJECTDIR} \
     out_dir=${HOST_WORKSPACE}/${PROJECTDIR}/3dgrut \
     initialization.fused_point_cloud_path=./dummy.ply \ # use dummy if no .ply 
     experiment_name=3dgut_gs \
     export_usdz.enabled=true \
     export_usdz.apply_normalizing_transform=true
```

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

```
ros2 run isaac_mapping_ros rosbag_to_mapping_data \
    --sensor_data_bag_file ${WORKSPACE}/${PROJECTDIR}/dataset/zed_nurec_compressed_adjusted_depth_point_cloud_round_0.db3 \
    --output_folder_path ${WORKSPACE}/${PROJECTDIR}/images/ \
    --pose_topic_name /zed/zed_node/odom \
    --base_link_name zed_camera_link \
    --camera_topic_config ${WORKSPACE}/${PROJECTDIR}/dataset/camera_config.yaml
    
export PROJECTDIR=maumai_3

cusfm_cli --input_dir ${WORKSPACE}/${PROJECTDIR}/images/ \
    --cusfm_base_dir ${WORKSPACE}/${PROJECTDIR}/cusfm \
    --min_inter_frame_distance 0.06 \
    --min_inter_frame_rotation_degrees=1.5
    
    
python3 ${ISAAC_MAPPING_ROS_PY}/run_ess_trt_offline.py \
    --image_dir ${WORKSPACE}/${PROJECTDIR}/images \
    --output_dir ${WORKSPACE}/${PROJECTDIR}/depth_ess \
    --frames_meta_file ${WORKSPACE}/${PROJECTDIR}/cusfm/kpmap/keyframes/frames_meta.json \
    --engine_file_path ${ISAAC_ROS_ASSETS}/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx_trt10.13/ess.engine \
    --verbose
    
./fuse_cusfm \
    --color_image_dir ${WORKSPACE}/${PROJECTDIR}/images/ \
    --depth_image_dir ${WORKSPACE}/${PROJECTDIR}/depth_ess/ \
    --frames_meta_file ${WORKSPACE}/${PROJECTDIR}/cusfm/kpmap/keyframes/frames_meta.json \
    --save_2d_occupancy_map_path ${WORKSPACE}/${PROJECTDIR}/nvblox_mesh/occupancy_map \
    --mapping_type_dynamic \
    --projective_integrator_max_integration_distance_m=2.5 \
    --esdf_slice_min_height=0.09 \
    --esdf_slice_max_height=0.65 \
    --esdf_slice_height=0.3 \
    --mesh_output_path ${WORKSPACE}/${PROJECTDIR}/nvblox_mesh/nvblox_mesh.ply \
    --nouse_2d_esdf_mode \
    --fit_to_z0
```


--------------------------------------------------------------------------------


```
# docker container 에서 생성된 학습 데이터 삭제하기 위해 container 직접 접속하는 명령어
$ docker run --rm -it -v ${HOST_MNT}:${HOST_MNT} --user root docker-pri.maum.ai:443/worv/3dgrut:latest bash
```

```
# teleop 켜기

$ source ${ROS_SETUP_SCRIPT}
$ ros2 run joy joy_node & ros2 run teleop_twist_joy teleop_node --ros-args --params-file ${TELEOP_CFG}

# teleop 하며 segway_E1 의 rgb_left 카메라로 촬영하기 - isaacSim Script Editor

import omni.replicator.core as rep

camera_path = "/World/Segway_E1_ROS2/base_link/sensor_link/rgb_left"
render_product = rep.create.render_product(camera_path, (1920, 1080))

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="${TELEOP_FRAMES_DIR}", 
    rgb=True,
    bounding_box_2d_tight=False
)
writer.attach([render_product])
rep.orchestrator.step()
```

----------------------------------------------------------

```
# Fixer

# 마운트할 input 로컬 경로, output 경로 설정
export FIXER_INPUT_DIR=${HOST_WORKSPACE}/demo/Close_Box_frames
export FIXER_OUT_DIR=${FIXER_OUT_DIR:-${FIXER_INPUT_DIR}/output}

# docker run 으로 실행
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



```
# 경로의 이미지들 영상 변환
ffmpeg -start_number 1439 -i Grap_Can_2.%04d.png -c:v libx264 -pix_fmt yuv420p output.mp4

# 비교 영상 만들기
ffmpeg -start_number 1439 -i ./Grap_Can_2.%04d.png \
       -start_number 1439 -i ./output/Grap_Can_2.%04d.png \
       -filter_complex hstack \
       -c:v libx264 -pix_fmt yuv420p comparison_output.mp4
```
