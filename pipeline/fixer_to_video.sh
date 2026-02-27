#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "${SCRIPT_DIR}/.env" ]; then
    set -a
    source "${SCRIPT_DIR}/.env"
    set +a
fi

# 1. 경로 설정 (환경변수 기반)
BASE_DIR="${FIXER_DEMO_DIR:-}"
SOURCE_PATH="${FIXER_SOURCE_PATH:-}"
FIXER_CONTAINER_WORKDIR="${FIXER_CONTAINER_WORKDIR:-}"
FIXER_MODEL_PATH="${FIXER_MODEL_PATH:-}"
FIXER_DOCKER_IMAGE="${FIXER_DOCKER_IMAGE:-fixer-cosmos-env}"
FIXER_FPS="${FIXER_FPS:-24}"
FIXER_CONTAINER_INPUT_DIR="${FIXER_CONTAINER_INPUT_DIR:-}"
AUTO_FIXER_SETUP="${AUTO_FIXER_SETUP:-1}"
FIXER_GIT_URL="${FIXER_GIT_URL:-}"
FIXER_GIT_REF="${FIXER_GIT_REF:-}"
FIXER_MODELS_DIR="${FIXER_MODELS_DIR:-}"
FIXER_MODEL_HOST_PATH="${FIXER_MODEL_HOST_PATH:-}"
FIXER_HF_REPO="${FIXER_HF_REPO:-}"
FIXER_DOCKER_CONTEXT="${FIXER_DOCKER_CONTEXT:-}"
FIXER_DOCKERFILE="${FIXER_DOCKERFILE:-}"

if [ -z "$BASE_DIR" ] || [ -z "$SOURCE_PATH" ]; then
    echo "[Error] FIXER_DEMO_DIR or FIXER_SOURCE_PATH is not set."
    exit 1
fi

if [ -z "$FIXER_CONTAINER_WORKDIR" ] || [ -z "$FIXER_MODEL_PATH" ] || [ -z "$FIXER_CONTAINER_INPUT_DIR" ]; then
    echo "[Error] FIXER_CONTAINER_WORKDIR, FIXER_MODEL_PATH, or FIXER_CONTAINER_INPUT_DIR is not set."
    exit 1
fi

if [ -z "$FIXER_MODELS_DIR" ]; then
    FIXER_MODELS_DIR="${SOURCE_PATH}/models"
fi

if [ -z "$FIXER_MODEL_HOST_PATH" ]; then
    FIXER_MODEL_HOST_PATH="${FIXER_MODELS_DIR}/pretrained/pretrained_fixer.pkl"
fi

if [ -z "$FIXER_DOCKER_CONTEXT" ]; then
    FIXER_DOCKER_CONTEXT="${SOURCE_PATH}"
fi

if [ -z "$FIXER_DOCKERFILE" ]; then
    FIXER_DOCKERFILE="Dockerfile.cosmos"
fi

ensure_fixer_repo() {
    if [ -d "${SOURCE_PATH}/.git" ]; then
        return 0
    fi

    if [ -z "$FIXER_GIT_URL" ]; then
        echo "[Error] FIXER_GIT_URL is not set and repo is missing at ${SOURCE_PATH}."
        exit 1
    fi

    echo "[Setup] Cloning Fixer repo..."
    mkdir -p "$(dirname "${SOURCE_PATH}")"
    git clone "${FIXER_GIT_URL}" "${SOURCE_PATH}"
    if [ -n "$FIXER_GIT_REF" ]; then
        (cd "${SOURCE_PATH}" && git checkout "${FIXER_GIT_REF}")
    fi
}

ensure_fixer_models() {
    if [ -f "${FIXER_MODEL_HOST_PATH}" ]; then
        return 0
    fi

    if [ -z "$FIXER_HF_REPO" ]; then
        echo "[Error] FIXER_HF_REPO is not set and model file is missing."
        exit 1
    fi

    if ! command -v hf >/dev/null 2>&1; then
        echo "[Setup] hf CLI not found. Installing huggingface_hub..."
        if command -v python3 >/dev/null 2>&1; then
            python3 -m pip install -U huggingface_hub
        elif command -v python >/dev/null 2>&1; then
            python -m pip install -U huggingface_hub
        else
            echo "[Error] python not found. Cannot install huggingface_hub."
            exit 1
        fi
    fi

    if ! command -v hf >/dev/null 2>&1; then
        echo "[Error] hf CLI still not found after install. Check PATH."
        exit 1
    fi

    echo "[Setup] Downloading Fixer models from Hugging Face..."
    mkdir -p "${FIXER_MODELS_DIR}"
    hf download "${FIXER_HF_REPO}" --local-dir "${FIXER_MODELS_DIR}"
}

ensure_fixer_docker() {
    if docker image inspect "${FIXER_DOCKER_IMAGE}" >/dev/null 2>&1; then
        return 0
    fi

    local dockerfile_path="${FIXER_DOCKER_CONTEXT}/${FIXER_DOCKERFILE}"
    if [ ! -f "${dockerfile_path}" ]; then
        echo "[Error] Dockerfile not found: ${dockerfile_path}"
        exit 1
    fi

    echo "[Setup] Building Fixer docker image..."
    docker build -t "${FIXER_DOCKER_IMAGE}" -f "${dockerfile_path}" "${FIXER_DOCKER_CONTEXT}"
}

if [ "${AUTO_FIXER_SETUP}" = "1" ]; then
    ensure_fixer_repo
    ensure_fixer_models
    ensure_fixer_docker
fi

echo "=================================================="
echo "🔍 [1단계] 대상 폴더 점검 및 중복 확인"
echo "=================================================="

TARGET_DIRS=()

# 모든 하위 폴더 검색
while read -r dir; do
    # 해당 폴더 내 원본 이미지 존재 확인
    IMG_COUNT=$(find "$dir" -maxdepth 1 -type f \( -iname "*.png" -o -iname "*.jpg" \) | wc -l)
    
    if [ "$IMG_COUNT" -gt 0 ]; then
        DIR_ID=$(echo "${dir#$BASE_DIR/}" | tr '/' '_')
        OUTPUT_DIR="$dir/output"
        
        # [체크 포인트] output 폴더가 있고, 그 안에 png 파일이 있으며, mp4 결과물들이 이미 존재하는지 확인
        if [ -d "$OUTPUT_DIR" ] && \
           [ "$(find "$OUTPUT_DIR" -maxdepth 1 -name "*.png" | wc -l)" -gt 0 ] && \
           [ -f "$OUTPUT_DIR/${DIR_ID}_output.mp4" ] && \
           [ -f "$OUTPUT_DIR/${DIR_ID}_comparison.mp4" ]; then
            echo "⏭️  [스킵] 이미 완료된 작업: $dir"
            continue
        fi

        TARGET_DIRS+=("$dir")
        echo "✅ [대상 확정] ($IMG_COUNT 장): $dir"
    fi
done < <(find "$BASE_DIR" -type d ! -path "*/output*" | sort -V)

if [ ${#TARGET_DIRS[@]} -eq 0 ]; then
    echo "🎉 새로 처리할 폴더가 없습니다. 모든 작업이 이미 완료되었습니다."
    exit 0
fi

echo "--------------------------------------------------"
echo "총 ${#TARGET_DIRS[@]} 개의 폴더를 새로 처리합니다."
sleep 2

echo ""
echo "=================================================="
echo "🚀 [2단계] Fixer 실행 및 PTS 교정 인코딩"
echo "=================================================="

for TARGET_DIR in "${TARGET_DIRS[@]}"; do
    DIR_ID=$(echo "${TARGET_DIR#$BASE_DIR/}" | tr '/' '_')
    OUTPUT_DIR="$TARGET_DIR/output"
    mkdir -p "$OUTPUT_DIR"

    OUT_VIDEO="$OUTPUT_DIR/${DIR_ID}_output.mp4"
    COMP_VIDEO="$OUTPUT_DIR/${DIR_ID}_comparison.mp4"

    echo "📂 진행 중: $TARGET_DIR"

    # 2. Fixer 실행 (도커)
    docker run --rm --gpus=all --ipc=host \
        -u $(id -u):$(id -g) \
        -v "${SOURCE_PATH}:${FIXER_CONTAINER_WORKDIR}" \
        -v "${TARGET_DIR}:${FIXER_CONTAINER_INPUT_DIR}" \
        -v "${OUTPUT_DIR}:${OUTPUT_DIR}" \
        --entrypoint python \
        "${FIXER_DOCKER_IMAGE}" \
        "${FIXER_CONTAINER_WORKDIR}/src/inference_pretrained_model.py" \
        --model "${FIXER_MODEL_PATH}" \
        --input "${FIXER_CONTAINER_INPUT_DIR}" \
        --output "${OUTPUT_DIR}" \
        --timestep 250

    # 3. 리스트 파일 생성
    find "$TARGET_DIR" -maxdepth 1 -type f \( -iname "*.png" -o -iname "*.jpg" \) | sort -V | sed "s/^/file '/;s/$/'/" > before_list.txt
    find "$OUTPUT_DIR" -maxdepth 1 -type f \( -iname "*.png" -o -iname "*.jpg" \) | sort -V | sed "s/^/file '/;s/$/'/" > after_list.txt

    # 4. FFmpeg 인코딩 (타임스탬프 오류 해결 필터 적용)
    if [ -s after_list.txt ]; then
        echo "🎬 인코딩 및 PTS 교정 중..."
        
        # 결과 비디오
        ffmpeg -y -f concat -safe 0 -i after_list.txt \
            -vf "settb=AVTB,setpts=N/${FIXER_FPS}/TB" \
            -r "${FIXER_FPS}" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$OUT_VIDEO"
        
        # 비교 비디오
        ffmpeg -y -f concat -safe 0 -i before_list.txt \
            -f concat -safe 0 -i after_list.txt \
            -filter_complex "[0:v][1:v]hstack,settb=AVTB,setpts=N/${FIXER_FPS}/TB" \
            -r "${FIXER_FPS}" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$COMP_VIDEO"
    fi

    rm -f before_list.txt after_list.txt
    echo "✅ 완료: $DIR_ID"
done

echo "=================================================="
echo "🎉 모든 작업이 끝났습니다."
echo "=================================================="
