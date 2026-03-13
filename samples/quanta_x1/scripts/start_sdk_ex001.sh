#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
CLIENT_ROOT=$(dirname "$SCRIPT_DIR")
WORKSPACE_SRC=$(dirname "$CLIENT_ROOT")

export PYTHONPATH=$CLIENT_ROOT:$WORKSPACE_SRC/robocontrol:$PYTHONPATH

# Configuration
MODEL_ADDRESS=${1:-"39.101.65.229"}
MODEL_PORT=${2:-1174}
INSTRUCTION=${3:-""}
CONTROL_MODE=${4:-"end_pose"}
INTERPOLATE_MULTIPLIER=${5:-20}
ROBOT_SDK_URL=${6:-"192.168.10.1:50051"}

echo "Starting EX001 Robot Client..."
echo "Model: $MODEL_ADDRESS:$MODEL_PORT"
echo "Instruction: $INSTRUCTION"
echo "Control Mode: $CONTROL_MODE"

python3 "$SCRIPT_DIR/start_sdk_ex001.py" \
    --model-address "$MODEL_ADDRESS" \
    --port "$MODEL_PORT" \
    --instruction "$INSTRUCTION" \
    --control-mode "$CONTROL_MODE" \
    --interpolate-multiplier "$INTERPOLATE_MULTIPLIER" \
    --robot_sdk_url "$ROBOT_SDK_URL" \
    --debug-step \
    # --use-car-pose-odom