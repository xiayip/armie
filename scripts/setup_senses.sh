#!/usr/bin/env bash
# Run inside the ROS container as its workspace owner after sourcing install/setup.bash.
set -euo pipefail
armie_workspace="${1:-/workspaces/zephyr-dev/zephyr_ws}"
armie_runtime="${armie_workspace}/.armie-runtime"
python3 -m venv --system-site-packages "${armie_runtime}"
"${armie_runtime}/bin/python" -m pip install --no-deps mediapipe==1.0.1 absl-py==2.5.0 flatbuffers==25.12.19
"${armie_runtime}/bin/python" -m ghost_game_perception.download_model --output "${armie_runtime}/models/gesture_recognizer.task"
MPLCONFIGDIR=/tmp/armie-mpl "${armie_runtime}/bin/python" - "${armie_runtime}/models/gesture_recognizer.task" <<'PY'
import sys
import numpy as np
from ghost_game_perception.gesture_model import GestureModel
with GestureModel(sys.argv[1]) as model:
    assert model.recognize(np.zeros((64,64,3),dtype=np.uint8)) == []
print('Armie gesture runtime ready')
PY
