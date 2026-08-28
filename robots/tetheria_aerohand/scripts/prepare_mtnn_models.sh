#!/usr/bin/env bash
#
# Prepare MediaPipe hand models for the Moore Threads MTNN NPU backend.
#
# Pipeline:
#   MediaPipe *_full.tflite  --(tf2onnx / tflite2onnx)-->  ONNX
#     palm_detection: NHWC -> NCHW input transpose
#   real-image calibration (div255, NCHW)
#     --(MTC, quant_type int16 + dynamic_fixed_point)-->  .mtnn
#
# int16 quantization is used (not uint8) because it preserves MediaPipe-level
# accuracy: end-to-end 91.0% / 85.5% on the palm/hand calibration sets, an
# exact match with the TFLite baseline. uint8 loses ~7% on the hand stage.
#
# Usage:
#   CALIB_DIR=/path/to/hand_images bash scripts/prepare_mtnn_models.sh
#
# Environment:
#   CALIB_DIR       directory of real hand/palm images for calibration (required)
#   CALIB_COUNT     number of calibration samples (default 30, MTC uses sample [0])
#   MODEL_DIR       override target directory for .mtnn output
#   PYTHON          python interpreter (default: python3)
#
# Prerequisites (pip):
#   tensorflow tf2onnx tflite2onnx onnx onnxruntime opencv-python numpy mediapipe
# plus the Moore Threads MTC toolkit (mtc on PATH).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AERO_HAND_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MODEL_DIR="${MODEL_DIR:-$AERO_HAND_DIR/rynnrcp_robot_aero_hand/models}"
WORK_DIR="${MTNN_WORK_DIR:-/tmp/mtnn_model_prep}"
PYTHON="${PYTHON:-python3}"
CALIB_DIR="${CALIB_DIR:-}"
CALIB_COUNT="${CALIB_COUNT:-30}"

log()  { printf '%s\n' "$*" >&2; }
fail() { log "Error: $*"; exit 1; }

command -v "$PYTHON" >/dev/null 2>&1 || fail "python not found: $PYTHON"
command -v mtc >/dev/null 2>&1 || fail "mtc not found on PATH; install the Moore Threads MTC toolkit first"
[[ -n "$CALIB_DIR" && -d "$CALIB_DIR" ]] || fail "CALIB_DIR must point to a directory of real hand/palm calibration images"

# Locate MediaPipe TFLite models inside the installed mediapipe package.
MP_DIR="$("$PYTHON" -c 'import mediapipe, os; print(os.path.dirname(mediapipe.__file__))')"
PALM_TFLITE="$MP_DIR/modules/palm_detection/palm_detection_full.tflite"
HAND_TFLITE="$MP_DIR/modules/hand_landmark/hand_landmark_full.tflite"
[[ -f "$PALM_TFLITE" ]] || fail "palm tflite not found: $PALM_TFLITE"
[[ -f "$HAND_TFLITE" ]] || fail "hand tflite not found: $HAND_TFLITE"

mkdir -p "$WORK_DIR" "$MODEL_DIR"

# ------------------------------------------------------------- 1. Export ONNX
log "=== Step 1: Export MediaPipe TFLite -> ONNX ==="
"$PYTHON" - "$PALM_TFLITE" "$HAND_TFLITE" "$WORK_DIR" <<'PYEOF'
import sys, os
palm_tflite, hand_tflite, work = sys.argv[1], sys.argv[2], sys.argv[3]

# hand: tflite2onnx emits NCHW directly
import tflite2onnx
tflite2onnx.convert(hand_tflite, os.path.join(work, "hand_landmark_224.onnx"))
print("  hand -> hand_landmark_224.onnx (NCHW)")

# palm: tf2onnx (tflite2onnx fails on the SSD head); emits NHWC
import subprocess
subprocess.run([sys.executable, "-m", "tf2onnx.convert",
                "--tflite", palm_tflite,
                "--output", os.path.join(work, "palm_detection_192.onnx"),
                "--opset", "13"], check=True,
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print("  palm -> palm_detection_192.onnx (NHWC)")
PYEOF

# ------------------------------------------------- 2. Palm NHWC -> NCHW input
log "=== Step 2: Transpose palm ONNX input to NCHW ==="
"$PYTHON" - "$WORK_DIR" <<'PYEOF'
import sys, os
import onnx
from onnx import helper, TensorProto
work = sys.argv[1]
m = onnx.load(os.path.join(work, "palm_detection_192.onnx"))
g = m.graph
old = g.input[0].name
new_in = helper.make_tensor_value_info("input_nchw", TensorProto.FLOAT, [1, 3, 192, 192])
tr = helper.make_node("Transpose", ["input_nchw"], [old], perm=[0, 2, 3, 1])
g.input.remove(g.input[0]); g.input.insert(0, new_in); g.node.insert(0, tr)
onnx.checker.check_model(m)
onnx.save(m, os.path.join(work, "palm_detection_192_nchw.onnx"))
print("  palm_detection_192_nchw.onnx")
PYEOF

# ---------------------------------------------------- 3. Calibration (div255)
log "=== Step 3: Generate real-image calibration tensors (div255, NCHW) ==="
"$PYTHON" - "$CALIB_DIR" "$CALIB_COUNT" "$WORK_DIR" <<'PYEOF'
import sys, os, glob
import cv2, numpy as np
calib_dir, count, work = sys.argv[1], int(sys.argv[2]), sys.argv[3]
imgs = sorted(glob.glob(os.path.join(calib_dir, "*.jpg")) +
              glob.glob(os.path.join(calib_dir, "*.png")))[:count]
if not imgs:
    raise SystemExit(f"no images in {calib_dir}")
# palm letterbox 192 + div255 + NCHW; hand resize 224 + div255 + NCHW
palm_batch, hand_batch = [], []
for p in imgs:
    img = cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]
    S = 192; s = S / max(w, h); nw, nh = int(round(w*s)), int(round(h*s))
    px, py = (S-nw)//2, (S-nh)//2
    c = np.zeros((S, S, 3), np.uint8); c[py:py+nh, px:px+nw] = cv2.resize(img, (nw, nh))
    palm_batch.append((c.transpose(2, 0, 1) / 255.0).astype(np.float32))
    hr = cv2.resize(img, (224, 224))
    hand_batch.append((hr.transpose(2, 0, 1) / 255.0).astype(np.float32))
np.save(os.path.join(work, "palm_calib.npy"), np.stack(palm_batch))
np.save(os.path.join(work, "hand_calib.npy"), np.stack(hand_batch))
print(f"  {len(imgs)} samples -> palm_calib.npy {np.stack(palm_batch).shape}, hand_calib.npy {np.stack(hand_batch).shape}")
PYEOF

# ---------------------------------------------------------- 4. MTC int16 conv
log "=== Step 4: MTC conversion (int16 + dynamic_fixed_point) ==="
# MTC reads a single [1,3,H,W] calibration sample (uses sample [0] of the batch).
"$PYTHON" - "$WORK_DIR" <<'PYEOF'
import sys, numpy as np, os
work = sys.argv[1]
for name in ("palm", "hand"):
    b = np.load(os.path.join(work, f"{name}_calib.npy"))
    np.save(os.path.join(work, f"{name}_calib_single.npy"), b[0:1])
PYEOF

gen_cfg() { # $1=name $2=model $3=calib $4=fp32outputs
  cat > "$WORK_DIR/mtc_$1.yaml" <<EOF
FuncConfig:
  layer_data_analyze: false
  layer_data_compare: false
  mtnnlite_test_enable: false
ModelConfig:
  input_file_list:
    - $3
  force_fp32input_list: true
  force_fp32output_list: $4
ModelPath: $2
Name: mp_$1
PerfConfig:
  perf_collect_enable: true
QuantConfig:
  quant_type: int16
  quantizer: dynamic_fixed_point
TestEnable: true
UseSingleCore: true
EOF
}
gen_cfg palm "$WORK_DIR/palm_detection_192_nchw.onnx" "$WORK_DIR/palm_calib_single.npy" "true#true"
gen_cfg hand "$WORK_DIR/hand_landmark_224.onnx"        "$WORK_DIR/hand_calib_single.npy" "true#true#true#true"

cd "$WORK_DIR"
log "Converting palm..."
mtc --config "$WORK_DIR/mtc_palm.yaml"
log "Converting hand..."
mtc --config "$WORK_DIR/mtc_hand.yaml"

# --------------------------------------------------------------- 5. Collect
log "=== Step 5: Collect .mtnn models -> $MODEL_DIR ==="
MTC_DATA="${MTC_BASE_DATA_DIR:-/opt/m1000-mtc-toolkit/bin/baselib}/data/net_data/onnx"
palm_out="$(find "$MTC_DATA/mp_palm" -name 'mp_palm.mtnn' 2>/dev/null | head -1)"
hand_out="$(find "$MTC_DATA/mp_hand" -name 'mp_hand.mtnn' 2>/dev/null | head -1)"
[[ -n "$palm_out" ]] || fail "palm .mtnn not found under $MTC_DATA/mp_palm"
[[ -n "$hand_out" ]] || fail "hand .mtnn not found under $MTC_DATA/mp_hand"
cp "$palm_out" "$MODEL_DIR/palm_detection.mtnn"
cp "$hand_out" "$MODEL_DIR/hand_landmark.mtnn"

log ""
log "Done. Models written to $MODEL_DIR:"
ls -lh "$MODEL_DIR/palm_detection.mtnn" "$MODEL_DIR/hand_landmark.mtnn"
log ""
log "Reminder: int16 models match MediaPipe accuracy (91.0%/85.5%). Use the"
log "C++ backend 'mediapipe_lite_mtnn_zero' or Python 'mediapipe_lite_mtnn'."
