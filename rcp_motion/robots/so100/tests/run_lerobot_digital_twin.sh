#!/bin/bash

cd ..
# launch real mode
echo "Start real mode and run for 60s..."
timeout -s SIGINT 60s python -m scripts.unified_controller --mode real --motion 2 &
REAL_PID=$!

# launch sim mode
echo "Start sim mode and run for 60s..."
timeout -s SIGINT 60s python -m scripts.unified_controller --mode sim --motion 2 &
SIM_PID=$!

echo "Waiting for both modes to finish..."
wait $REAL_PID $SIM_PID

echo "All modes completed."
