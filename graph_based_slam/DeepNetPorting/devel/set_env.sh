TENSORRT="/usr/src/tensorrt" 
TORCH_TENSORRT="/.local/lib/python3.8/site-packages/torch_tensorrt/lib"

# Minimal path needed to it to work
LD_LIBRARY_PATH=$TORCH_TENSORRT:$LD_LIBRARY_PATH:$TENSORRT

# For cuda memory optimization
CUDA_MODULE_LOADING=LAZY