TENSORRT="/usr/src/tensorrt" 
TORCH_TENSORRT="/home/navid/Downloads/libtorchtrt-1.3.0-cudnn8.5-tensorrt8.5-cuda11.7-libtorch1.13.0-x86_64-linux/torch_tensorrt/lib"

# Minimal path needed to it to work
LD_LIBRARY_PATH=$TORCH_TENSORRT:$LD_LIBRARY_PATH:$TENSORRT

# For cuda memory optimization
CUDA_MODULE_LOADING=LAZY