# DeepNetPorting
Porting your NN trained and validated on Pytorch to your c++ projects

# Dependencies
## C++
- LibTorch v1.13.0+cu117
- TensorRT v8.5.1.7
- Cuda v11.7
- Torch-TensorRT v1.3.0
- MKL-Library

## Python
- torch = 1.12.1 + cu113
- torchvision = 0.13.1 + cu113
- torch_tensorrt = 1.2.0

```
pip3 install torch-tensorrt==1.2.0 --find-links https://github.com/pytorch/TensorRT/releases/expanded_assets/v1.2.0

```

## Step 1
Import your favorite model to c++ using the python code in the scripts folder.

To do this modify the script to choose your preferred model and then execute the command :
```
python3 model_porting.py
```

## Step 2
Modify the bash program in devel to correct the paths that need to be added for correct compilation and executions and then use the command :
```
source set_env.sh
```

Modify the CMakeList.txt to adjust the different paths for correct compilation and execution of the programs.

After that execute :
```
mkdir build && cd build
cmake ..
make -j3
```

## Step 3
The model can be executed directly using both resnet18_test and resnet18rt_test. The function load loads in the same way the torchscript and the TensorRT script, the difference is under which engine is running ( TorchScript engine or TensorRT).


A small example of resnet can be run with the following command :
```
./torch_app "path/to/nonoptmodel.pt"
```
or
```
./torchRT_app "path/to/tensorrt_mod.ts"
```

## Step 4 ( Optional )
The model can be optimized using the third executable that takes in input the parameters traced by the python script and then uses TensorRT optimizations on the model.

It can be done using the following command :
```
./compiletorchRT_app "path/to/nonoptmodel.pt"
```

# Notes
- Do not compile cpp codes with both libtorchtrt_runtime and libtorchtrt, because there could cause some error due to the redifinition of some modules;
- Without executing the set_env.sh the program will complain about missing libraries;