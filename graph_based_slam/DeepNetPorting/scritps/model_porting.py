import random
import torch
import torchvision
import torch_tensorrt
import numpy as np



def main(tensort_rt=False) :
    # Load model of choice
    model = torchvision.models.resnet18()
    model2 = torchvision.models.resnet18().eval()

    # Can be done if torch and torch_tensorrt have been built
    # using the same cuDNN version, default false, preferred in cpp
    if ( tensort_rt ) :
        
        scripted_model = torch.jit.script(model2)

        compile_settings = {
        "inputs": [torch_tensorrt.Input([1, 3, 224, 224])],
        "enabled_precisions": {torch.float32}
        }

        trt_ts_module = torch_tensorrt.compile(scripted_model, **compile_settings)
        torch.jit.save(trt_ts_module, "resnet_tensorrt.jit")

    # Generate random input for tracing
    input = torch.rand(1, 3, 224, 224)

    # Use jit.trace to translate the model
    traced_module = torch.jit.trace(model, input)

    # Save the model
    traced_module.save('traced_resnet_model.pt')



if __name__ == "__main__":
    main()
