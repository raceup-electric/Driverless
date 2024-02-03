#include "utils.hpp"
#include "torch_tensorrt/torch_tensorrt.h"

int main(int argc, const char* argv[]) 
{
  if (argc != 2) 
  {
    std::cerr << "usage: example-app <path-to-exported-script-module>\n";
    return -1;
  }


  torch::jit::script::Module module;
  try 
  {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);
  }
  catch (const c10::Error& e) 
  {
    std::cerr << "error loading the model\n";
    return -1;
  }

  // Example input
  module.eval();
  module.to(torch::kCUDA);

  std::string image_path = "/home/slam-emix/Immagini/img.png";
  int size_resnet = 224;
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  cv::resize(img, img, cv::Size(size_resnet, size_resnet), cv::INTER_LINEAR);
  at::Tensor tens_img = toTensor(img);  
  tens_img = tens_img.clamp_max(c10::Scalar(50));
  tens_img = tens_img.to(torch::kFloat32).div(255);
  tens_img = transpose(tens_img, { (2),(0),(1) });
  tens_img.unsqueeze_(0);
  tens_img = tens_img.to(torch::kCUDA);

  // Execute the model and turn its output into a tensor.
  at::Tensor output = module.forward({tens_img}).toTensor();
  
  std::cout << "Output size : " << output.sizes() << std::endl;

  // Only showing the first 5 values
  std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';
  /**/
  
}
