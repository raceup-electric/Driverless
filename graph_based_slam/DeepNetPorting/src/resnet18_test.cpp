#include "utils.hpp"

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

  std::cout << "ok\n";

  /**/
  // Loading image example of input
  std::string image_path = "path/to/img.png";
  int size_resnet = 244;
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  cv::resize(img, img, cv::Size(size_resnet, size_resnet), cv::INTER_LINEAR);
  cv::imshow("Display window", img);
  cv::waitKey();
  
  at::Tensor tens_img = toTensor(img);
  
  tens_img = tens_img.clamp_max(c10::Scalar(50));
  cv::Mat cv_img = toCvImage(tens_img);

  cv::imshow("Effects of ClampMax", cv_img);
  cv::waitKey();

  // Conversion to float for deepnet
  tens_img = tens_img.toType(c10::kFloat).div(255);

  // Swapping the axes of the image for the net
  // Example input is (1, 3, 244, 244) but image is (dim1, dim2, channels)
  tens_img = transpose(tens_img, { (2),(0),(1) });

  // Adding batch dimension, it is a single input though
  // ( Usually in training this is not true )
  // Adding it to the dimension in pos 0
  tens_img.unsqueeze_(0);
  tens_img.to(at::kCUDA);

  // Create a vector of inputs.
  std::vector<torch::jit::IValue> inputs;
  
  // Real image converted to tensor
  inputs.push_back(tens_img);

  // Execute the model and turn its output into a tensor.
  at::Tensor output = module.forward(inputs).toTensor();

  std::cout << "Output size : " << output.sizes() << std::endl;
  //std::cout << "output: \n" << output[0] << std::endl;

  // Only showing the first 5 values
  std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';
  /**/
  
}
