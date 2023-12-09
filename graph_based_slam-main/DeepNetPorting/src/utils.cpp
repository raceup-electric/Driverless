#include "utils.hpp"

// Function used to convert OpenCV image to Tensor
at::Tensor toTensor(cv::Mat img, bool show_output, bool unsqueeze, int unsqueeze_dim)
{
    // Conversion of image to tensor            data , dimensions, type maybe?
    at::Tensor tensor_image = torch::from_blob(img.data, { img.rows, img.cols, 3 }, at::kByte);

    // Changing size of tensor
    if (unsqueeze) tensor_image.unsqueeze_(unsqueeze_dim);
    
    if (show_output)
        std::cout << tensor_image.slice(2, 0, 1) << std::endl;
    
    std::cout << "Tensor shape: " << tensor_image.sizes() << std::endl;
    return tensor_image;

}

// Function that is used to transpose tensors
at::Tensor transpose(at::Tensor tensor, c10::IntArrayRef dims)
{
    //std::cout << "shape before : " << tensor.sizes() << std::endl;
    at::Tensor tensor_t = tensor.permute(dims);
    //std::cout << "shape after : " << tensor_t.sizes() << std::endl;
    return tensor_t;
}

// Converting tensor back to Opencv Image
cv::Mat toCvImage(at::Tensor tensor)
{
    int width = tensor.sizes()[0];
    int height = tensor.sizes()[1];
    try
    {
        cv::Mat img(cv::Size{ height, width }, CV_8UC3, tensor.data_ptr<uchar>());    
        return img.clone();
    }
    catch (const c10::Error& e)
    {
        std::cout << "Error : " << e.msg() << std::endl;
    }
    return cv::Mat(height, width, CV_8UC3);
}