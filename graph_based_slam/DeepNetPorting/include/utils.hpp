#pragma once

#include <torch/script.h> // One-stop header

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <memory>

enum Det {
    tl_x = 0,
    tl_y = 1,
    br_x = 2,
    br_y = 3,
    score = 4,
    class_idx = 5
};

struct Detection {
    cv::Rect bbox;
    float score;
    int class_idx;
};

// Function used to convert OpenCV image to Tensor
at::Tensor toTensor(cv::Mat img, bool show_output = false, bool unsqueeze = false, int unsqueeze_dim = 0 );

// Function that is used to transpose tensors
at::Tensor transpose(at::Tensor tensor, c10::IntArrayRef dims = { 0, 3, 1, 2 });

// Converting tensor back to Opencv Image
cv::Mat toCvImage(at::Tensor tensor);

