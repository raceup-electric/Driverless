#pragma once

#include <stdint.h>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>

void bumblebee2Debayer( const cv::Mat_<uint8_t> &img, cv::Mat_<cv::Vec3b> &rgb_img);

class Bumblebee2Driver
{
public:
  Bumblebee2Driver();
  virtual ~Bumblebee2Driver();

  void setGuid ( uint64_t guid ) { guid_ = guid; };
  uint64_t guid() { return guid_; };
  int width() { return width_; }
  int height() { return height_; }

  uint32_t getShutter();
  void getShutterBoundaries( uint32_t &min, uint32_t &max );
  void setShutter( uint32_t val );

  uint32_t getGain();
  void getGainBoundaries( uint32_t &min, uint32_t &max);
  void setGain( uint32_t val );

  void setDataCallback ( void ( *callback ) ( cv::Mat &, cv::Mat &, uint64_t& ) )
  {
    data_callback_ = callback;
  };
  void openDevice();
  void closeDevice();
 
  void startAcquisition();
  void acquireFrame( uint8_t *left_img, uint8_t *right_img, uint64_t &time_stamp );
  
protected:

  virtual uint64_t getTime();

private:

  bool startTransmission();
  void extractStereo(uint8_t *src,  uint8_t *dest_left, uint8_t *dest_right );
  void lookForCameras();

  /* Pimpl idiom */
  class LowLevelDriver;
  std::unique_ptr<LowLevelDriver> ll_driver_;

  uint64_t guid_ = 0;
  const std::string cam_model_string_ = std::string("Bumblebee2");
  int width_ = 1024, height_ = 768;
  uint64_t num_frames_ = 0;

  uint32_t shutter_min_ = 0, shutter_max_ = 0;
  uint32_t gain_min_ = 0, gain_max_ = 0;

  bool is_running_ = false;

  void ( *data_callback_ ) ( cv::Mat &left_img, cv::Mat &right_img, uint64_t &time_stamp );
};
