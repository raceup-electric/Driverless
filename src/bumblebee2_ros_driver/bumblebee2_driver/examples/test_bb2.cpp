#include <cstdio>
#include <opencv2/opencv.hpp>

#include <boost/program_options.hpp>
#include <iostream>

#include "rt_utils.h"
#include "bumblebee2_driver.h"

using namespace boost;
namespace po = boost::program_options;

bool debayer = false;

struct TrackbarData
{
  uint32_t min;
  Bumblebee2Driver *driver;
};

static void onShutterTrackbar( int val, void* data )
{
   TrackbarData *t_data = static_cast<TrackbarData*>(data);
   uint32_t shutter_val = static_cast<uint32_t>(val) + t_data->min;
   t_data->driver->setShutter(shutter_val);
}

static void onGainTrackbar( int val, void* data )
{
   TrackbarData *t_data = static_cast<TrackbarData*>(data);
   uint32_t gain_val = static_cast<uint32_t>(val) + t_data->min;
   t_data->driver->setGain(gain_val);
}

inline void testCallbackShow( cv::Mat &left_img, cv::Mat &right_img, uint64_t &time_stamp )
{
//   static uint64_t last_time_stamp = 0;
//   if(last_time_stamp) std::cout<<time_stamp - last_time_stamp<<std::endl;
//     last_time_stamp = time_stamp;

  if(debayer)
  {
    cv::Mat_<cv::Vec3b> left_img_rgb(left_img.rows, left_img.cols),
                        right_img_rgb(right_img.rows, right_img.cols);

    bumblebee2Debayer(left_img, left_img_rgb);
    bumblebee2Debayer(right_img, right_img_rgb);

    //printf ( "%ld\n", time_stamp );
    cv::imshow("left", left_img_rgb);
    cv::imshow("right", right_img_rgb);
  }
  else
  {
    cv::imshow("left", left_img);
    cv::imshow("right", right_img);
  }

  cv::waitKey(10);
}

std::string g_dirname;
inline void testCallbackSave( cv::Mat &left_img, cv::Mat &right_img, uint64_t &time_stamp )
{
  static int counter = 0;
  char l_name[256], r_name[256];

  if(debayer)
  {
    cv::Mat_<cv::Vec3b> left_img_rgb(left_img.rows, left_img.cols),
                        right_img_rgb(right_img.rows, right_img.cols);

    bumblebee2Debayer(left_img, left_img_rgb);
    bumblebee2Debayer(right_img, right_img_rgb);

    sprintf(l_name, "%s/%.10dleft.ppm",g_dirname.c_str(), counter);
    sprintf(r_name, "%s/%.10dright.ppm",g_dirname.c_str(), counter++);

    cv::imwrite(l_name, left_img_rgb);
    cv::imwrite(r_name, right_img_rgb);
  }
  else
  {
    sprintf(l_name, "%s/%.10dleft.pgm",g_dirname.c_str(), counter);
    sprintf(r_name, "%s/%.10dright.pgm",g_dirname.c_str(), counter++);

    cv::imwrite(l_name, left_img);
    cv::imwrite(r_name, right_img);
  }
  printf("%s %s\n",l_name, r_name );
}

int main(int argc, char **argv) 
{
  int new_shutter = -1, new_gain = -1;

  try
  {
    po::options_description desc{"Options"};
    desc.add_options()
      ("help,h", "Help screen")
      ("shutter,s", po::value<int>(&new_shutter), "Required camera shutter")
      ("gain,g", po::value<int>(&new_gain), "Required camera gain")
      ("dir,d", po::value<std::string>(&g_dirname), "Directory in which to store images (optional)")
      ("debayer,b", "Converts the input images, arranged with Bayer pattenr,into regular RGB color images");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      std::cout << "Usage: options_description [options]\n";
      std::cout << desc;
      return 0;
    }
    if (vm.count("debayer")) debayer = true;
  }
  catch(std::exception& e)
  {
    std::cout << e.what() << "\n";
    return 1;
  }

  setupRealTime();
  
  Bumblebee2Driver stereo_cam;
  stereo_cam.openDevice();

  if( new_shutter >= 0 )
    stereo_cam.setShutter(static_cast<uint32_t>(new_shutter));

  if( new_gain >= 0 )
    stereo_cam.setGain(static_cast<uint32_t>(new_gain));


  uint32_t shutter, shutter_min, shutter_max, gain, gain_min, gain_max;

  shutter = stereo_cam.getShutter();
  gain = stereo_cam.getGain();
  stereo_cam.getShutterBoundaries(shutter_min,shutter_max);
  stereo_cam.getGainBoundaries(gain_min, gain_max);

  printf("Camera guid : %ld\n",stereo_cam.guid());
  printf("Camera Shutter : (%d, %d, %d)\n", shutter_min, shutter, shutter_max);
  printf("Camera Gain : (%d, %d, %d)\n", gain_min, gain, gain_max);

  if( g_dirname.empty() )
  {
    cv::namedWindow("left");
    cv::namedWindow("right");

    TrackbarData tdata_shutter, tdata_gain;
    tdata_shutter.driver = tdata_gain.driver = &stereo_cam;
    tdata_shutter.min = shutter_min;
    tdata_gain.min = gain_min;

    int shutter_slider = shutter - shutter_min, gain_slider = gain - gain_min;
    cv::createTrackbar( "Shutter", "left", &shutter_slider, shutter_max - shutter_min, onShutterTrackbar, &tdata_shutter);
    cv::createTrackbar( "Gain", "left", &gain_slider, gain_max - gain_min, onGainTrackbar, &tdata_gain );

    stereo_cam.setDataCallback(testCallbackShow);

  }
  else
    stereo_cam.setDataCallback(testCallbackSave);

  stereo_cam.startAcquisition();

  return 0;
}
