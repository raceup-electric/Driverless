#include <sys/time.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <poll.h>

#include <libraw1394/raw1394.h>
#include <dc1394/camera.h>
#include <dc1394/conversions.h>
#include <dc1394/control.h>
#include <dc1394/utils.h>

#include "bumblebee2_driver.h"

#define BUMBLEBEE2_DRIVER_FATAL_ERROR( message )\
{\
fprintf(stderr, "BUMBLEBEE2 : %s\n", message);\
exit( EXIT_FAILURE );\
}

// PGR specific register that contains the Bayer Tile mapping information
#define BUMBLEBEE2_BAYER_TILE_MAPPING_REGISTER (0x1040)
#define BUMBLEBEE2_SENSOR_BOARD_INFO_REGISTER (0x1f28)
#define BUMBLEBEE2_IMAGE_DATA_FORMAT_REGISTER (0x1048)
#define BUMBLEBEE2_BIG_ENDIAN (0x80000001)
#define BUMBLEBEE2_LITTLE_ENDIAN (0x80000000)
#define BUMBLEBEE2_COLOR_FILTER_GRBG (0x47524247) // DC1394_COLOR_FILTER_GRBG

void bumblebee2Debayer( const cv::Mat_<uint8_t> &img, cv::Mat_<cv::Vec3b> &rgb_img)
{
  cv::cvtColor(img, rgb_img, cv::COLOR_BayerGR2RGB);
}

void defaultCallback ( cv::Mat &left_img, cv::Mat &right_img, uint64_t &time_stamp )
{
  printf ( "Callback not set!\n" );
}

class Bumblebee2Driver::LowLevelDriver
{
public:
  LowLevelDriver() {};
  ~LowLevelDriver() {};
  dc1394camera_t *camera;
  dc1394_t *context;
};

Bumblebee2Driver::Bumblebee2Driver()
{
  ll_driver_ = std::make_unique<LowLevelDriver>();
  data_callback_ = &defaultCallback;
}

Bumblebee2Driver::~Bumblebee2Driver()
{
  closeDevice();
}

uint64_t Bumblebee2Driver::getTime()
{
  return 0;
}

void Bumblebee2Driver::startAcquisition()
{
  if ( !startTransmission())
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "startTransmission() failed" );
  
  dc1394video_frame_t *frame;
  
  is_running_ = true;
  while(is_running_) 
  {

    dc1394error_t res = dc1394_capture_dequeue(ll_driver_->camera, DC1394_CAPTURE_POLICY_WAIT, &frame);

    uint64_t time = getTime();
    
    // If 0 (default getTimeUs()) use the frame internal timestamp
    if(!time)
      time = frame->timestamp;
      
    if (res != DC1394_SUCCESS) 
    {
      fprintf(stderr, "Capture error %d", res);
      continue;
    }
    cv::Mat left_img = cv::Mat(cv::Size(width_,height_), cv::traits::Type<uchar>::value), 
            right_img = cv::Mat(cv::Size(width_,height_), cv::traits::Type<uchar>::value);
     
    extractStereo ( frame->image, left_img.data, right_img.data );
    
    data_callback_(left_img, right_img, time );
    
    res = dc1394_capture_enqueue(ll_driver_->camera, frame);

    if (res != DC1394_SUCCESS) 
    {
      fprintf(stderr, "Capture error %d", res);
      continue;
    }
    ++num_frames_;
  }
    
}
void Bumblebee2Driver::acquireFrame ( uint8_t *left_img, uint8_t *right_img, uint64_t& time_stamp )
{
  if( !is_running_)
  {
    if ( !startTransmission())
      BUMBLEBEE2_DRIVER_FATAL_ERROR ( "startTransmission() failed" );

    is_running_ = true;
  }

  dc1394video_frame_t *frame;
  dc1394error_t res = dc1394_capture_dequeue(ll_driver_->camera, DC1394_CAPTURE_POLICY_WAIT, &frame);

  time_stamp = getTime();
  
  // If 0 (default getTimeUs()) use the frame internal timestamp
  if(!time_stamp)
    time_stamp = frame->timestamp;

  if (res != DC1394_SUCCESS) 
  {
    fprintf(stderr, "Capture error %d", res);
    return;
  }
    
  extractStereo ( frame->image, left_img, right_img );  

  res = dc1394_capture_enqueue(ll_driver_->camera, frame);
  if (res != DC1394_SUCCESS) 
  {
    fprintf(stderr, "Capture error %d", res);
    return;
  }
  ++num_frames_;    
}

void Bumblebee2Driver::openDevice()
{
  dc1394error_t err;

  ll_driver_->context = dc1394_new();
  if ( guid_ )
    ll_driver_->camera = dc1394_camera_new ( ll_driver_->context, guid_ );
  else
    lookForCameras();

  if ( !ll_driver_->camera )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't find any camera" );

  if ( !cam_model_string_.compare ( ll_driver_->camera->model ) )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "The attached camera is not a Bumblebee2 stereo camera" );

  uint32_t value;
  // This register is an advanced PGR register called SENSOR_BOARD_INFO
  err = dc1394_get_control_register ( ll_driver_->camera, BUMBLEBEE2_SENSOR_BOARD_INFO_REGISTER, &value );
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Could not query the Sensor Info Register" );

  unsigned char sensor_info = 0xf & value;
     
  if ( sensor_info != 0xC )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Only color 1024x768 images are supported" );

  // load the factory defaults - this is auto-everything
  err = dc1394_memory_load ( ll_driver_->camera, 0 );
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't load default memory channel" );

  err = dc1394_set_control_register ( ll_driver_->camera,
                                      BUMBLEBEE2_IMAGE_DATA_FORMAT_REGISTER,
                                      BUMBLEBEE2_LITTLE_ENDIAN );
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't set Bumblebee2 into little-endian mode" );

  dc1394_video_set_iso_speed ( ll_driver_->camera, DC1394_ISO_SPEED_400 );
  dc1394_video_set_mode ( ll_driver_->camera, DC1394_VIDEO_MODE_FORMAT7_3 );

  err = dc1394_format7_set_roi ( ll_driver_->camera,
                                 DC1394_VIDEO_MODE_FORMAT7_3,
                                 DC1394_COLOR_CODING_RAW16,
                                 // bytes per packet - sets frame rate
                                 DC1394_USE_MAX_AVAIL,
                                 0, 0, width_, height_ );
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't setup capture" );


  err = dc1394_feature_set_mode(ll_driver_->camera,DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't set manual mode for shutter" );

  err = dc1394_feature_get_boundaries(ll_driver_->camera,DC1394_FEATURE_SHUTTER,&shutter_min_, &shutter_max_);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't get the shutter boundaries" );

  err = dc1394_feature_set_mode(ll_driver_->camera,DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't set manual mode for gain" );

  err = dc1394_feature_get_boundaries(ll_driver_->camera,DC1394_FEATURE_GAIN,&gain_min_, &gain_max_);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't get the gain boundaries" );

  err = dc1394_feature_set_mode(ll_driver_->camera,DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_AUTO);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't set auto mode for white balance" );

  err = dc1394_capture_setup( ll_driver_->camera, 8, DC1394_CAPTURE_FLAGS_DEFAULT );
  if ( err != DC1394_SUCCESS ) 
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't setup capture" );
         
  err = dc1394_get_control_register ( ll_driver_->camera, BUMBLEBEE2_BAYER_TILE_MAPPING_REGISTER, &value );

  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Could not query the Bayer Tile Register" );

  if ( value != BUMBLEBEE2_COLOR_FILTER_GRBG )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Only supported GRBG bayer tile mapping" );
}

void Bumblebee2Driver::closeDevice()
{
  if ( ll_driver_->camera )
  {
    dc1394_capture_stop ( ll_driver_->camera );
    dc1394_video_set_transmission ( ll_driver_->camera, DC1394_OFF );
    dc1394_camera_free ( ll_driver_->camera );
  }

  if ( ll_driver_->context )
    dc1394_free ( ll_driver_->context );

  ll_driver_->camera = NULL;
  ll_driver_->context = NULL;
  num_frames_ = 0;
  is_running_ = false;
}

void Bumblebee2Driver::lookForCameras()
{
  dc1394camera_list_t * list;
  dc1394error_t err;
  
  err = dc1394_camera_enumerate ( ll_driver_->context, &list );
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Unable to look for cameras" );

  if ( list->num == 0 )
  {
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "No cameras found" );
    exit ( EXIT_FAILURE );
  }

  for ( unsigned int i = 0; i < list->num; i++ )
  {
    ll_driver_->camera = dc1394_camera_new ( ll_driver_->context, list->ids[i].guid );
    if ( !ll_driver_->camera )
    {
      ll_driver_->camera = NULL;
      continue;
    }
    
    if( !strncmp( ll_driver_->camera->model, cam_model_string_.data(), 
      strlen(cam_model_string_.data())) )
    {
      guid_ = list->ids[i].guid;
      break;
    }

    dc1394_camera_free ( ll_driver_->camera );
    ll_driver_->camera = NULL;
  }
  dc1394_camera_free_list ( list );
}

bool Bumblebee2Driver::startTransmission()
{
  dc1394error_t err;
  // have the camera start sending us data
  err = dc1394_video_set_transmission ( ll_driver_->camera, DC1394_ON );
  if ( err != DC1394_SUCCESS )
  {
    fprintf ( stderr, "Unable to start camera iso transmission\n" );
    return err;
  }

  //  Sleep untill the camera has a transmission
  dc1394switch_t status = DC1394_OFF;

  for ( int i = 0; i <= 5; i++ )
  {
    usleep ( 50000 );
    err = dc1394_video_get_transmission ( ll_driver_->camera, &status );
    if ( err != DC1394_SUCCESS )
    {
      fprintf ( stderr, "Unable to get transmision status\n" );
      return false;
    }
    if ( status != false )
      break;

    if ( i == 5 )
    {
      fprintf ( stderr,"Camera doesn't seem to want to turn on!\n" );
      return false;
    }
  }

  dc1394video_frame_t *frame;

  //flush any frames waiting for us
  struct pollfd pollfd;
  pollfd.events = POLLIN;
  pollfd.fd = dc1394_capture_get_fileno(ll_driver_->camera);
  while(poll(&pollfd, 1, 0) > 0)
  {
    dc1394_capture_dequeue(ll_driver_->camera, DC1394_CAPTURE_POLICY_POLL, &frame);
    fprintf(stderr, "camera dumped an old frame\n");
    dc1394_capture_enqueue(ll_driver_->camera, frame);
  }

  return true;
}

uint32_t Bumblebee2Driver::getShutter()
{
  uint32_t val;
  dc1394error_t err = dc1394_feature_get_value(ll_driver_->camera,DC1394_FEATURE_SHUTTER, &val);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't get the current shutter value" );
  return val;
}

void Bumblebee2Driver::getShutterBoundaries( uint32_t &min, uint32_t &max)
{
  min = shutter_min_;
  max = shutter_max_;
}

void Bumblebee2Driver::setShutter( uint32_t val )
{
  dc1394error_t err = dc1394_feature_set_value(ll_driver_->camera,DC1394_FEATURE_SHUTTER, val);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't set the shutter value" );
}

uint32_t Bumblebee2Driver::getGain()
{
  uint32_t val;
  dc1394error_t err = dc1394_feature_get_value(ll_driver_->camera,DC1394_FEATURE_GAIN, &val);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't get the current gain value" );
  return val;
}

void Bumblebee2Driver::getGainBoundaries( uint32_t &min, uint32_t &max)
{
  min = gain_min_;
  max = gain_max_;
}

void Bumblebee2Driver::setGain( uint32_t val )
{
  dc1394error_t err = dc1394_feature_set_value(ll_driver_->camera,DC1394_FEATURE_GAIN, val);
  if ( err != DC1394_SUCCESS )
    BUMBLEBEE2_DRIVER_FATAL_ERROR ( "Can't set the gain value" );
}

void Bumblebee2Driver::extractStereo ( uint8_t* src, uint8_t* dest_left, uint8_t* dest_right )
{
  int dst_size = (width_*height_);
  for( int i = 0, j = 0; j < dst_size; j++ )
  {
    dest_right[j] = src[i++];
    dest_left[j] = src[i++];
  }
}
