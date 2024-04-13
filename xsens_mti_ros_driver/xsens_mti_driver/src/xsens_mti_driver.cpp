#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termio.h>
#include <unistd.h>
#include <linux/serial.h>

#include "cmtpacket.h"
#include "cmt3.h"
#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"

#include "xsens_mti_driver.h"

#define XSENS_SAMPLE_COUNTER_MAX 65535
#define XSENS_EXIT_ON_ERROR(res,comment) if (res != XRV_OK) \
{ fprintf(stderr, "Error %d occurred in " comment ": %s, exiting!\n",res,xsensResultText(res)); exit(-1); }

using namespace xsens;

void defaultCallback ( double data[9], uint64_t time_stamp )
{
  printf ( "Callback not set!\n" );
}

void defaultTimeoutCallback(){}

void setupLowLatencySerial( char *port_name  )
{
    printf("Setup a low latency serial connection\n");
    struct serial_struct serinfo;
    int fd = open (port_name, O_RDONLY);
    if(fd == -1)
    {
      fprintf(stderr, "Unable to open %s, exiting\n", port_name);
      exit(-1);
    }
    
    if(ioctl (fd, TIOCGSERIAL, &serinfo) == -1)
    {
      fprintf(stderr, "Unable to get flags for %s, exiting\n", port_name);
      exit(-1);
    }
    serinfo.flags |= ASYNC_LOW_LATENCY;
    
    if(ioctl (fd, TIOCSSERIAL, &serinfo) == -1)
    {
      fprintf(stderr, "Setting \"low_latency\" flag: unable to set flags for %s, exiting\n", port_name);
      exit(-1);
    }
    
    close(fd);
    fsync(fd);
    // Ugly workaround
    usleep(1000000);
}

class XSensMTiDriver::LowLevelDriver
{
public:
  LowLevelDriver() {};
  ~LowLevelDriver() {};
  Cmt3 *cmt3_;
  Packet *packet_;
  CmtOutputMode mode;
  CmtOutputSettings settings;
  CmtDeviceId deviceIds[256];
};

XSensMTiDriver::XSensMTiDriver()
{
  ll_driver_ = std::make_unique<LowLevelDriver>();
  data_callback_ = &defaultCallback;
  timeout_callback_ = &defaultTimeoutCallback;
}

XSensMTiDriver::~XSensMTiDriver()
{
  closeDevice();
}

void XSensMTiDriver::startAcquisition()
{
  openDevice();

  double data[9];
  unsigned short s_count;
  //structs to hold data.
  CmtCalData caldata;
  CmtRawData rawdata;

  XsensResultValue res = ll_driver_->cmt3_->gotoMeasurement();
  XSENS_EXIT_ON_ERROR ( res,"gotoMeasurement" );

  is_running_ = true;
  while ( is_running_ )
  {

    XsensResultValue imu_ret = ll_driver_->cmt3_->waitForDataMessage ( ll_driver_->packet_ );

    if ( imu_ret == XRV_OK )
    {

      uint64_t time = getTime();

      s_count = ll_driver_->packet_->getSampleCounter();

      if ( use_freq_filter_ )
        time = xsensFreqFilter ( time, s_count );


      if ( calibrated_data_ )
      {

        caldata = ll_driver_->packet_->getCalData();

        data[0] = double ( caldata.m_acc.m_data[0] );
        data[1] = double ( caldata.m_acc.m_data[1] );
        data[2] = double ( caldata.m_acc.m_data[2] );
        data[3] = double ( caldata.m_gyr.m_data[0] );
        data[4] = double ( caldata.m_gyr.m_data[1] );
        data[5] = double ( caldata.m_gyr.m_data[2] );
        data[6] = double ( caldata.m_mag.m_data[0] );
        data[7] = double ( caldata.m_mag.m_data[1] );
        data[8] = double ( caldata.m_mag.m_data[2] );        

      }
      else
      {
        rawdata = ll_driver_->packet_->getRawData();

        data[0] = double ( rawdata.m_acc.m_data[0] );
        data[1] = double ( rawdata.m_acc.m_data[1] );
        data[2] = double ( rawdata.m_acc.m_data[2] );
        data[3] = double ( rawdata.m_gyr.m_data[0] );
        data[4] = double ( rawdata.m_gyr.m_data[1] );
        data[5] = double ( rawdata.m_gyr.m_data[2] );
        data[6] = double ( rawdata.m_mag.m_data[0] );
        data[7] = double ( rawdata.m_mag.m_data[1] );
        data[8] = double ( rawdata.m_mag.m_data[2] );        
      }

      data_callback_ ( data, time );
    }
    else if( imu_ret == XRV_TIMEOUT || imu_ret == XRV_TIMEOUTNODATA )
    {
      fprintf ( stderr, "WARNING: Measurement timeout!\n");
      timeout_callback_();
    }
    else
    {
      XSENS_EXIT_ON_ERROR(imu_ret, "waitForDataMessage");
    }
  }
}

void XSensMTiDriver::stopAcquisition()
{
  is_running_ = false;
}

uint64_t XSensMTiDriver::getTime()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
                
  struct timeval tv;
  gettimeofday ( &tv, NULL );
  return ( uint64_t ) ts.tv_sec*1000000000 + ( uint64_t ) ts.tv_nsec;
}

void XSensMTiDriver::openDevice()
{
  unsigned long mt_count = 0;

  if ( calibrated_data_ )
    ll_driver_->mode = CMT_OUTPUTMODE_CALIB;
  else
    ll_driver_->mode = CMT_OUTPUTMODE_RAW;

  ll_driver_->settings = CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG;
  ll_driver_->settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
  XsensResultValue res = XRV_OK;

  ll_driver_->cmt3_ = new Cmt3();

  // Perform hardware scan
  mt_count = doHardwareScan();

  if ( mt_count == 0 )
  {
    fprintf ( stderr, "Xsens MTI : No Xsens MTI detected!\n" );
    exit( EXIT_FAILURE );
  }

  // Set device to user input settings
  doSettings();

  // Initialize packet for data
  ll_driver_->packet_ = new Packet ( ( unsigned short ) mt_count, ll_driver_->cmt3_->isXm() );

  initFreqFilter();

}

void XSensMTiDriver::closeDevice()
{
  ll_driver_->cmt3_->closePort();
  delete ll_driver_->packet_;
  delete ll_driver_->cmt3_;
}

int XSensMTiDriver::doHardwareScan()
{
  XsensResultValue res;
  List<CmtPortInfo> portInfo;
  unsigned long portCount = 0;
  int mt_count;

  printf ( "Xsens MTI : Scanning for connected Xsens devices..." );
  xsens::cmtScanPorts ( portInfo );
  portCount = portInfo.length();
  printf ( "done\n" );

  if ( portCount == 0 )
    return 0;


  for ( int i = 0; i < ( int ) portCount; i++ )
  {
    printf ( "Xsens MTI : Using COM port %s at ", portInfo[i].m_portName );

    switch ( portInfo[i].m_baudrate )
    {
    case B9600  :
      printf ( "9k6" );
      break;
    case B19200 :
      printf ( "19k2" );
      break;
    case B38400 :
      printf ( "38k4" );
      break;
    case B57600 :
      printf ( "57k6" );
      break;
    case B115200:
      printf ( "115k2" );
      break;
    case B230400:
      printf ( "230k4" );
      break;
    case B460800:
      printf ( "460k8" );
      break;
    case B921600:
      printf ( "921k6" );
      break;
    default:
      printf ( "0x%lx", portInfo[i].m_baudrate );
    }
    printf ( " baud\n" );
  }

  //open the port which the device is connected to and connect at the device's baudrate.
  for ( int p = 0; p < ( int ) portCount; p++ )
  {
    if( low_latency_serial_ )
      setupLowLatencySerial(portInfo[p].m_portName);

    printf ( "Xsens MTI : Opening port %s...", portInfo[p].m_portName);
    res = ll_driver_->cmt3_->openPort ( portInfo[p].m_portName, portInfo[p].m_baudrate );
    printf ( "done\n" );
    XSENS_EXIT_ON_ERROR ( res,"cmtOpenPort" );
  }

  //get the Mt sensor count.
  printf ( "Xsens MTI : Retrieving MotionTracker count (excluding attached Xbus Master(s))\n" );
  mt_count = ll_driver_->cmt3_->getMtCount();
  printf ( "Xsens MTI : MotionTracker count: %d\n", mt_count );

  // retrieve the device IDs
  printf ( "Xsens MTI : Retrieving MotionTrackers device ID(s)\n" );
  for ( int j = 0; j < mt_count; j++ )
  {
    res = ll_driver_->cmt3_->getDeviceId ( ( unsigned char ) ( j+1 ), ll_driver_->deviceIds[j] );
    XSENS_EXIT_ON_ERROR ( res,"getDeviceId" );
    printf ( "Xsens MTI : Device ID at busId %i: %08lx\n",j+1, ( long ) ll_driver_->deviceIds[j] );
  }

  return mt_count;
}

void XSensMTiDriver::doSettings()
{
  XsensResultValue res;
  unsigned long mt_count = ll_driver_->cmt3_->getMtCount();

  // set sensor to config sate
  res = ll_driver_->cmt3_->gotoConfig();
  XSENS_EXIT_ON_ERROR ( res,"gotoConfig" );

  //cmt3->setTimeoutMeasurement(10);
  //XSENS_EXIT_ON_ERROR(res,"setTimeoutMeasurement");

  printf ( "Xsens MTI : Measurement timeout = %d\n",ll_driver_->cmt3_->getTimeoutMeasurement() );
//
  ;
  // set the device output mode for the device(s)
  printf ( "Xsens MTI : Configuring your mode selection\n" );

  for ( unsigned int i = 0; i < mt_count; i++ )
  {
    res = ll_driver_->cmt3_->restoreFactoryDefaults(ll_driver_->deviceIds[i]);
    XSENS_EXIT_ON_ERROR ( res,"restoreFactoryDefaults" );
    CmtDeviceMode deviceMode ( ll_driver_->mode, ll_driver_->settings, sample_freq_ );
    if ( ( ll_driver_->deviceIds[i] & 0xFFF00000 ) != 0x00500000 )
    {
      // not an MTi-G, remove all GPS related stuff
      deviceMode.m_outputMode &= 0xFF0F;
    }
    res = ll_driver_->cmt3_->setDeviceMode ( deviceMode, true, ll_driver_->deviceIds[i] );
    XSENS_EXIT_ON_ERROR ( res,"setDeviceMode" );
    
    unsigned short sampleFreq = ll_driver_->cmt3_->getSampleFrequency();
    printf ( "Xsens MTI : Sample Freq = %d\n",sampleFreq );
  
  }
}

void XSensMTiDriver::initFreqFilter()
{
  ff_sum_delay_ = 0;
  ff_num_delay_ = 0;
  ff_expected_time_ = 0;
  ff_prec_time_ = 0;
  ff_prec_system_time_ = 0;
  ff_prec_s_count_ = 0;
}

uint64_t XSensMTiDriver::xsensFreqFilter ( uint64_t time, short unsigned int s_count )
{
  uint64_t res;

  if ( !ff_expected_time_ )
  {
    ff_num_delay_ = 0;
    ff_prec_time_ = time;
    ff_prec_system_time_ = time;
    res = time;
    ff_prec_s_count_ = s_count;
    ff_expected_time_ = time;
  }
  else
  {
    if ( s_count != ( ff_prec_s_count_ < XSENS_SAMPLE_COUNTER_MAX?ff_prec_s_count_+1:0 ) )
      fprintf ( stderr, "**************WARNING! Missed IMU packets!!**************" );

    if ( s_count > ff_prec_s_count_ )
      ff_num_delay_ += s_count - ff_prec_s_count_;
    else
      ff_num_delay_ += s_count + XSENS_SAMPLE_COUNTER_MAX - ff_prec_s_count_;

    ff_prec_s_count_ = s_count;

    uint64_t d = time - ff_prec_time_;
    ff_sum_delay_ = ff_sum_delay_ + d;
    uint64_t mean_d = uint64_t ( llround ( double ( ff_sum_delay_ ) /double ( ff_num_delay_ ) ) );
    ff_expected_time_ += mean_d - 1;


    if ( time > ff_expected_time_ /*+ 1500 || time < df_expected_time - 1500*/ )
    {
      //double weight = 1.0/(fabs(double(mean_d/1000) - double(d/1000)));
      res = ff_prec_system_time_ + mean_d;
      ff_expected_time_ = res;
    }
    else
    {
//       printf("RESINCRO PACKET %d : df_expected_time = %d time %d mean_d = %d diff = %d\n",s_count,df_expected_time,time,
//              mean_d,df_expected_time - time);

      ff_expected_time_ = time;
      res = time;
    }

//     if(res != time)
//       printf("PACKET %d CORRECTED  : time = %d corrected_time %d mean_d = %d correction = %d\n",s_count,time,
//          res, mean_d, res - time);

    ff_prec_time_ = time;
    ff_prec_system_time_ = res;
  }

  return res;
}
