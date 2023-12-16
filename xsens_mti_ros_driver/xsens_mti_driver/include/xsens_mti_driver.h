#pragma once

#include <stdint.h>
#include <memory>
#include <functional>

class XSensMTiDriver
{
public:
  XSensMTiDriver();
  ~XSensMTiDriver();
  
  void setSampleFrequency( int freq ) { sample_freq_ = freq; };
  void enableCalibratedData( bool enable ) { calibrated_data_ = enable; };
  void enableLowLatencySerial( bool enable ) { low_latency_serial_ = enable; };
  void enableFreqFilter( bool enable ) { use_freq_filter_ = enable; };

  void setDataCallback( std::function<void(double[9], uint64_t)> callback ) { data_callback_ = callback; };
  void setTimeoutCallback( std::function<void()> callback  ) { timeout_callback_ = callback; };

  void startAcquisition();
  void stopAcquisition();
  
protected:
  
  virtual uint64_t getTime();
  
private:
  
  void openDevice();
  void closeDevice();

  int doHardwareScan();
  void doSettings();
  void initFreqFilter();
  uint64_t xsensFreqFilter( uint64_t time, short unsigned int s_count );
  
  
  /* Pimpl idiom */
  class LowLevelDriver; 
  std::unique_ptr<LowLevelDriver> ll_driver_;

  int sample_freq_ = 100;
  bool calibrated_data_ = true;
  bool low_latency_serial_ = true;
  bool use_freq_filter_ = false;

  bool is_running_ = false;
  
  uint64_t ff_sum_delay_;
  uint64_t ff_num_delay_;
  uint64_t ff_expected_time_;
  uint64_t ff_prec_time_;
  uint64_t ff_prec_system_time_;
  unsigned short ff_prec_s_count_;
  
  std::function<void(double[9], uint64_t)> data_callback_;
  std::function<void()> timeout_callback_;
};
