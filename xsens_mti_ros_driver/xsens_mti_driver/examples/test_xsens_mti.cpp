#include <cstdio>
#include<signal.h>
#include<unistd.h>

#include "rt_utils.h"
#include "xsens_mti_driver.h"

FILE *f = NULL;
XSensMTiDriver xsens;
  
void sig_handler(int signo)
{
  xsens.stopAcquisition();
  if( f != NULL )
  {
    fclose(f);
  }
}

void testCallbackStdOut( double *data, uint64_t time_stamp )
{
  printf("Timestamp : %ld",time_stamp);
  printf(" Accelerometers: [%f %f %f]",data[0], data[1], data[2]); 
  printf(" Gyros : [%f %f %f]",data[3], data[4], data[5]); 
  printf(" Magnetometers : [%f %f %f]",data[6], data[7], data[8]);

  static uint64_t last_time = 0;
  uint64_t delta;

  if ( last_time )
  {
    delta = time_stamp - last_time;
    printf(" delta %u us", (unsigned int)delta);
  }
  last_time = time_stamp;

  printf("\n");
}

void testCallbackFile( double *data, uint64_t time_stamp )
{
  fprintf(f, "%ld ",time_stamp);
  fprintf(f, "%f %f %f ",data[0], data[1], data[2]); 
  fprintf(f, "%f %f %f ",data[3], data[4], data[5]); 
  fprintf(f, "%f %f %f\n",data[6], data[7], data[8]); 
}


int main(int argc, char **argv) 
{
  
  signal(SIGINT, sig_handler);
  
  if( argc > 1 )
  {
    f = fopen(argv[1],"w");
    printf("Using file %s\n",argv[1]);
  }
  
  setupRealTime();
  
  xsens.setSampleFrequency(200);

  if ( f != NULL )
  {
    xsens.setDataCallback(testCallbackFile);
  }
  else
    xsens.setDataCallback(testCallbackStdOut);
  
  xsens.startAcquisition();
  
  return 0;
}
