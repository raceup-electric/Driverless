#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/mman.h>
 
#include "rt_utils.h"

void setupRealTime( int32_t priority )
{
  if( priority > 99 ) 
    priority = 99;
  printf("Setting realtime scheduling with priority : %d\n",priority);
  struct sched_param schp; 
  memset(&schp,0,sizeof(schp));
  schp.sched_priority = priority;
  struct rlimit rt_limit = {priority,priority};
  
  
  if(setrlimit(RLIMIT_RTPRIO, &rt_limit) || sched_setscheduler(0, SCHED_FIFO, &schp)) 
  {
    fprintf(stderr, "\n**********************ALERT**********************\n");    
    fprintf(stderr, "Unable to get realtime scheduling priority.\n");
    fprintf(stderr, "This is BAD if you are trying to capture data.\n");
    fprintf(stderr, "To enable realtime scheduling, add the following line:\n");
    fprintf(stderr, "<user_name> hard rtprio 99\n");
    fprintf(stderr, "to the /etc/security/limits.conf file\n");
  }
}
