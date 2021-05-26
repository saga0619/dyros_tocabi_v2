#include <limits.h>
#include <pthread.h>
#include <thread>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <cstring>
#include <atomic>
#include <chrono>

#include "tocabi_ecat/shm_msgs.h"
#include "tocabi_ecat/ecat_settings.h"

int main()
{
  init_shm();

  printf("\n\n\n\n\n");

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    double time = (double)shm_msgs_->t_cnt/2000.0;
    
    double sec;

    int hour, min;

    min = (int)(time/60);

    sec = time - min*60;

    hour = (int)(min/60);

    min = min - hour*60;

    printf("\x1b[A\33[2K\r");

    printf("\x1b[A\33[2K\r");
    printf("\x1b[A\33[2K\r");

    printf("\x1b[A\33[2K\r\x1b[A\33[2K\r\x1b[A\33[2K\r\x1b[A\33[2K\r\x1b[A\33[2K\r time : %d h %d m %7.4f %d, %d \n", hour,min,sec, shm_msgs_->maxTorque, shm_msgs_->t_cnt - shm_msgs_->t_cnt2);

    printf(" cnt1 %10d lat avg %6.3f max %6.3f min %6.3f dev %6.4f, send avg %6.3f max %6.3f min %6.3f dev %6.4f\n", (int)shm_msgs_->t_cnt,
           shm_msgs_->lat_avg / 1000.0, shm_msgs_->lat_max / 1000.0, shm_msgs_->lat_min / 1000.0, shm_msgs_->lat_dev / 1000.0,
           shm_msgs_->send_avg / 1000.0, shm_msgs_->send_max / 1000.0, shm_msgs_->send_min / 1000.0, shm_msgs_->send_dev / 1000.0);
    printf(" cnt2 %10d lat avg %6.3f max %6.3f min %6.3f dev %6.4f, send avg %6.3f max %6.3f min %6.3f dev %6.4f\n", (int)shm_msgs_->t_cnt2,
           shm_msgs_->lat_avg2 / 1000.0, shm_msgs_->lat_max2 / 1000.0, shm_msgs_->lat_min2 / 1000.0, shm_msgs_->lat_dev2 / 1000.0,
           shm_msgs_->send_avg2 / 1000.0, shm_msgs_->send_max2 / 1000.0, shm_msgs_->send_min2 / 1000.0, shm_msgs_->send_dev2 / 1000.0);
    int i = 0;
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    printf("%6.3f %6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n ", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);
    
    printf("%6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++]);
    
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f ", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);
    
    std::fflush(stdout);


    

    if (shm_msgs_->t_cnt > 100000000 || shm_msgs_->shutdown)
    {
      break;
    }

  }
    deleteSharedMemory();

  return 0;
}
