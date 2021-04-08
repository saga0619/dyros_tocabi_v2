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

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    printf("\33[2K\33[2K\r  cnt %10d lat avg %6.3f max %6.3f min %6.3f dev %6.4f, send avg %6.3f max %6.3f min %6.3f dev %6.4f\n", (int)shm_msgs_->t_cnt,
           shm_msgs_->lat_avg / 1000.0, shm_msgs_->lat_max / 1000.0, shm_msgs_->lat_min / 1000.0, shm_msgs_->lat_dev / 1000.0,
           shm_msgs_->send_avg / 1000.0, shm_msgs_->send_max / 1000.0, shm_msgs_->send_min / 1000.0, shm_msgs_->send_dev / 1000.0);
    printf("%3.1f %3.1f %3.1f %3.1f %3.1f %3.1f \n", shm_msgs_->pos[JointMap[0]], shm_msgs_->pos[JointMap[1]], shm_msgs_->pos[JointMap[2]], shm_msgs_->pos[JointMap[3]], shm_msgs_->pos[JointMap[4]], shm_msgs_->pos[JointMap[5]]);
    printf("%3.1f %3.1f %3.1f %3.1f %3.1f %3.1f ", shm_msgs_->pos[JointMap[6]], shm_msgs_->pos[JointMap[7]], shm_msgs_->pos[JointMap[8]], shm_msgs_->pos[JointMap[9]], shm_msgs_->pos[JointMap[10]], shm_msgs_->pos[JointMap[11]]);
    std::fflush(stdout);

    if (shm_msgs_->t_cnt > 100000000)
    {
      break;
    }
  }

  return 0;
}
