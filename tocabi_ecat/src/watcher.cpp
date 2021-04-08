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

    printf("\x1b[A\33[2K\r\x1b[A\33[2K\r\x1b[A\33[2K\r cnt %10d lat avg %6.3f max %6.3f min %6.3f dev %6.4f, send avg %6.3f max %6.3f min %6.3f dev %6.4f\n", (int)shm_msgs_->t_cnt,
           shm_msgs_->lat_avg / 1000.0, shm_msgs_->lat_max / 1000.0, shm_msgs_->lat_min / 1000.0, shm_msgs_->lat_dev / 1000.0,
           shm_msgs_->send_avg / 1000.0, shm_msgs_->send_max / 1000.0, shm_msgs_->send_min / 1000.0, shm_msgs_->send_dev / 1000.0);
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos[0], shm_msgs_->pos[1], shm_msgs_->pos[2], shm_msgs_->pos[3], shm_msgs_->pos[4], shm_msgs_->pos[5]);
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos[6], shm_msgs_->pos[7], shm_msgs_->pos[8], shm_msgs_->pos[9], shm_msgs_->pos[10], shm_msgs_->pos[11]);
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f ", shm_msgs_->pos[15], shm_msgs_->pos[16], shm_msgs_->pos[17], shm_msgs_->pos[18], shm_msgs_->pos[19], shm_msgs_->pos[20], shm_msgs_->pos[21], shm_msgs_->pos[22]);
    std::fflush(stdout);

    if (shm_msgs_->t_cnt > 100000000)
    {
      break;
    }
  }

  return 0;
}
