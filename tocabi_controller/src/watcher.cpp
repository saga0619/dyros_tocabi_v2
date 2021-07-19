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

#include "shm_msgs.h"
#include "tocabi_ecat/ecat_settings.h"

int main()
{
  int shm_id;
  SHMmsgs *shm_msgs_;

  init_shm(shm_msg_key, shm_id, &shm_msgs_);

  printf("\n\n\n\n\n");

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    printf("\x1b[A\33[2K\r\x1b[A\33[2K\r\x1b[A\33[2K\r\x1b[A\33[2K\r");
    printf(" sm cnt %10d sm1 avg %6.3f max %6.3f min %6.3f dev %6.4f, sm2 avg %6.3f max %6.3f min %6.3f dev %6.4f\n", (int)shm_msgs_->t_cnt,
           shm_msgs_->lat_avg / 1000.0, shm_msgs_->lat_max / 1000.0, shm_msgs_->lat_min / 1000.0, shm_msgs_->lat_dev / 1000.0,
           shm_msgs_->send_avg / 1000.0, shm_msgs_->send_max / 1000.0, shm_msgs_->send_min / 1000.0, shm_msgs_->send_dev / 1000.0);
    printf(" tc cnt %10d tc1 avg %6.3f max %6.3f min %6.3f dev %6.4f, tc2 avg %6.3f max %6.3f min %6.3f dev %6.4f\n", (int)shm_msgs_->t_cnt2,
           shm_msgs_->lat_avg2 / 1000.0, shm_msgs_->lat_max2 / 1000.0, shm_msgs_->lat_min2 / 1000.0, shm_msgs_->lat_dev2 / 1000.0,
           shm_msgs_->send_avg2 / 1000.0, shm_msgs_->send_max2 / 1000.0, shm_msgs_->send_min2 / 1000.0, shm_msgs_->send_dev2 / 1000.0);
    int i = 0;
    printf("MT : %d :: %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->maxTorque, shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);
    i = 15;
    printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f ", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);
    std::fflush(stdout);

    if (shm_msgs_->t_cnt > 100000000 || shm_msgs_->shutdown)
    {
      break;
    }
  }
  deleteSharedMemory(shm_id, shm_msgs_);

  return 0;
}
