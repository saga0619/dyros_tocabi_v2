#include <limits.h>
#include <pthread.h>
#include <thread>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <signal.h>
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

std::atomic<bool> prog_shutdown;

void SIGINT_handler(int sig)
{
  std::cout << "shutdown Signal" << std::endl;
  prog_shutdown = true;
}

int main()
{
  int shm_id_;
  SHMmsgs *shm_msgs_;

  init_shm(shm_msg_key, shm_id_, &shm_msgs_);
  prog_shutdown = false;

  signal(SIGINT, SIGINT_handler);

  printf("\n\n\n\n\n\n\n\n");
  while (!prog_shutdown)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    double time = (double)shm_msgs_->t_cnt / 2000.0;

    double sec;

    int hour, min;

    min = (int)(time / 60);

    sec = time - min * 60;

    hour = (int)(min / 60);

    min = min - hour * 60;

    printf("\x1b[A\33[2K\r");
    printf("\x1b[A\33[2K\r");
    printf("\x1b[A\33[2K\r");
    printf("\x1b[A\33[2K\r");
    printf("\x1b[A\33[2K\r");
    // printf("\x1b[A\33[2K\r");
    // printf("\x1b[A\33[2K\r");
    // printf("\x1b[A\33[2K\r");
    // printf("\x1b[A\33[2K\r");

    printf("time : %d h %d m %7.4f %d, %d %d\n", hour, min, sec, shm_msgs_->maxTorque, (int)shm_msgs_->statusCount, (int)shm_msgs_->process_num);

    printf(" cnt1 %10d lat avg %6.3f max %6.3f min %6.3f dev %6.4f, send avg %6.3f max %6.3f min %6.3f dev %6.4f\n", (int)shm_msgs_->t_cnt,
           shm_msgs_->lat_avg / 1000.0, shm_msgs_->lat_max / 1000.0, shm_msgs_->lat_min / 1000.0, shm_msgs_->lat_dev / 1000.0,
           shm_msgs_->send_avg / 1000.0, shm_msgs_->send_max / 1000.0, shm_msgs_->send_min / 1000.0, shm_msgs_->send_dev / 1000.0);

    printf("rcv cur %6.3f avg %6.3f max %6.3f ovf %d\n", shm_msgs_->low_rcv_us/ 1000.0, shm_msgs_->low_rcv_avg/ 1000.0, shm_msgs_->low_rcv_max/ 1000.0, shm_msgs_->low_rcv_ovf);
    printf("mid cur %6.3f avg %6.3f max %6.3f ovf %d\n", shm_msgs_->low_mid_us/ 1000.0, shm_msgs_->low_mid_avg/ 1000.0, shm_msgs_->low_mid_max/ 1000.0, shm_msgs_->low_mid_ovf);
    printf("snd cur %6.3f avg %6.3f max %6.3f ovf %d\n", shm_msgs_->low_snd_us/ 1000.0, shm_msgs_->low_snd_avg/ 1000.0, shm_msgs_->low_snd_max/ 1000.0, shm_msgs_->low_snd_ovf);

    // int i = 0;
    //printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    // printf("%6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos_virtual[3], shm_msgs_->pos_virtual[4], shm_msgs_->pos_virtual[5], shm_msgs_->pos_virtual[6]);

    // printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    // printf("%6.3f %6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    // printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f \n ", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    // printf("%6.3f %6.3f \n", shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    // printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f ", shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++], shm_msgs_->pos[i++]);

    std::fflush(stdout);
  }

  return 0;
}
