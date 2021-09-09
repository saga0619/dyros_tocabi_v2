/*
* Dyros Tocabi Controller
* (C) 2021 JuneWhee Ahn <june992@snu.ac.kr>
*
*  ** CC Divided Version **
*/

#include "tocabi_controller/tocabi_controller.h"
#include <signal.h>


#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>


std::atomic<bool> *prog_shutdown;

void SIGINT_handler(int sig)
{
    cout << "shutdown Signal" << endl;
    *prog_shutdown = true;
}


static int latency_target_fd = -1;
static int32_t latency_target_value = 0;
static void set_latency_target(void)
{
    struct stat s;
    int err;
    int errno;
    errno = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1)
    {
        std::cout<<"WARN: stat /dev/cpu_dma_latency failed"<<std::endl;
        return;
    }

    errno = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1)
    {
        std::cout<<"WARN: open /dev/cpu_dma_latency"<<std::endl;
        return;
    }

    errno = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1)
    {
        std::cout<<"# error setting cpu_dma_latency to %d!"<<latency_target_value<<std::endl;
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}

int main(int argc, char **argv)
{
    set_latency_target();
    signal(SIGINT, SIGINT_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    ros::init(argc, argv, "tocabi_controller", ros::init_options::NoSigintHandler);

    DataContainer dc_;

    bool activateLogger;

    dc_.nh.param("/tocabi_controller/sim_mode", dc_.simMode, false);
    dc_.nh.getParam("/tocabi_controller/Kp", dc_.Kps);
    dc_.nh.getParam("/tocabi_controller/Kv", dc_.Kvs);
    dc_.nh.param("/tocabi_controller/log", activateLogger, false);

    if (dc_.Kps.size() != MODEL_DOF)
    {
        std::cout << "Kps size error ! " << dc_.Kps.size() << std::endl;
    }
    if (dc_.Kvs.size() != MODEL_DOF)
    {
        std::cout << "Kps size error ! " << dc_.Kvs.size() << std::endl;
    }

    StateManager stm(dc_);

    TocabiController tc_(stm);

    int shm_id_;

    init_shm(shm_msg_key, shm_id_, &dc_.tc_shm_);

    prog_shutdown = &dc_.tc_shm_->shutdown;

    std::cout << "process num : " << (int)dc_.tc_shm_->process_num << std::endl;

    std::cout << "shm initialized" << std::endl;

    if (dc_.tc_shm_->shutdown)
    {
        std::cout << cred << "Shared memory was not successfully removed from the previous run. " << std::endl;
        std::cout << "Please Execute shm_reset : rosrun tocabi_controller shm_reset " << std::endl;
        std::cout << "Or you can remove reset shared memory with 'sudo ipcrm -m " << shm_id_ << "'" << creset << std::endl;
    }
    else
    {
        const int thread_number = 3;

        struct sched_param param_st;
        struct sched_param param;
        pthread_attr_t attrs[thread_number];
        pthread_t threads[thread_number];
        param.sched_priority = 80;
        param_st.sched_priority = 95;
        cpu_set_t cpusets[thread_number];

        if (dc_.simMode)
            cout << "Simulation Mode" << endl;

        //set_latency_target();

        /* Initialize pthread attributes (default values) */

        for (int i = 0; i < thread_number; i++)
        {
            if (pthread_attr_init(&attrs[i]))
            {
                printf("attr %d init failed ", i);
            }

            if (!dc_.simMode)
            {
                if (pthread_attr_setschedpolicy(&attrs[i], SCHED_FIFO))
                {
                    printf("attr %d setschedpolicy failed ", i);
                }
                if (pthread_attr_setschedparam(&attrs[i], &param))
                {
                    printf("attr %d setschedparam failed ", i);
                }
                if (pthread_attr_setinheritsched(&attrs[i], PTHREAD_EXPLICIT_SCHED))
                {
                    printf("attr %d setinheritsched failed ", i);
                }
            }
        }

        if (!dc_.simMode)
        {

            if (pthread_attr_setschedparam(&attrs[0], &param_st))
            {
                printf("attr %d setschedparam failed ", 0);
            }

            CPU_ZERO(&cpusets[0]);
            CPU_SET(5, &cpusets[0]);
            if (pthread_attr_setaffinity_np(&attrs[0], sizeof(cpu_set_t), &cpusets[0]))
            {
                printf("attr %d setaffinity failed ", 0);
            }
        }

        if (pthread_create(&threads[0], &attrs[0], &StateManager::ThreadStarter, &stm))
        {
            printf("threads[0] create failed\n");
        }
        if (pthread_create(&threads[1], &attrs[1], &TocabiController::Thread1Starter, &tc_))
        {
            printf("threads[1] create failed\n");
        }
        if (pthread_create(&threads[2], &attrs[2], &TocabiController::Thread2Starter, &tc_))
        {
            printf("threads[2] create failed\n");
        }
        pthread_t loggerThread;
        pthread_attr_t loggerattrs;
        if (true)
        {
            pthread_attr_init(&loggerattrs);

            if (pthread_create(&loggerThread, &loggerattrs, &StateManager::LoggerStarter, &stm))
            {
                printf("threads[0] create failed\n");
            }
        }
        for (int i = 0; i < thread_number; i++)
        {
            pthread_attr_destroy(&attrs[i]);
        }

        cout << "waiting cont..." << endl;
        /* Join the thread and wait until it is done */
        if (true)
        {
            pthread_join(loggerThread, NULL);
        }
        for (int i = 0; i < thread_number; i++)
        {
            pthread_join(threads[i], NULL);
        }

        cout << "waiting cont..." << endl;

        deleteSharedMemory(shm_id_, dc_.tc_shm_);
    }
    std::cout << cgreen << "//////////////////////////" << creset << std::endl;
    std::cout << cgreen << "tocabi controller Shutdown" << creset << std::endl;
    std::cout << cgreen << "//////////////////////////" << creset << std::endl;
    return 0;
}