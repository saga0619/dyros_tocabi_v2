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
#include <ctime>

volatile bool *prog_shutdown;

void SIGINT_handler(int sig)
{
    cout << " CNTRL : shutdown Signal" << endl;
    *prog_shutdown = true;
}

// static int latency_target_fd = -1;
// static int32_t latency_target_value = 0;
// static void set_latency_target(void)
// {
//     struct stat s;
//     int err;
//     int errno;
//     errno = 0;
//     err = stat("/dev/cpu_dma_latency", &s);
//     if (err == -1)
//     {
//         std::cout << "WARN: stat /dev/cpu_dma_latency failed" << std::endl;
//         return;
//     }

//     errno = 0;
//     latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
//     if (latency_target_fd == -1)
//     {
//         std::cout << "WARN: open /dev/cpu_dma_latency" << std::endl;
//         return;
//     }

//     errno = 0;
//     err = write(latency_target_fd, &latency_target_value, 4);
//     if (err < 1)
//     {
//         std::cout << "# error setting cpu_dma_latency to %d!" << latency_target_value << std::endl;
//         close(latency_target_fd);
//         return;
//     }
//     printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
// }

int main(int argc, char **argv)
{
    cout << std::endl
         << "=====================================" << std::endl;
    cout << " CNTRL : Starting TOCABI CONTROLLER! " << endl;
    // set_latency_target();
    signal(SIGINT, SIGINT_handler);

#ifdef COMPILE_REALROBOT
    mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
    ros::init(argc, argv, "tocabi_controller", ros::init_options::NoSigintHandler);

    DataContainer dc_;

    bool activateLogger;
    bool lower_disable;

    dc_.nh.param("/tocabi_controller/sim_mode", dc_.simMode, false);
    dc_.nh.getParam("/tocabi_controller/Kp", dc_.Kps);
    dc_.nh.getParam("/tocabi_controller/Kv", dc_.Kvs);
    dc_.nh.param("/tocabi_controller/log", activateLogger, false);
    dc_.nh.param("/tocabi_controller/disablelower", lower_disable, false);
    dc_.nh.param("/tocabi_controller/avatar_mode", dc_.avatarMode, false);

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

    if (lower_disable)
    {
        std::cout << " CNTRL : LOWER BODY DISABLED BY DEFAULT" << std::endl;
    }
    else
    {
        std::cout << " CNTRL : LOWERBODY ENABLED " << std::endl;
    }

    if (dc_.avatarMode)
    {
        std::cout << " CNTRL : AVATAR MODE ENABLED " << std::endl;
    }

    dc_.tc_shm_->lower_disabled = lower_disable;

    bool zp_load;
    dc_.nh.param("/tocabi_controller/force_load_zp", zp_load, false);

    bool ecat_report;
    dc_.nh.param("/tocabi_controller/ecat_report",ecat_report,false);


    // std::cout << "process num : " << (int)dc_.tc_shm_->process_num << std::endl;
    dc_.tc_shm_->force_load_saved_signal = zp_load;
    dc_.tc_shm_->ecat_report = ecat_report;    

    // std::cout << "shm initialized" << std::endl;

    std::string sysLogFile = "/home/dyros/tocabi_log/system_report";

    ofstream system_log;

    if (dc_.tc_shm_->shutdown)
    {
        std::cout << cred << "Shared memory was not successfully removed from the previous run. " << std::endl;
        std::cout << "Please Execute shm_reset : rosrun tocabi_controller shm_reset " << std::endl;
        std::cout << "Or you can remove reset shared memory with 'sudo ipcrm -m " << shm_id_ << "'" << creset << std::endl;
    }
    else
    {
        const int thread_number = 4;

        struct sched_param param_st;
        struct sched_param param;
        struct sched_param param_controller;
        struct sched_param param_logger;
        pthread_attr_t attrs[thread_number];
        pthread_t threads[thread_number];
        param.sched_priority = 42 + 50;
        param_logger.sched_priority = 41 + 50;
        param_controller.sched_priority = 45 + 50;
        param_st.sched_priority = 45 + 50;
        cpu_set_t cpusets[thread_number];

        if (dc_.simMode)
            cout << "Simulation Mode" << endl;

        // set_latency_target();

        /* Initialize pthread attributes (default values) */

        pthread_t loggerThread;
        pthread_attr_t loggerattrs;

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

        pthread_attr_init(&loggerattrs);
        
        if (!dc_.simMode)
        {

            system_log.open(sysLogFile.c_str(), fstream::out | fstream::app);

            std::time_t t_clock_start = std::time(0);
            system_log << "===================================================================================================" << std::endl;
            system_log << "System Successfully Started " << std::ctime(&t_clock_start);

            if (pthread_attr_setschedparam(&attrs[0], &param_st))
            {
                printf("attr %d setschedparam failed ", 0);
            }
            
            if (pthread_attr_setschedparam(&attrs[1], &param_controller))
            {
                printf("attr %d setschedparam failed ", 0);
            }

            CPU_ZERO(&cpusets[0]);
            CPU_SET(5, &cpusets[0]);
            if (pthread_attr_setaffinity_np(&attrs[0], sizeof(cpu_set_t), &cpusets[0]))
            {
                printf("attr %d setaffinity failed ", 0);
            }

            if (pthread_attr_setschedpolicy(&loggerattrs, SCHED_FIFO))
            {
                printf("attr logger setschedpolicy failed ");
            }
            if (pthread_attr_setschedparam(&loggerattrs, &param_logger))
            {
                printf("attr logger setschedparam failed ");
            }
            if (pthread_attr_setinheritsched(&loggerattrs, PTHREAD_EXPLICIT_SCHED))
            {
                printf("attr logger setinheritsched failed ");
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
        if (pthread_create(&threads[3], &attrs[3], &TocabiController::Thread3Starter, &tc_))
        {
            printf("threads[3] create failed\n");
        }
        if (true)
        {

            if (pthread_create(&loggerThread, &loggerattrs, &StateManager::LoggerStarter, &stm))
            {
                printf("logger Thread create failed\n");
            }
        }
        for (int i = 0; i < thread_number; i++)
        {
            pthread_attr_destroy(&attrs[i]);
        }

        // cout << "waiting cont..." << endl;
        /* Join the thread and wait until it is done */
        if (true)
        {
            pthread_join(loggerThread, NULL);
        }
        for (int i = 0; i < thread_number; i++)
        {
            pthread_join(threads[i], NULL);
        }

        // cout << "waiting cont..." << endl;
    }

    if (system_log.is_open())
    {
        std::time_t t_clock_end = std::time(0);

        int max_us = 30;

        int h_1 = 4;
        int h_2 = 10;

        int setw_up[max_us];
        int setw_low[max_us];

        for (int i = 0; i < max_us; i++)
        {
            int j = 0;
            while (pow(10, j) < dc_.tc_shm_->lat_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->rcv_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->send_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->lat2_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->rcv2_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->send2_h[i])
            {
                j++;
            }

            if (j < 2)
                j = 2;

            setw_up[i] = j;
        }

        system_log << "System Successfully Ended   " << std::ctime(&t_clock_end);
        int hr, mr;
        double sr;

        hr = dc_.rd_.control_time_ / 3600;
        mr = (dc_.rd_.control_time_ - hr * 3600) / 60;

        sr = dc_.rd_.control_time_ - hr * 3600 - mr * 60;

        if (hr > 0)
        {
            system_log << "Running Time : " << hr << " h : " << mr << " m : " << fixed << setprecision(3) << sr << " s" << std::endl;
        }
        else if (mr > 0)
        {
            system_log << "Running Time : " << mr << " m : " << fixed << setprecision(3) << sr << " s" << std::endl;
        }
        else
        {
            system_log << "Running Time : " << fixed << setprecision(3) << dc_.rd_.control_time_ << " s" << std::endl;
        }
        system_log << "ECAT UPPER REPORT | TOTAL COUNT : " << dc_.tc_shm_->statusCount << std::endl;
        system_log << "Latency    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_avg / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_max / 1000.0 << std::endl;
        system_log << "ec_receive avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_avg / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_max / 1000.0 << " ovf : " << dc_.tc_shm_->send_ovf << std::endl;
        system_log << "ec_send    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_avg / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_max / 1000.0 << std::endl;
        system_log << "Histogram : ";
        for (int i = 0; i < 29; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << i << "  ";
        system_log << " +";
        system_log << std::endl;
        system_log << "latency   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->lat_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_recv   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->rcv_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_send   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->send_h[i] << "  ";
        system_log << std::endl;

        system_log << "ECAT LOWER REPORT | TOTAL COUNT : " << dc_.tc_shm_->statusCount2 << std::endl;
        system_log << "Latency    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_avg2 / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_max2 / 1000.0 << std::endl;
        system_log << "ec_receive avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_avg2 / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_max2 / 1000.0 << " ovf : " << dc_.tc_shm_->send_ovf2 << std::endl;
        system_log << "ec_send    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_avg2 / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_max2 / 1000.0 << std::endl;
        system_log << "Histogram : ";
        for (int i = 0; i < 29; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << i << "  ";
        system_log << " +";
        system_log << std::endl;
        system_log << "latency   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->lat2_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_recv   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->rcv2_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_send   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->send2_h[i] << "  ";
        system_log << std::endl
                   << std::endl;

        system_log.close();
    }
    ros::shutdown();
    
    deleteSharedMemory(shm_id_, dc_.tc_shm_);
    std::cout << cgreen << " CNTRL : tocabi controller Shutdown" << creset << std::endl;
    return 0;
}
