/*
* Dyros Tocabi Controller
* (C) 2021 JuneWhee Ahn <june992@snu.ac.kr>
*
*  ** CC Divided Version **
*/

#include "tocabi_controller/tocabi_controller.h"
#include <signal.h>

std::atomic<bool> *prog_shutdown;

void SIGINT_handler(int sig)
{
    cout << "shutdown Signal" << endl;
    *prog_shutdown = true;
}

int main(int argc, char **argv)
{
    signal(SIGINT, SIGINT_handler);

    ros::init(argc, argv, "tocabi_controller", ros::init_options::NoSigintHandler);

    std::cout << "Starting tocabi controller ..." << std::endl;

    DataContainer dc_;
    
    int shm_id_;

    init_shm(shm_msg_key, shm_id_, &dc_.tc_shm_);

    prog_shutdown = &dc_.tc_shm_->shutdown;


    int shm_rd_id_;
    if ((shm_rd_id_ = shmget(shm_rd_key, sizeof(RobotData), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "RobotData shmget failed..." << std::endl;
        dc_.tc_shm_->shutdown = true;
    }
    if ((dc_.rd_ = (RobotData *)shmat(shm_rd_id_, NULL, 0)) == (RobotData *)-1)
    {
        std::cout << "RobotData shmat failed..." << std::endl;
        dc_.tc_shm_->shutdown = true;
    }

    std::cout << "process num : " << (int)dc_.tc_shm_->process_num << std::endl;

    std::cout << "shm initialized" << std::endl;

    dc_.nh.param("/tocabi_controller/sim_mode", dc_.simMode, false);
    dc_.nh.getParam("/tocabi_controller/Kp", dc_.Kps);
    dc_.nh.getParam("/tocabi_controller/Kv", dc_.Kvs);

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

    if (dc_.tc_shm_->shutdown)
    {
        std::cout << cred << "Shared memory was not successfully removed from the previous run. " << std::endl;
        std::cout << "Please Execute shm_reset : rosrun tocabi_controller shm_reset " << std::endl;
        std::cout << "Or you can remove reset shared memory with 'sudo ipcrm -m " << shm_id_ << "'" << creset << std::endl;
    }
    else
    {
        const int thread_number = 3;

        struct sched_param param;
        pthread_attr_t attrs[thread_number];
        pthread_t threads[thread_number];
        param.sched_priority = 80;
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
                CPU_ZERO(&cpusets[i]);
                CPU_SET(5 - i, &cpusets[i]);
                if (pthread_attr_setaffinity_np(&attrs[i], sizeof(cpu_set_t), &cpusets[i]))
                {
                    printf("attr %d setaffinity failed ", i);
                }
                if (pthread_attr_setinheritsched(&attrs[i], PTHREAD_EXPLICIT_SCHED))
                {
                    printf("attr %d setinheritsched failed ", i);
                }
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

        for (int i = 0; i < thread_number; i++)
        {
            pthread_attr_destroy(&attrs[i]);
        }

        cout << "waiting cont..." << endl;
        /* Join the thread and wait until it is done */
        for (int i = 0; i < thread_number; i++)
        {
            pthread_join(threads[i], NULL);
        }

        cout << "waiting cont..." << endl;
    }

    deleteSharedMemory(shm_id_, dc_.tc_shm_);

    if (shmctl(shm_rd_id_, IPC_RMID, NULL) == -1)
    {
        printf("shared memoty failed to remove. \n");
    }
    else
    {
        printf("Shared memory succesfully removed\n");
    }

    std::cout << cgreen << "//////////////////////////" << creset << std::endl;
    std::cout << cgreen << "tocabi controller Shutdown" << creset << std::endl;
    std::cout << cgreen << "//////////////////////////" << creset << std::endl;
    return 0;
}