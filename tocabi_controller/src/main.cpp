//#include "tocabi_controller/state_manager.h"
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

    DataContainer dc_;

    dc_.nh.param("/tocabi_controller/sim_mode", dc_.simMode, false);

    StateManager stm(dc_);

    TocabiController tc_(stm);

    if ((shm_msg_id = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed " << std::endl;
        exit(0);
    }

    if ((dc_.tc_shm_ = (SHMmsgs *)shmat(shm_msg_id, NULL, 0)) == (SHMmsgs *)-1)
    {
        std::cout << "shmat failed " << std::endl;
        exit(0);
    }

    prog_shutdown = &dc_.tc_shm_->shutdown;

    std::cout << "shm initialized" << std::endl;

    //stm.stateThread();

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

    if (pthread_create(&threads[0], &attrs[0], &StateManager::thread_starter, &stm))
    {
        printf("threads[0] create failed\n");
    }
    if (pthread_create(&threads[1], &attrs[1], &TocabiController::thread1_starter, &tc_))
    {
        printf("threads[1] create failed\n");
    }
    if (pthread_create(&threads[2], &attrs[2], &TocabiController::thread2_starter, &tc_))
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

    deleteSharedMemory();

    //Checking if shared memory exist

    std::cout << cgreen << "//////////////////////////" << creset << std::endl;
    std::cout << cgreen << "tocabi controller Shutdown" << creset << std::endl;
    std::cout << cgreen << "//////////////////////////" << creset << std::endl;
    return 0;
}