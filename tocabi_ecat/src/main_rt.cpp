#include "tocabi_ecat/tocabi_ecat_rtnet.h"
// #include <cstring>

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;
// static void set_latency_target(void)
// {
//     struct stat s;
//     int err;
//     int errno_;
//     errno_ = 0;
//     err = stat("/dev/cpu_dma_latency", &s);
//     if (err == -1)
//     {
//         std::cout << "WARN: stat /dev/cpu_dma_latency failed" << std::endl;
//         return;
//     }

//     errno_ = 0;
//     latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
//     if (latency_target_fd == -1)
//     {
//         std::cout << "WARN: open /dev/cpu_dma_latency" << std::endl;
//         return;
//     }

//     errno_ = 0;
//     err = write(latency_target_fd, &latency_target_value, 4);
//     if (err < 1)
//     {
//         std::cout << "# error setting cpu_dma_latency to %d!" << latency_target_value << std::endl;
//         close(latency_target_fd);
//         return;
//     }
//     printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
// }

#define stack64k (64 * 1024)
int main(int argc, char *argv[])
{
    // ios::sync_with_stdio(false);
    bool val_received = false;

    int ret = 0;
    int lock_core_ = 0;

    if (argc != 7)
    {
        // std::cout << "usage : tocabi_ecat {port1} {port2} {period_us} {ecat num} {starting num} {Core}" << std::endl;
        printf("usage : tocabi_ecat {port1} {port2} {period_us} {ecat num} {starting num} {Core}\n");
        return 0;
    }
    
    TocabiInitArgs init_args;
    strcpy(init_args.port1,argv[1]);
    strcpy(init_args.port2,argv[2]);
    // soem_port = argv[1];
    int period_us = atoi(argv[3]);
    init_args.period_ns = period_us * 1000;
    expected_counter = atoi(argv[4]);
    init_args.ecat_slave_num = expected_counter;
    start_joint_ = atoi(argv[5]);
    init_args.ecat_slave_start_num = start_joint_;
    lock_core_ = atoi(argv[6]);
    init_args.lock_core = lock_core_;
    period_ns = period_us * 1000;

    // std::cout << " ecat port : " << soem_port << std::endl;
    // std::cout << " period_ns  : " << period_ns << std::endl;
    // std::cout << " elmo num  : " << expected_counter << std::endl;
    // std::cout << " start from : " << start_joint_ << ", : " << ELMO_NAME[start_joint_] << std::endl;
    // std::cout << " locked at core " << lock_core_ << std::endl;
    // std::cout << " ----------------------------- " << std::endl;
    // std::cout << " command :  q(quit), l(lower init), u(upper init), d(debug), p(position), h(homming), c(force control), o(lock), f(torque off), w(status log)" << std::endl;

    val_received = true;

    struct sched_param param;
    pthread_attr_t attr, attr2;
    pthread_t thread1, thread2;
    //set_latency_target();
    
    printf("[ECAT - INFO] start init process\n");
    // bool init_result = initTocabiSystem(init_args);
    // if (!init_result)
    // {
    //     printf("[ECAT - ERRO] init failed\n");
    //     return -1;
    // }

    // printf("[ECAT - INFO] init process has been done\n");

    // /* create RT thread */
    // // osal_thread_create_rt(&thread1, stack64k * 2, (void*) &ethercatThread1, NULL);
    // // osal_thread_create(&thread2, stack64k * 4, (void*) &ethercatThread2, NULL);

    // // cpu_set_t cpuset;
    // // CPU_ZERO(&cpuset);
    // // CPU_SET(lock_core_, &cpuset);

    // // ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    // // if (ret)
    // // {
    // //     printf("pthread setaffinity failed\n");
    // //     return ret;
    // // }

    // // pthread_join(thread1, NULL);
    // // pthread_join(thread2, NULL);


    // printf("[ECAT - INFO] start main threads\n");
    // /* Initialize pthread attributes (default values) */
    // ret = pthread_attr_init(&attr);
    // if (ret)
    // {
    //     printf("init pthread attributes failed\n");
    //     return ret;
    // }

    // ret = pthread_attr_init(&attr2);
    // if (ret)
    // {
    //     printf("init pthread attributes failed\n");
    //     return ret;
    // }

    // /* Set scheduler policy and priority of pthread */
    // ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    // if (ret)
    // {
    //     printf("pthread setschedpolicy failed\n");
    //     return ret;
    // }
    // param.sched_priority = 80;
    // ret = pthread_attr_setschedparam(&attr, &param);
    // if (ret)
    // {
    //     printf("pthread setschedparam failed\n");
    //     return ret;
    // }

    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // CPU_SET(lock_core_, &cpuset);

    // ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    // if (ret)
    // {
    //     printf("pthread setaffinity failed\n");
    //     return ret;
    // }

    // /* Use scheduling parameters of attr */
    // ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    // if (ret)
    // {
    //     printf("pthread setinheritsched failed\n");
    //     return ret;
    // }

    // // /* Create a pthread with specified attributes */
    // ret = pthread_create(&thread1, &attr, ethercatThread1, NULL);
    // if (ret)
    // {
    //     printf("create pthread 1 failed\n");
    //     return ret;
    // }
    // ret = pthread_create(&thread2, &attr2, ethercatThread2, NULL);
    // if (ret)
    // {
    //     printf("create pthread 2 failed\n");
    //     return ret;
    // }

    // pthread_attr_destroy(&attr);
    // pthread_attr_destroy(&attr2);

    // /* Join the thread and wait until it is done */
    // ret = pthread_join(thread1, NULL);
    // if (ret)
    //     printf("join pthread failed: %m\n");

    // ret = pthread_join(thread2, NULL);
    // if (ret)
    //     printf("join pthread failed: %m\n");

    printf("[ECAT - INFO] cleaning up\n");
    cleanupTocabiSystem();
    return 0;
}