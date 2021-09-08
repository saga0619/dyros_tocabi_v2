#include "tocabi_ecat/tocabi_ecat_rtnet.h"
// #include <ros/ros.h>
// #include <cstring>
#include "sys/mman.h"
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

int main(int argc, char **argv)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    // ros::init(argc, argv, "tocabi_ecat_upper");
    // ros::NodeHandle nh_;

    TocabiInitArgs init_args;
    strcpy(init_args.port1, "rteth2");
    strcpy(init_args.port2, "rteth3");
    // soem_port = argv[1];
    init_args.period_ns = 500 * 1000;
    init_args.ecat_slave_num = 18;
    init_args.ecat_slave_start_num = 0;
    init_args.lock_core = 7;
    init_args.ecat_device = 1;
    init_args.is_main = true;
    strcpy(init_args.commutation_cache_file, "/home/dyros/.tocabi_bootlog/commutationlog_upper");
    strcpy(init_args.zeropoint_cache_file, "/home/dyros/.tocabi_bootlog/zeropointlog_upper");
    // nh_.param("/tocabi_ecat_upper/verbose", init_args.verbose, true);

    // nh_.getParam("/tocabi_controller/vellimit", joint_velocity_limit);
    // nh_.getParam("/tocabi_controller/jointlimit_u", joint_upper_limit);
    // nh_.getParam("/tocabi_controller/jointlimit_l", joint_lower_limit);
    // nh_.getParam("/tocabi_controller/NM2CNT", NM2CNT);

    struct sched_param param;
    pthread_attr_t attr, attr2;
    pthread_t thread1, thread2;
    int ret;
    //set_latency_target();


    printf("[ECAT - INFO] start main threads\n");
    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        return ret;
    }

    ret = pthread_attr_init(&attr2);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        return ret;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        return ret;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        return ret;
    }

    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // CPU_SET(7, &cpuset);

    // ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    // if (ret)
    // {
    //     printf("pthread setaffinity failed\n");
    //     return ret;
    // }

    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        return ret;
    }

    /* Create a pthread with specified attributes */

    ret = pthread_create(&thread1, &attr, ethercatThread1, &init_args);
    if (ret)
    {
        printf("create pthread 1 failed\n");
        return ret;
    }
    ret = pthread_create(&thread2, &attr2, ethercatThread2, &init_args);
    if (ret)
    {
        printf("create pthread 2 failed\n");
        return ret;
    }


    printf("[ECAT - INFO] start init process\n");
    bool init_result = initTocabiSystem(init_args);
    if (!init_result)
    {
        printf("[ECAT - ERRO] init failed\n");
        return -1;
    }

    printf("[ECAT - INFO] init process has been done\n");


    pthread_attr_destroy(&attr);
    pthread_attr_destroy(&attr2);

    /* Join the thread and wait until it is done */
    ret = pthread_join(thread1, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

    ret = pthread_join(thread2, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

    printf("[ECAT - INFO] cleaning up\n");
    cleanupTocabiSystem();
    return 0;
}