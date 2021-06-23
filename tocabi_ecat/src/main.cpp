#include "tocabi_ecat/tocabi_ecat.h"
#include <cstring>

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;
static void set_latency_target(void)
{
    struct stat s;
    int err;
    int errno_;
    errno_ = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1)
    {
        std::cout << "WARN: stat /dev/cpu_dma_latency failed" << std::endl;
        return;
    }

    errno_ = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1)
    {
        std::cout << "WARN: open /dev/cpu_dma_latency" << std::endl;
        return;
    }

    errno_ = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1)
    {
        std::cout << "# error setting cpu_dma_latency to %d!" << latency_target_value << std::endl;
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}

int main(int argc, char *argv[])
{
    bool val_received = false;

    int ret;

    if (argc != 4)
    {
        std::cout << "usage : tocabi_ecat {port} {period_us} {ecat num}" << std::endl;

        goto out;
    }
    else
    {
        soem_port = argv[1];
        period_ns = atoi(argv[2]);
        expected_counter = atoi(argv[3]);

        std::cout << " ecat port : " << soem_port << std::endl;
        std::cout << " period_us  : " << period_ns << std::endl;
        std::cout << " elmo num  : " << expected_counter << std::endl;
        std::cout << " ----------------------------- " << std::endl;
        std::cout << " command :  q(quit), l(lower init), u(upper init), d(debug), p(position), h(homming), c(force control)" << std::endl;

        val_received = true;

        struct sched_param param;
        pthread_attr_t attr, attr2;
        pthread_t thread1, thread2;
        //set_latency_target();

        /* Initialize pthread attributes (default values) */
        ret = pthread_attr_init(&attr);
        if (ret)
        {
            printf("init pthread attributes failed\n");
            goto out;
        }

        ret = pthread_attr_init(&attr2);

        /* Set scheduler policy and priority of pthread */
        ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (ret)
        {
            printf("pthread setschedpolicy failed\n");
            goto out;
        }
        param.sched_priority = 80;
        ret = pthread_attr_setschedparam(&attr, &param);
        if (ret)
        {
            printf("pthread setschedparam failed\n");
            goto out;
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(7, &cpuset);

        ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
        if (ret)
        {
            printf("pthread setaffinity failed\n");
            goto out;
        }

        /* Use scheduling parameters of attr */
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (ret)
        {
            printf("pthread setinheritsched failed\n");
            goto out;
        }

        /* Create a pthread with specified attributes */
        ret = pthread_create(&thread2, &attr2, ethercatThread2, NULL);

        ret = pthread_create(&thread1, &attr, ethercatThread1, NULL);

        if (ret)
        {
            printf("create pthread failed\n");
            goto out;
        }
        pthread_attr_destroy(&attr);
        pthread_attr_destroy(&attr2);

        /* Join the thread and wait until it is done */
        ret = pthread_join(thread1, NULL);
        if (ret)
            printf("join pthread failed: %m\n");

        ret = pthread_join(thread2, NULL);
        if (ret)
            printf("join pthread failed: %m\n");
    }

out:
    return ret;
}