#include "tocabi_controller/state_manager.h"
#include <thread>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tocabi_controller");


    DataContainer dc_;
    
    StateManager stm(dc_);
    
    //stm.stateThread();


    
    struct sched_param param;
    pthread_attr_t attr, attr2;
    pthread_t thread1, thread2;
    int ret;
    //set_latency_target();

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
    
    }

    ret = pthread_attr_init(&attr2);

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
     
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
       
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(6, &cpuset);

    ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");

    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&thread1, &attr, &StateManager::thread_helper, &stm);

    pthread_attr_destroy(&attr);

    /* Join the thread and wait until it is done */
    pthread_join(thread1, NULL);



    std::cout << "tocabi controller Shutdown" << std::endl;
    return 0;
}