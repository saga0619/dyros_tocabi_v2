#include "tocabi_sensor/sensor_manager.h"
#include <thread>
#include <chrono>

SensorManager::SensorManager()
{
    gui_command_sub_ = nh_.subscribe("/tocabi/command", 100, &SensorManager::GuiCommandCallback, this);
    gui_state_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("/tocabi/systemstate", 100);
}

void SensorManager::GuiCommandCallback(const std_msgs::StringConstPtr &msg)
{

    if (msg->data == "imureset")
    {
        imu_reset_signal_ = true;
    }
    else if(msg->data == "ftcalib")
    {
        ft_calib_signal_ = true;
    }
}

void *SensorManager::SensorThread(void)
{

    mscl::Connection con_;
    bool imu_ok = true;

    try
    {
        con_ = mscl::Connection::Serial("/dev/ttyACM0", 115200);
    }
    catch (mscl::Error &err)
    {
        std::cout << "imu connection errer " << err.what() << std::endl;
        imu_ok = false;
    }

    if (imu_ok)
    {
        mscl::InertialNode node(con_);
        sensor_msgs::Imu imu_msg;

        MX5IMU mx5(node);

        mx5.initIMU();

        int cycle_count = 0;

        std::cout << "Sensor Thread Start" << std::endl;

        auto t_begin = std::chrono::steady_clock::now();

        shm_->imu_state = 0;
        shm_->ft_state = 0;

        while (!shm_->shutdown && ros::ok())
        {
            ros::spinOnce();

            std::this_thread::sleep_until(t_begin + cycle_count * std::chrono::microseconds(1000));
            cycle_count++;

            //if signal_ imu reset

            if (imu_reset_signal_)
            {
                mx5.resetEFIMU();
                imu_reset_signal_ = false;
            }
            imu_msg = mx5.getIMU(shm_->imu_state);
            mx5.checkIMUData();
            shm_->imuWriting = true;
            shm_->pos_virtual[3] = imu_msg.orientation.x;
            shm_->pos_virtual[4] = imu_msg.orientation.y;
            shm_->pos_virtual[5] = imu_msg.orientation.z;
            shm_->pos_virtual[6] = imu_msg.orientation.w;
            shm_->vel_virtual[3] = imu_msg.angular_velocity.x;
            shm_->vel_virtual[4] = imu_msg.angular_velocity.y;
            shm_->vel_virtual[5] = imu_msg.angular_velocity.z;
            shm_->imu_acc[0] = imu_msg.linear_acceleration.x;
            shm_->imu_acc[1] = imu_msg.linear_acceleration.y;
            shm_->imu_acc[2] = imu_msg.linear_acceleration.z;
            //std::cout<<shm_->pos_virtual[3]<<shm_->pos_virtual[4]<<shm_->pos_virtual[5]<<shm_->pos_virtual[6]<<std::endl;
            shm_->imuWriting = false;




            //FT sensor related functions ... 

            if(ft_calib_signal_)
            {
                std::cout<<"FT : start calibration ..."<<std::endl;
                ft_calib_signal_ = false;
            }

            shm_->ftWriting = true;

            //Write FT data to shm here
            //for (int i = 0; i < 12; i++)
            //    shm_->ftSensor[i] = ftdata[i];
            
            shm_->ftWriting = false;

            //std::cout << "while end" << std::endl;
        }

        //std::cout << "imu end" << std::endl;

        mx5.endIMU();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_manager");
    SensorManager sm_;

    if ((shm_msg_id = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed " << std::endl;
        exit(0);
    }

    if ((sm_.shm_ = (SHMmsgs *)shmat(shm_msg_id, NULL, 0)) == (SHMmsgs *)-1)
    {
        std::cout << "shmat failed " << std::endl;
        exit(0);
    }
    sm_.shm_->shutdown = false;
    sm_.shm_->process_num++;

    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;

    param.sched_priority = 80;
    // cpu_set_t cpusets[thread_number];

    //set_latency_target();

    /* Initialize pthread attributes (default values) */

    if (pthread_attr_init(&attr))
    {
        printf("attr init failed ");
    }

    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO))
    {
        printf("attr setschedpolicy failed ");
    }
    if (pthread_attr_setschedparam(&attr, &param))
    {
        printf("attr setschedparam failed ");
    }

    // CPU_ZERO(&cpusets[i]);
    // CPU_SET(5 - i, &cpusets[i]);
    // if (pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpusets[i]))
    // {
    //     printf("attr %d setaffinity failed ", i);
    // }

    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED))
    {
        printf("attr setinheritsched failed ");
    }

    if (pthread_create(&thread, &attr, &SensorManager::SensorThread_starter, &sm_))
    {
        printf("threads[0] create failed\n");
    }

    pthread_join(thread, NULL);

    sm_.shm_->process_num--;
    if (sm_.shm_->process_num == 0)
        deleteSharedMemory();

    return 0;
}