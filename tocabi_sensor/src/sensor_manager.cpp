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
    else if (msg->data == "ftcalib")
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
            int imu_state__;
            imu_msg = mx5.getIMU(imu_state__);

            static int no_imu_count = 0;
            static int yes_imu_count = 0;
            if (imu_state__ == -1)
            {
                no_imu_count++;
            }
            else
            {
                yes_imu_count++;
                mx5.checkIMUData();
                shm_->imuWriting = true;
                shm_->imu_state = imu_state__;
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
            }

            if ((cycle_count % 1000) == 0)
            {
                static int tb_;
                int ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_begin).count();

                if ((mx5.packet_num < 490) || (mx5.packet_num > 510))
                {
                    std::cout << (ts - tb_) << " : imu packet error, count : " << mx5.packet_num << std::endl;
                }
                tb_ = ts;
                mx5.packet_num = 0;
            }

            //FT sensor related functions ...

            if (ft_calib_signal_)
            {
                std::cout << "FT : start calibration ..." << std::endl;
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
    int shm_id_;

    init_shm(shm_msg_key, shm_id_, &sm_.shm_);

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

    deleteSharedMemory(shm_id_, sm_.shm_);

    return 0;
}