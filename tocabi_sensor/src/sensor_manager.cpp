#include "tocabi_sensor/sensor_manager.h"
#include <thread>
#include <chrono>
#include <signal.h>

volatile bool *prog_shutdown;

void SIGINT_handler(int sig)
{
    // std::cout << " SENSOR : shutdown Signal" << std::endl;
    *prog_shutdown = true;
}

SensorManager::SensorManager()
{
    gui_command_sub_ = nh_.subscribe("/tocabi/command", 100, &SensorManager::GuiCommandCallback, this);
    gui_state_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("/tocabi/systemstate", 100);
    imu_pub = nh_.advertise<sensor_msgs::Imu>("/tocabi/imu", 100);
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
    else if (msg->data == "E0")
    {
        shm_->emergencyOff = true;
    }
    else if (msg->data == "E1")
    {
        // pos
    }
}

void *SensorManager::SensorThread(void)
{

    mscl::Connection con_;
    bool imu_ok = true;
    bool is_ft_board_ok = true;
    bool ft_calib_init = false;
    bool ft_calib_ui = false;
    bool ft_calib_finish = false;
    bool ft_init_write = false;
    bool ft_init_load = false;
    int ft_cycle_count;
    char ft_cache_file[100];

    strcpy(ft_cache_file, "/home/dyros/.tocabi_bootlog/ftsensor");

    try
    {
        con_ = mscl::Connection::Serial("/dev/ttyACM0", 115200);
    }
    catch (mscl::Error &err)
    {
        std::cout << "imu connection errer " << err.what() << std::endl;
        imu_ok = false;
    }

    sensoray826_dev ft = sensoray826_dev(1);
    is_ft_board_ok = ft.open();
    atiforce hand_ft;
    Response r;
    SOCKET_HANDLE socketHandle;
 
    if (!is_ft_board_ok)
    {
        std::cout << "ft connection error, error code" << std::endl;
    }

    ft.analogSingleSamplePrepare(slotAttrs, 16);
    ft.initCalibration();

    if (hand_ft.Connect(&socketHandle, "192.168.1.1" , PORT) != 0)
    {
        fprintf(stderr, "Could not connect to device...");
    }
    else
    {
        fprintf(stderr, "connect device!!!");
    }

    if (imu_ok)
    {
        mscl::InertialNode node(con_);
        sensor_msgs::Imu imu_msg;

        MX5IMU mx5(node);

        mx5.initIMU();

        int cycle_count = 0;

        // std::cout << "Sensor Thread Start" << std::endl;

        auto t_begin = std::chrono::steady_clock::now();

        shm_->imu_state = 0;
        shm_->ft_state = 0;

        while (!shm_->shutdown && ros::ok())
        {
            ros::spinOnce();

            std::this_thread::sleep_until(t_begin + cycle_count * std::chrono::microseconds(1000));
            cycle_count++;

            // if signal_ imu reset

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
                // std::cout<<shm_->pos_virtual[3]<<shm_->pos_virtual[4]<<shm_->pos_virtual[5]<<shm_->pos_virtual[6]<<std::endl;

                imu_pub.publish(imu_msg);
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

            // FT sensor related functions ...
            ft.analogOversample();

            hand_ft.SendCommand(&socketHandle);
            r = hand_ft.Receive(&socketHandle);
            hand_ft.ShowResponse(r);

            std::string tmp;
            int i = 0;

            if (ft_init_load == false)
            {
                ft_init_log.open(ft_cache_file, std::ios_base::in);

                if (ft_init_log.is_open())
                {
                    while (getline(ft_init_log, tmp))
                    {
                        if (i < 6)
                        {
                            ft.leftFootBias[i] = atof(tmp.c_str());
                        }
                        else
                        {
                            ft.rightFootBias[i - 6] = atof(tmp.c_str());
                        }
                        i++;
                    }
                    ft_init_load = true;
                    ft_init_log.close();
                    std::cout << cgreen << "    FT : Calibration data loaded ..." << creset << std::endl;
                    //   pub_to_gui(dc, "ft bias loaded");
                    //   dc.ft_state = 2;

                    shm_->ft_state = 2;
                }
                else
                {
                    //  pub_to_gui(dc, "ft bias load failed");
                }
            }

            if (ft_calib_signal_) // enabled by gui
            {
                // dc.ft_state = 1;
                shm_->ft_state = 1;
                if (ft_calib_init == false)
                {
                    ft_cycle_count = cycle_count;
                    ft_calib_init = true;

                    for (int i = 0; i < 6; i++)
                    {
                        ft._calibLFTData[i] = 0.0;
                        ft._calibRFTData[i] = 0.0;
                    }
                    std::cout << "    FT : start calibration ..." << std::endl;
                    // ROS_INFO("FT : start calibration ...");
                    // pub_to_gui(dc, "ft sensor : calibration ... ");
                }
                if (cycle_count < 5 * SAMPLE_RATE + ft_cycle_count)
                {
                    if (cycle_count == 5 * SAMPLE_RATE + ft_cycle_count - 1)
                    {
                        ft_calib_finish = true;
                        ft_calib_signal_ = false;
                        // dc.ftcalib = false;
                    }

                    ft.calibrationFTData(ft_calib_finish);
                }
            }
            else
            {
                if (ft_calib_finish == true)
                {
                    ft_calib_init = false;
                }
            }

            if (ft_calib_ui == false && ft_calib_finish == true)
            {
                /*    dc.print_ft_info_tofile = true;
                pub_to_gui(dc, "ft sensor : calibration finish ");
                pub_to_gui(dc, "ftgood"); */
                printf("    FT : calibration finish\n");

                ft_init_log.open(ft_cache_file, std::ios_base::out);
                if (ft_init_log.is_open())
                {
                    for (int i = 0; i < 6; i++)
                    {
                        ft_init_log << ft.leftFootBias[i] << "\n";
                    }

                    for (int i = 0; i < 6; i++)
                    {
                        ft_init_log << ft.rightFootBias[i] << "\n";
                    }
                }
                ft_init_log.close();
                shm_->ft_state = 2;
                ft_calib_ui = true;
            }

            ft.computeFTData(ft_calib_finish);

            shm_->ftWriting = true;

            // Write FT data to shm here
            for (int i = 0; i < 6; i++)
            {
                shm_->ftSensor[i] = ft.leftFootAxisData[i];
                shm_->ftSensor[i + 6] = ft.rightFootAxisData[i];
            }

            shm_->ftWriting = false;

            shm_->ftWriting2 = true;
            for (int i = 0; i < 6; i++)
            {
                shm_->ftSensor2[i] = (double)r.FTData[i]/1000000.0;
                shm_->ftSensor2[i + 6] = 0.0;
            }
            shm_->ftWriting2 = false;
            // std::cout << "while end" << std::endl;
        }

        // std::cout << "imu end" << std::endl;

        mx5.endIMU();
    }
    return (void *)NULL;
}

int main(int argc, char **argv)
{
    signal(SIGINT, SIGINT_handler);
    ros::init(argc, argv, "sensor_manager", ros::init_options::NoSigintHandler);
    SensorManager sm_;
    int shm_id_;

    init_shm(shm_msg_key, shm_id_, &sm_.shm_);

    prog_shutdown = &sm_.shm_->shutdown;

    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;

    param.sched_priority = 41;
    // cpu_set_t cpusets[thread_number];

    // set_latency_target();

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
