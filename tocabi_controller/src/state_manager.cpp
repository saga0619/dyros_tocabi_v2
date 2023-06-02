#include "tocabi_controller/state_manager.h"
#include "fstream"
#include "algorithm"
#include <iomanip>
#include <unistd.h>

using namespace std;
using namespace TOCABI;

volatile bool qdot_estimation_switch = false;

StateManager::StateManager(DataContainer &dc_global) : dc_(dc_global), rd_gl_(dc_global.rd_)
{
    string urdf_path;

    ros::param::get("/tocabi_controller/urdf_path", urdf_path);

    bool verbose = false;

    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_local_, true, verbose);
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_global_, true, verbose);

    if (model_local_.dof_count == MODEL_DOF_VIRTUAL)
    {
        A_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_id_[i] = model_local_.GetBodyId(TOCABI::LINK_NAME[i]);

            if (!model_local_.IsBodyId(link_id_[i]))
            {
                ROS_INFO_COND(verbose, "Failed to get body id at link %d : %s", i, TOCABI::LINK_NAME[i]);
            }
        }

        total_mass_ = 0;

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_[i].Initialize(model_global_, link_id_[i]);
            total_mass_ += link_[i].mass;
        }
        rd_gl_.total_mass_ = total_mass_;

        link_[Right_Foot].contact_point << 0.03, 0, -0.1585;
        link_[Right_Foot].sensor_point << 0.0, 0.0, -0.09;
        link_[Left_Foot].contact_point << 0.03, 0, -0.1585;
        link_[Left_Foot].sensor_point << 0.0, 0.0, -0.09;

        link_[Right_Hand].contact_point << 0, 0.0, -0.1542;
        link_[Right_Hand].sensor_point << 0.0, 0.0, -0.1028;
        link_[Left_Hand].contact_point << 0, 0.0, -0.1542;
        link_[Left_Hand].sensor_point << 0.0, 0.0, -0.1028;

        memcpy(link_local_, link_, sizeof(LinkData) * LINK_NUMBER);
    }

    if (dc_.simMode)
    {
        mujoco_sim_command_pub_ = dc_.nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 1);
        mujoco_sim_command_sub_ = dc_.nh.subscribe("/mujoco_ros_interface/sim_command_sim2con", 1, &StateManager::SimCommandCallback, this);
    }

    hand_torque_r_sub_ = dc_.nh.subscribe("/hand/r/current", 1, &StateManager::handrcurrentCallback, this);
    hand_torque_l_sub_ = dc_.nh.subscribe("/hand/l/current", 1, &StateManager::handlcurrentCallback, this);

    joint_state_pub_ = dc_.nh.advertise<sensor_msgs::JointState>("/tocabi/jointstates", 1);

    joint_state_msg_.name.resize(MODEL_DOF);
    joint_state_msg_.position.resize(MODEL_DOF);
    joint_state_msg_.velocity.resize(MODEL_DOF);
    joint_state_msg_.effort.resize(MODEL_DOF);

    for (int i = 0; i < MODEL_DOF; i++)
        joint_state_msg_.name[i] = JOINT_NAME[i];

    gui_command_sub_ = dc_.nh.subscribe("/tocabi/command", 1, &StateManager::GuiCommandCallback, this);
    gui_state_pub_ = dc_.nh.advertise<std_msgs::Int8MultiArray>("/tocabi/systemstate", 1);
    point_pub_ = dc_.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/point", 1);
    status_pub_ = dc_.nh.advertise<std_msgs::String>("/tocabi/guilog", 1);
    timer_pub_ = dc_.nh.advertise<std_msgs::Float32>("/tocabi/time", 1);
    head_pose_pub_ = dc_.nh.advertise<geometry_msgs::Pose>("/tocabi/headpose", 1);

    stop_tocabi_sub_ = dc_.nh.subscribe("/tocabi/stopper", 1, &StateManager::StopCallback, this);
    elmo_status_pub_ = dc_.nh.advertise<std_msgs::Int8MultiArray>("/tocabi/ecatstates", 1);
    com_status_pub_ = dc_.nh.advertise<std_msgs::Float32MultiArray>("/tocabi/comstates", 1);

    hand_ft_pub_ = dc_.nh.advertise<std_msgs::Float32MultiArray>("/tocabi/handft_global", 1);

    hand_ft_pub_org_ = dc_.nh.advertise<std_msgs::Float32MultiArray>("/tocabi/handft_raw", 100);

    com_status_msg_.data.resize(17);
    point_pub_msg_.polygon.points.resize(24);
    syspub_msg.data.resize(8);
    elmo_status_msg_.data.resize(MODEL_DOF * 3);
    hand_ft_msg_.data.resize(12);
    hand_ft_org_msg_.data.resize(12);
    LH_CALIB.resize(6, 3);
    LH_CALIB.setZero();
    RH_CALIB.resize(6, 3);
    RH_CALIB.setZero();

    rd_.task_force_.setZero(6);
}

StateManager::~StateManager()
{
    cout << "StateManager Terminate" << endl;
}

void *StateManager::StateThread()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    cout << " STATE : started with pid : " << getpid() << std::endl;

    int rcv_tcnt = -1;

    // Checking Connect//

    // Check Coonnect Complete//
    rcv_tcnt = dc_.tc_shm_->statusCount;
    // cout << "first packet " << rcv_tcnt << endl;
    int cycle_count_ = rcv_tcnt;
    dc_.stm_cnt = 0;

    int cnt = 0;
    int cnt2 = 0;
    int cnt3 = 0;
    auto time_start = std::chrono::steady_clock::now();

    timespec tv_us1;
    tv_us1.tv_sec = 0;
    tv_us1.tv_nsec = 10000;

    initializeJointMLP();
    loadJointVelNetwork("/home/dyros/joint_vel_net/");

    while (true)
    {
        if (dc_.tc_shm_->shutdown)
            break;
        //////////////////////////////
        //////////State Loop//////////
        //////////////////////////////

        auto t0 = std::chrono::steady_clock::now();

        while (!dc_.tc_shm_->triggerS1)
        {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &tv_us1, NULL);

            __asm__("pause" ::
                        : "memory");
            if (dc_.tc_shm_->shutdown)
                break;
        }

        rd_.tp_state_ = std::chrono::steady_clock::now();
        auto t1 = rd_.tp_state_;

        if (chrono::duration_cast<chrono::microseconds>(t1 - t0).count() > 500)
        {
            if (control_time_ > 0.5)
            {
                if (!dc_.tc_shm_->shutdown)
                {
                    if (!dc_.simMode)
                        std::cout << " STATE : Waiting for signal for over 500us, " << chrono::duration_cast<chrono::microseconds>(t1 - t0).count() << " at, " << control_time_ << std::endl;
                }
            }
        }

        dc_.tc_shm_->triggerS1 = false;
        cycle_count_++;
        dc_.stm_cnt++;

        rcv_tcnt = dc_.tc_shm_->statusCount;

        GetJointData(); // 0.246 us //w/o march native 0.226

        InitYaw();

        auto d1 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t1).count();

        auto t2 = chrono::steady_clock::now();

        auto dur_start_ = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_start).count();
        control_time_ = rcv_tcnt / 2000.0;

        // local kinematics update : 33.7 us // w/o march native 20 us
        auto d2 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t2).count();

        auto t3 = chrono::steady_clock::now();
        UpdateKinematics_local(model_local_, link_local_, q_virtual_local_, q_dot_virtual_local_, q_ddot_virtual_local_);

        GetSensorData();

        auto d3 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t3).count();

        auto t4 = chrono::steady_clock::now();
        StateEstimate();

        // global kinematics update : 127 us //w/o march native 125 us
        UpdateKinematics(model_global_, link_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);

        UpdateCMM(rd_, link_);

        // check signal
        static bool grav_sig = false;
        static bool pos_sig = false;
        static int rcv_count = 0;

        static const int ignore_count = 20; // 10ms

        if (grav_sig != dc_.tc_shm_->grav_signal)
        {
            if (rcv_tcnt > rcv_count + ignore_count)
            {

                if (grav_sig)
                {
                    std::cout << control_time_ << "GRAV SIG ON" << std::endl;
                    rd_gl_.tc_run = false;
                    rd_gl_.pc_mode = false;
                }
                else
                    std::cout << control_time_ << "GRAV SIG OFF" << std::endl;
            }

            rcv_count = rcv_tcnt;
        }

        if (pos_sig != dc_.tc_shm_->pos_signal)
        {
            if (pos_sig)
            {
                std::cout << control_time_ << "POS SIG ON" << std::endl;
                rd_gl_.positionControlSwitch = true;
            }
            else
                std::cout << control_time_ << "POS SIG OFF" << std::endl;

            rcv_count = rcv_tcnt;
        }

        grav_sig = dc_.tc_shm_->grav_signal;
        pos_sig = dc_.tc_shm_->pos_signal;

        // rd_gl_.position_signal = pos_sig;
        // rd_gl_.gravity_signal = grav_sig;

        // MeasureTime(dc_.stm_cnt, d1, d2);

        // rd_gl_.control_time_ = dur_start_ / 1000000.0;
        // rd_gl_.control_time_us_ = dur_start_;
        dc_.tc_shm_->control_time_us_ = rcv_tcnt * 500;
        control_time_us_l_ = rcv_tcnt * 500;

        StoreState(rd_gl_); // 6.2 us //w/o march native 8us

        // dc_.tc_shm_->t_cnt2 = dc_.stm_cnt;
        // dc_.tc_shm_->t_cnt2 = cnt3;

        dc_.tc_shm_->stloopCount.store(dc_.stm_cnt);

        SendCommand();

        auto d4 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t4).count();
        rd_gl_.state_ctime_total_ += (d1 + d2 + d3 + d4);
        rd_gl_.state_ctime_avg_ = rd_gl_.state_ctime_total_ / dc_.stm_cnt;

        static int e_cnt = dc_.tc_shm_->statusCount;
        static int stm_diff = dc_.stm_cnt - e_cnt;
        static int tcm_diff = dc_.tcm_cnt - e_cnt + 1;

        static bool pub_once = true;

        if (pub_once)
        {

            std::cout << "STATUS : init stm diff : " << stm_diff << "  init tcm diff : " << tcm_diff << std::endl;

            if (stm_diff < -5000 && tcm_diff < -5000)
            {
                std::cout << "STATUS : REBOOT MODE ! " << std::endl;

                bool safety_status = true;
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    safety_status = safety_status && (dc_.tc_shm_->safety_status[i] == 4);
                }
                if (safety_status)
                {
                    std::cout << "STATUS : REBOOT FROM COMMANDLOCK!!!!!!!!!!!!!!" << std::endl;

                    std::cout << "STATUS : POSITION COMMAND!" << std::endl;

                    dc_.tc_shm_->safety_reset_upper_signal = true;
                    dc_.tc_shm_->safety_reset_lower_signal = true;

                    dc_.torqueOn = true;
                    cCount = dc_.tc_shm_->commandCount;
                    torqueRatio = 1.0;

                    if (dc_.avatarMode)
                    {
                        dc_.inityawSwitch = true;
                        dc_.stateEstimateSwitch = true;
                        rd_gl_.semode = true;

                        rd_gl_.tc_avatar_switch = true;

                        rd_gl_.avatar_reboot_signal = true;
                    }
                }
            }

            pub_once = false;
        }

        if (dc_.stm_cnt % 10 == 0 && !dc_.simMode)
        {
            int e_cnt_l = dc_.tc_shm_->statusCount;
            int stm_diff_l = dc_.stm_cnt - e_cnt_l;
            int tcm_diff_l = dc_.tcm_cnt - e_cnt_l + 1;

            if (abs(stm_diff - stm_diff_l) > 1)
            {
                // StatusPub("STATUS : STM DESYNC AT %7.1f, ecat cnt : %d, stm cnt : %d, dcm cnt : %d, %d, %d", control_time_, e_cnt_l, (int)dc_.stm_cnt - e_cnt_l, (int)dc_.tcm_cnt - e_cnt_l, stm_diff, stm_diff_l);
                printf("STATUS : STM DESYNC AT %7.1f, ecat cnt : %d, stm cnt : %d, dcm cnt : %d\n", control_time_, e_cnt_l, (int)dc_.stm_cnt - e_cnt_l, (int)dc_.tcm_cnt - e_cnt_l);
                stm_diff = stm_diff_l;
            }

            if (abs(tcm_diff - tcm_diff_l) > 1)
            {
                // StatusPub("STATUS : TCM DESYNC AT %7.1f, ecat cnt : %d, stm cnt : %d, dcm cnt : %d, %d, %d", control_time_, e_cnt_l, (int)dc_.stm_cnt - e_cnt_l, (int)dc_.tcm_cnt - e_cnt_l, tcm_diff, tcm_diff_l);
                printf("STATUS : TCM DESYNC AT %7.1f, ecat cnt : %d, stm cnt : %d, dcm cnt : %d\n", control_time_, e_cnt_l, (int)dc_.stm_cnt - e_cnt_l, (int)dc_.tcm_cnt - e_cnt_l);
                tcm_diff = tcm_diff_l;
            }
        }

        if ((d1 + d2 + d3 + d4) > 500)
        {
            if (control_time_ > 0.1)
                printf(" STATE : %7.1f stm over 500, d1 : %ld, d2 : %ld, d3 : %ld, d4 : %ld\n", control_time_, d1, d2, d3, d4);
        }

        for (int i = 0; i < MODEL_DOF; i++)
        {

            state_safety_before_[i] = state_safety_[i];
        }

        // printf("%d\n", rcv_tcnt);
        // printf("\x1b[A\x1b[A\33[2K\r");
        //  if (rcv_tcnt % 33 == 0)
        //  {
        //      printf("\33[2K\r");
        //      printf("%8d %8d avg : %7.3f max : %4d min : %4d, cnt : %4d, cnt2 : %4d, cnt3 : %4d ", rcv_tcnt, cycle_count_, avg, max, min, cnt, cnt2, cnt3);
        //      fflush(stdout);
        //  }
    }
    cout << " STATE : StateManager END" << endl;
    return (void *)NULL;
}

void *StateManager::LoggerThread()
{
    bool activateLogger = false;
    bool startLogger = false;

    dc_.nh.getParam("/tocabi_controller/log", activateLogger);

    // wait for both ecat are in control mode !
    //  while (!dc_.tc_shm_->shutdown)
    //  {
    //      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //      if (dc_.tc_shm_->controlModeLower && dc_.tc_shm_->controlModeUpper)
    //      {
    //          std::cout << "Logger : Both ECAT is now on CONTROL MODE! Logging start..." << std::endl;
    //          break;
    //      }
    //  }

    std::string username = "dyros";
    // std::cout << "username : " << username << std::endl;

    std::string log_folder = "/home/" + username + "/tocabi_log/";

    std::ostringstream start_time;

    auto t_ = std::time(nullptr);
    auto tm_ = *std::localtime(&t_);
    start_time << std::put_time(&tm_, "%Y%m%d-%H%M%S");

    std::string torqueLogFile = "torque_elmo_log";
    std::string ecatStatusFile = "ecat_status_log";
    std::string torqueclogFile = "torque_command_log";
    std::string torqueActualLogFile = "torque_actual_log";
    std::string posLogFile = "pos_log";
    std::string velLogFile = "vel_log";
    std::string maskLogFile = "mask_log";
    std::string posDesiredLogFile = "pos_des_log";
    std::string velDesiredLogFile = "vel_des_log";
    std::string sensorLogFile = "sensor_log";

    bool switch_torqueLog = false;        // int
    bool switch_torqueCommandLog = false; //
    bool switch_torqueActualLog = false;  //
    bool switch_maskLog = false;
    bool switch_ecatStatusLog = false;
    bool switch_posLog = false;
    bool switch_velLog = false;
    bool switch_posDesiredLog = false;
    bool switch_velDesiredLog = false;
    bool switch_sensorLog = false;

    int record_seconds = 60;
    dc_.nh.getParam("/logger_switch/torqueLog", switch_torqueLog);
    dc_.nh.getParam("/logger_switch/torqueCommandLog", switch_torqueCommandLog);
    dc_.nh.getParam("/logger_switch/torqueActualLog", switch_torqueActualLog);
    dc_.nh.getParam("/logger_switch/maskLog", switch_maskLog);
    dc_.nh.getParam("/logger_switch/ecatStatusLog", switch_ecatStatusLog);
    dc_.nh.getParam("/logger_switch/posLog", switch_posLog);
    dc_.nh.getParam("/logger_switch/velLog", switch_velLog);
    dc_.nh.getParam("/logger_switch/posDesiredLog", switch_posDesiredLog);
    dc_.nh.getParam("/logger_switch/velDesiredLog", switch_velDesiredLog);
    dc_.nh.getParam("/logger_switch/sensorLog", switch_sensorLog);
    dc_.nh.getParam("/logger_switch/record_seconds", record_seconds);

    if (activateLogger)
    {
        std::cout << "Logger : log folder : " << log_folder << "  Start Time : " << start_time.str() << std::endl;

        std::cout << "Logger : ON | record interval : "<< record_seconds<<" s | torque : "
                  << switch_torqueLog
                  << " | toruqeCommand : " << switch_torqueCommandLog
                  << " | torqueActual : " << switch_torqueActualLog
                  << " | mask : " << switch_maskLog << std::endl
                  << " | ecatStatus : " << switch_ecatStatusLog
                  << " | posLog : " << switch_posLog
                  << " | velLog : " << switch_velLog << std::endl
                  << " | posDesired : " << switch_posDesiredLog
                  << " | velDesired : " << switch_velDesiredLog
                  << " | sensorLog : " << switch_sensorLog << std::endl;

    }

    ofstream torqueLog;
    ofstream torqueCommandLog;
    ofstream torqueActualLog;
    ofstream maskLog;
    ofstream ecatStatusLog;
    ofstream posLog;
    ofstream velLog;
    ofstream posDesiredLog;
    ofstream velDesiredLog;
    ofstream sensorLog;

    int log_count = 0;
    int pub_count = 0;
    int s_count = 0;

    timespec ts;

    while (dc_.stm_cnt == 0)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    clock_gettime(CLOCK_MONOTONIC, &ts);


    int record_tick = record_seconds * 2000;

    long current_time = rd_gl_.control_time_us_;

    std::string current_logging_folder;
    std::string previous_logging_folder;

    while (true)
    {
        if (dc_.tc_shm_->shutdown)
        {
            if (s_count > 0)
            {
                if (switch_torqueLog)
                    torqueLog.close();
                if (switch_torqueCommandLog)
                    torqueCommandLog.close();
                if (switch_torqueActualLog)
                    torqueActualLog.close();
                if (switch_maskLog)
                    maskLog.close();
                if (switch_ecatStatusLog)
                    ecatStatusLog.close();
                if (switch_posLog)
                    posLog.close();
                if (switch_posDesiredLog)
                    posDesiredLog.close();
                if (switch_velDesiredLog)
                    velDesiredLog.close();
                if (switch_velLog)
                    velLog.close();
                if (switch_sensorLog)
                    sensorLog.close();


                // std::stringstream sstr;

                // sstr << " zip -j -q " << log_folder << "log_" << start_time.str() << "_" << std::setfill('0') << std::setw(3) << s_count << std::setw(0) << ".zip " << log_folder << apd_ << "* &";

                // int status = system(sstr.str().c_str());
                // std::cout << " log file compressed : " << s_count << std::endl;
            }

            break;
        }

        // hand ft global publish for haptic feedback

        static Vector12d hand_bias = Vector12d::Zero();

        static int hand_pub_tick = 0;
        if (hand_pub_tick == 10)
        {
            for (int i = 0; i < 6; i++)
            {
                // hand_ft_msg_.data[i] = LH_CF_FT[i];
                hand_ft_msg_.data[i] = LH_CF_FT[i] - hand_bias[i];

                hand_ft_msg_.data[i + 6] = RH_CF_FT[i] - hand_bias[6 + i];
            }
            hand_ft_pub_.publish(hand_ft_msg_);
            hand_pub_tick = 0;
        }
        hand_pub_tick++;

        pub_count++;
        if (pub_count % 16 == 0)
        {
            if (current_time < rd_gl_.control_time_us_)
            {
                PublishData();
                current_time = rd_gl_.control_time_us_;
            }
            timer_msg_.data = control_time_;
            timer_pub_.publish(timer_msg_);
        }
        ros::spinOnce();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        ts.tv_nsec += 500000;

        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_nsec -= 1000000000;
            ts.tv_sec++;
        }

        if (dc_.logdata_start)
        {
            activateLogger = true;
            dc_.logdata_start = false;
            std::cout << "Start Logging Data" << std::endl;
        }

        if (activateLogger && (!startLogger)) // Start Logging if activateLogger is true and upper and lower ecat controller is online
        {
            if (dc_.tc_shm_->controlModeLower && dc_.tc_shm_->controlModeUpper)
            {
                startLogger = true;
            }
        }

        if (startLogger)
        {
            static long local_control_time = 0;

            while (local_control_time == control_time_us_l_)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
            local_control_time = control_time_us_l_;

            if (log_count % record_tick == 0)
            {
                auto t = std::time(nullptr);
                auto tm = *std::localtime(&t);
                std::ostringstream oss;
                oss << std::put_time(&tm, "%Y%m%d-%H%M%S");
                auto t_str = oss.str();

                // std::cout << str << std::endl;

                if (s_count % 2 == 0)
                {
                    if (s_count == 0)
                    {
                        std::stringstream create_dir_str;

                        create_dir_str << "mkdir " << log_folder << "/" << start_time.str();
                        int status = system(create_dir_str.str().c_str());
                    }

                    std::stringstream create_dir_str2;

                    previous_logging_folder = log_folder + "/" + start_time.str() + "/" + std::to_string(s_count - 1) + "/";
                    current_logging_folder = log_folder + "/" + start_time.str() + "/" + std::to_string(s_count) + "/";

                    create_dir_str2 << "mkdir " << current_logging_folder;

                    int status = system(create_dir_str2.str().c_str());

                    // apd_ = "0/";
                    // cpd_ = "1/";
                    std::cout << "LOGGER : Open Log Files : " << s_count << " " << t_str << std::endl;
                }
                else
                {
                    std::stringstream create_dir_str2;

                    previous_logging_folder = log_folder + "/" + start_time.str() + "/" + std::to_string(s_count - 1) + "/";
                    current_logging_folder = log_folder + "/" + start_time.str() + "/" + std::to_string(s_count) + "/";

                    create_dir_str2 << "mkdir " << current_logging_folder;
                    int status = system(create_dir_str2.str().c_str());

                    // apd_ = "1/";
                    // cpd_ = "0/";
                    std::cout << "LOGGER : Open Log Files : " << s_count << " " << t_str << std::endl;
                }

                if (s_count > 0)
                {
                    if (switch_torqueLog)
                        torqueLog.close();
                    if (switch_torqueCommandLog)
                        torqueCommandLog.close();
                    if (switch_torqueActualLog)
                        torqueActualLog.close();
                    if (switch_maskLog)
                        maskLog.close();
                    if (switch_ecatStatusLog)
                        ecatStatusLog.close();
                    if (switch_posLog)
                        posLog.close();
                    if (switch_posDesiredLog)
                        posDesiredLog.close();
                    if (switch_velDesiredLog)
                        velDesiredLog.close();
                    if (switch_velLog)
                        velLog.close();
                    if (switch_sensorLog)
                        sensorLog.close();

                    std::stringstream sstr;
                    sstr << "{ zip -j -q " << log_folder << "/" << start_time.str() << "/"
                         << "log_" << std::setfill('0') << std::setw(3) << s_count - 1 << std::setw(0) << ".zip " << previous_logging_folder << "*; rm -rf " << previous_logging_folder << "; }&";

                    int status = system(sstr.str().c_str());

                }

                if (switch_torqueLog)
                {
                    torqueLog.open((current_logging_folder + torqueLogFile).c_str());
                    torqueLog.fill(' ');
                    // torqueLog << t_str << " Direct command input(CNT) to elmo" << std::endl;
                    torqueLog << "time ";
                    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                    {
                        torqueLog << "t" + to_string(i) << " ";
                    }
                    torqueLog << std::endl;
                }

                if (switch_torqueCommandLog)
                {
                    torqueCommandLog.open((current_logging_folder + torqueclogFile).c_str());
                    // torqueCommandLog << t_str << " torque command(NM) to elmo" << std::endl;
                    torqueCommandLog << "time ";
                    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                    {
                        torqueCommandLog << "tcom" + to_string(i) << " ";
                    }
                    torqueCommandLog << std::endl;
                }

                if (switch_torqueActualLog)
                {

                    torqueActualLog.open((current_logging_folder + torqueActualLogFile).c_str());
                    // torqueActualLog << t_str << " Actual torque from elmo" << std::endl;
                    torqueActualLog << "time ";
                    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                    {
                        torqueActualLog << "treal" + to_string(i) << " ";
                    }
                    torqueActualLog << std::endl;
                }

                if (switch_maskLog)
                {
                    maskLog.open((current_logging_folder + maskLogFile).c_str());
                }

                if (switch_ecatStatusLog)
                {
                    ecatStatusLog.open((current_logging_folder + ecatStatusFile).c_str());
                }

                if (switch_posLog)
                {
                    posLog.open((current_logging_folder + posLogFile).c_str());
                    // posLog << t_str << std::endl;
                    posLog << "time ";
                    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                    {
                        posLog << "q" + to_string(i) << " ";
                    }
                    posLog << std::endl;
                }

                if (switch_posDesiredLog)
                {
                    posDesiredLog.open((current_logging_folder + posDesiredLogFile).c_str());
                    // posDesiredLog << t_str << std::endl;
                    posDesiredLog << "time ";
                    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                    {
                        posDesiredLog << "qdes" + to_string(i) << " ";
                    }
                    posDesiredLog << std::endl;
                }

                if (switch_velDesiredLog)
                {

                    velDesiredLog.open((current_logging_folder + velDesiredLogFile).c_str());
                    // velDesiredLog << t_str << std::endl;
                    velDesiredLog << "time ";
                    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                    {
                        velDesiredLog << "qddes" + to_string(i) << " ";
                    }
                    velDesiredLog << std::endl;
                }

                if (switch_velLog)
                {
                    velLog.open((current_logging_folder + velLogFile).c_str());
                    // velLog << t_str << std::endl;
                    velLog << "time ";
                    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                    {
                        velLog << "qdot" + to_string(i) << " ";
                    }
                    velLog << std::endl;
                }

                if (switch_sensorLog)
                {
                    sensorLog.open((current_logging_folder + sensorLogFile).c_str());
                    // sensorLog << t_str << std::endl;
                    sensorLog << "time lfx lfy lfz ltx lty ltz rfx rfy rfz rtx rty rtz imu_r imu_p imu_y w_r w_y w_z a_x a_y a_z" << std::endl;
                }

                s_count++;
            }
            log_count++;

            if (switch_torqueLog)
            {
                torqueLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    torqueLog << (int)dc_.tc_shm_->elmo_torque[i] << " ";
                }
                torqueLog << std::endl;
            }

            if (switch_torqueCommandLog)
            {

                torqueCommandLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    torqueCommandLog << dc_.torque_command[i] << " ";
                }
                torqueCommandLog << std::endl;
            }

            if (switch_posLog)
            {
                posLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < MODEL_DOF_QVIRTUAL; i++)
                {
                    posLog << rd_gl_.q_virtual_[i] << " ";
                }
                posLog << std::endl;
            }

            if (switch_posDesiredLog)
            {

                posDesiredLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    posDesiredLog << rd_gl_.q_desired[i] << " ";
                }
                posDesiredLog << std::endl;
            }

            if (switch_velDesiredLog)
            {

                velDesiredLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    velDesiredLog << rd_gl_.q_dot_desired[i] << " ";
                }
                velDesiredLog << std::endl;
            }

            if (switch_velLog)
            {
                velLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
                {
                    velLog << q_dot_virtual_[i] << " ";
                }
                velLog << std::endl;
            }

            if (switch_torqueActualLog)
            {
                torqueActualLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < MODEL_DOF; i++)
                {
                    torqueActualLog << dc_.tc_shm_->torqueActual[i] << " ";
                }
                torqueActualLog << std::endl;
            }

            if (switch_maskLog)
            {

                maskLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";
                for (int i = 0; i < 10; i++)
                {
                    maskLog << std::setfill(' ') << std::setw(6) << (int)dc_.tc_shm_->e1_m[i] << " ";
                }
                for (int i = 0; i < 10; i++)
                {
                    maskLog << std::setfill(' ') << std::setw(6) << (int)dc_.tc_shm_->e2_m[i] << " ";
                }
                maskLog << std::endl;
            }

            if (switch_sensorLog)
            {
                sensorLog << std::fixed << std::setprecision(6) << local_control_time / 1000000.0 << " ";

                /*   for (int i = 0; i < 6; i++)
                   {
                       sensorLog << rd_gl_.LF_CF_FT(i) << " ";
                   }
                   for (int i = 0; i < 6; i++)
                   {
                       sensorLog << rd_gl_.RF_CF_FT(i) << " ";
                   }*/

                for (int i = 0; i < 6; i++)
                {
                    sensorLog << rd_gl_.LH_CF_FT(i) << " ";
                }
                for (int i = 0; i < 6; i++)
                {
                    sensorLog << rd_gl_.RH_CF_FT(i) << " ";
                }

                sensorLog << rd_gl_.roll << " " << rd_gl_.pitch << " " << rd_gl_.yaw << " ";
                sensorLog << rd_gl_.imu_ang_vel(0) << " " << rd_gl_.imu_ang_vel(1) << " " << rd_gl_.imu_ang_vel(2) << " ";
                sensorLog << rd_gl_.imu_lin_acc(0) << " " << rd_gl_.imu_lin_acc(1) << " " << rd_gl_.imu_lin_acc(2) << " ";
                sensorLog << std::endl;
            }

            if (switch_ecatStatusLog)
            {

                bool change = false;

                static int elmoStatus_before[MODEL_DOF];
                int elmoStatus_now[MODEL_DOF];

                std::copy(dc_.tc_shm_->ecat_status, dc_.tc_shm_->ecat_status + MODEL_DOF, elmoStatus_now);

                for (int i = 0; i < MODEL_DOF; i++)
                {
                    if (elmoStatus_now[i] != elmoStatus_before[i])
                        change = true;
                }

                if (change)
                {
                    ecatStatusLog << (float)rd_gl_.control_time_us_ / 1000000.0 << "\t ";
                    for (int i = 0; i < MODEL_DOF; i++)
                    {
                        ecatStatusLog << elmoStatus_now[i] << "  ";
                    }
                    ecatStatusLog << std::endl;
                }

                std::copy(elmoStatus_now, elmoStatus_now + MODEL_DOF, elmoStatus_before);
            }
        }
    }

    if (switch_torqueLog)
        torqueLog.close();
    if (switch_torqueCommandLog)
        torqueCommandLog.close();
    if (switch_torqueActualLog)
        torqueActualLog.close();
    if (switch_maskLog)
        maskLog.close();
    if (switch_ecatStatusLog)
        ecatStatusLog.close();
    if (switch_posLog)
        posLog.close();
    if (switch_posDesiredLog)
        posDesiredLog.close();
    if (switch_velDesiredLog)
        velDesiredLog.close();
    if (switch_velLog)
        velLog.close();
    if (switch_sensorLog)
        sensorLog.close();

    std::cout << "Logger : END!" << std::endl;
    return (void *)NULL;
}

void StateManager::SendCommand()
{
    timespec t_u10;

    t_u10.tv_nsec = 10000;
    t_u10.tv_sec = 0;
    static double torque_command[MODEL_DOF];
    while (dc_.t_c_)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }
    dc_.t_c_ = true;
    std::copy(dc_.torque_command, dc_.torque_command + MODEL_DOF, torque_command);
    static int rcv_c_count = dc_.control_command_count;

    dc_.t_c_ = false;

    if (dc_.tc_shm_->lower_disabled)
    {
        for (int i = 0; i < 12; i++)
        {
            torque_command[i] = 0.0;
        }
    }
    else
    {
        if (dc_.locklower)
        {
            for (int i = 0; i < 12; i++)
            {
                torque_command[i] = rd_gl_.pos_kp_v[i] * (dc_.qlock_des[i] - q_[i]) + rd_gl_.pos_kv_v[i] * (-q_dot_[i]);
            }
        }
    }

    static int rcv_c_count_before;
    static int warning_cnt = 0;
    if (rcv_c_count_before == rcv_c_count)
    {
        warning_cnt++;
    }

    if (warning_cnt > 0)
    {
        static int prob_cnt;
        prob_cnt = rcv_c_count;

        if (warning_cnt > 10)
        {
            if (prob_cnt != rcv_c_count)
            {
                std::cout << " STATE : Command not received for " << warning_cnt << "times " << std::endl;
                warning_cnt = 0;
            }
        }
        else if (prob_cnt != rcv_c_count)
        {
            warning_cnt = 0;
        }
    }

    rcv_c_count_before = rcv_c_count;

    const double maxTorque = _MAXTORQUE; // SYSTEM MAX TORQUE

    const double rTime1 = 4.0;
    const double rTime2 = 1.0;

    const double rat1 = 0.3;
    const double rat2 = 0.7;

    int maxTorqueCommand;

    float control_time_at_ = rd_gl_.control_time_;

    if (dc_.torqueOnSwitch)
    {
        dc_.rd_.positionControlSwitch = true;

        dc_.torqueOnSwitch = false;

        if (dc_.torqueOn)
        {
            std::cout << " STATE : Torque is already on " << std::endl;
        }
        else
        {
            std::cout << " STATE : Turning on ... " << std::endl;
            dc_.torqueOnTime = control_time_at_;
            dc_.torqueOn = true;
            dc_.torqueRisingSeq = true;

            if (dc_.avatarMode)
            {
                dc_.inityawSwitch = true;
                dc_.stateEstimateSwitch = true;
                rd_gl_.semode = true;

                rd_gl_.tc_avatar_switch = true;
            }
        }
    }
    if (dc_.torqueOffSwitch)
    {
        dc_.torqueOffSwitch = false;

        if (dc_.torqueOn)
        {
            std::cout << " STATE : Turning off ... " << std::endl;
            dc_.torqueOffTime = control_time_at_;
            dc_.toruqeDecreaseSeq = true;
        }
        else
        {
            std::cout << " STATE : Torque is already off" << std::endl;
        }
    }

    if (dc_.torqueOn)
    {
        if (dc_.torqueRisingSeq)
        {
            if (control_time_at_ <= dc_.torqueOnTime + rTime1)
            {
                torqueRatio = rat1 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOnTime) / rTime1, 0.0, 1.0);
            }
            if (control_time_at_ > dc_.torqueOnTime + rTime1 && control_time_at_ <= dc_.torqueOnTime + rTime1 + rTime2)
            {
                torqueRatio = rat1 + rat2 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOnTime - rTime1) / rTime2, 0.0, 1.0);
            }
            else if (control_time_at_ > dc_.torqueOnTime + rTime1 + rTime2)
            {
                std::cout << " STATE : Torque 100% ! " << std::endl;
                StatusPub("%f Torque 100%", control_time_);

                torqueRatio = 1.0;

                dc_.torqueRisingSeq = false;
            }

            maxTorqueCommand = maxTorque * torqueRatio;
        }
        else if (dc_.toruqeDecreaseSeq)
        {

            if (control_time_at_ <= dc_.torqueOffTime + rTime2)
            {
                torqueRatio = (1 - rat2 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOffTime) / rTime2, 0.0, 1.0));
            }
            if (control_time_at_ > dc_.torqueOffTime + rTime2 && control_time_at_ <= dc_.torqueOffTime + rTime2 + rTime1)
            {
                torqueRatio = (1 - rat2 - rat1 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOffTime - rTime2) / rTime1, 0.0, 1.0));
            }
            else if (control_time_at_ > dc_.torqueOffTime + rTime2 + rTime1)
            {
                dc_.toruqeDecreaseSeq = false;

                rd_gl_.tc_run = false;

                std::cout << " STATE : Torque 0% .. torque Off " << std::endl;
                StatusPub("%f Torque 0%", control_time_);
                torqueRatio = 0.0;
                dc_.torqueOn = false;
            }

            maxTorqueCommand = maxTorque * torqueRatio;
        }
        else
        {
            torqueRatio = 1.0;
            maxTorqueCommand = (int)maxTorque;
        }
    }
    else
    {
        torqueRatio = 0.0;
        maxTorqueCommand = 0;
    }

    if (dc_.E1Switch) // Emergency stop
    {
        if (dc_.E1Status)
        {
            dc_.E1Status = false;
        }
        else
        {
            std::cout << "E1 : STOP" << std::endl;
            rd_.q_desired = rd_gl_.q_;
            rd_.q_dot_desired.setZero();
            dc_.E1Status = true;
            rd_gl_.tc_run = false;
            rd_gl_.pc_mode = false;
        }

        dc_.E1Switch = false;
    }
    if (dc_.E2Switch) // Emergency damping
    {
        if (dc_.E2Status)
        {
            dc_.E2Status = false;
        }
        else
        {
            dc_.E2Status = true;
            rd_gl_.tc_run = false;
            rd_gl_.pc_mode = false;

            // Damping mode = true!
        }

        dc_.E2Switch = false;
    }
    if (dc_.emergencySwitch)
    {
        dc_.emergencyStatus = true; //
        rd_gl_.tc_run = false;
        rd_gl_.pc_mode = false;
    }

    if (dc_.E1Status)
    {
        for (int i = 0; i < MODEL_DOF; i++)
            torque_command[i] = rd_gl_.pos_kp_v[i] * (rd_.q_desired(i) - rd_gl_.q_(i)) + rd_gl_.pos_kv_v[i] * (rd_.q_dot_desired(i) - rd_gl_.q_dot_(i));
    }

    if (dc_.E2Status)
    {
        for (int i = 0; i < MODEL_DOF; i++)
            torque_command[i] = rd_gl_.pos_kv_v[i] * (-rd_.q_dot_(i));
    }

    if (dc_.emergencyStatus)
    {
        for (int i = 0; i < MODEL_DOF; i++)
            torque_command[i] = 0.0;
    }

    dc_.tc_shm_->commanding = true;

    // UpperBody
    while (dc_.tc_shm_->cmd_upper)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }
    dc_.tc_shm_->cmd_upper = true;
    // std::fill(dc_.tc_shm_->commandMode, dc_.tc_shm_->commandMode + MODEL_DOF, 1);
    std::copy(torque_command + 15, torque_command + MODEL_DOF, dc_.tc_shm_->torqueCommand + 15);
    dc_.tc_shm_->maxTorque = maxTorqueCommand;
    // static int cCount = 0;
    cCount++;
    dc_.tc_shm_->commandCount.store(cCount);

    dc_.tc_shm_->cmd_upper = false;
    // LowerBody

    while (dc_.tc_shm_->cmd_lower)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }

    dc_.tc_shm_->cmd_lower = true;
    std::copy(torque_command, torque_command + 15, dc_.tc_shm_->torqueCommand);
    dc_.tc_shm_->cmd_lower = false;

    dc_.tc_shm_->commanding.store(false);

    // dc_.tc_shm_->commandCount++;
    // dc_.tc_shm_->commanding = false;

    // static timespec ts_before;
    // timespec ts_now;
    // clock_gettime(CLOCK_MONOTONIC, &ts_now);

    // int _latency = ts_now.tv_nsec - ts_before.tv_nsec;

    // if (_latency < 0)
    //     _latency += 1000000000;

    // static int tick = 0;
    // tick++;

    // static int lat_all = 0;
    // static int lat_max = 0;
    // static int lat_min = 100000000;

    // lat_all += _latency;

    // if (lat_max < _latency)
    //     lat_max = _latency;

    // if (lat_min > _latency)
    //     lat_min = _latency;

    // if (tick == 2000)
    // {

    //     std::cout << control_time_ << "    " << lat_all / tick << " lat min : " << lat_min << " lat max : " << lat_max << std::endl;
    //     tick = 0;
    //     lat_all = 0;
    //     lat_max = 0;
    //     lat_min = 1000000000;
    // }

    // ts_before = ts_now;
}

void StateManager::InitYaw()
{
    q_virtual_local_yaw_initialized = q_virtual_local_;

    tf2::Quaternion q(q_virtual_local_(3), q_virtual_local_(4), q_virtual_local_(5), q_virtual_local_(MODEL_DOF_VIRTUAL));
    tf2::Matrix3x3 m(q);
    m.getRPY(rd_.roll, rd_.pitch, rd_.yaw);

    if (dc_.inityawSwitch)
    {
        std::cout << " STATE : Yaw Initialized : " << rd_.yaw << std::endl;
        rd_gl_.yaw_init = rd_.yaw;
        dc_.inityawSwitch = false;
    }

    // const tf2Scalar& r_,p_,y_;

    tf2::Quaternion q_mod;
    rd_.yaw = rd_.yaw - rd_gl_.yaw_init;

    q_mod.setRPY(rd_.roll, rd_.pitch, rd_.yaw);
    // tf2::Quaternion q_rot;
    // q_rot.setRPY(0, 0, -yaw_init);
    // q = q * q_rot;

    q_virtual_local_(3) = q_mod.getX();
    q_virtual_local_(4) = q_mod.getY();
    q_virtual_local_(5) = q_mod.getZ();
    q_virtual_local_(MODEL_DOF_VIRTUAL) = q_mod.getW();

    // q_virtual_local_ = q_virtual_local_yaw_initialized;
}

void StateManager::GetJointData()
{
    while (dc_.tc_shm_->statusWriting.load(std::memory_order_acquire))
    {
        usleep(10);
        if (dc_.tc_shm_->shutdown)
            break;
    }

    memcpy(q_a_, dc_.tc_shm_->pos, sizeof(float) * MODEL_DOF);
    memcpy(q_dot_a_, dc_.tc_shm_->vel, sizeof(float) * MODEL_DOF);
    memcpy(torqueActual_a_, dc_.tc_shm_->torqueActual, sizeof(float) * MODEL_DOF);
    memcpy(q_ext_a, dc_.tc_shm_->posExt, sizeof(float) * MODEL_DOF);

    // q_dot_a : actual qdot from elmo with float

    // q_dot_ =

    // memecy(q)

    q_ = Map<VectorQf>(q_a_, MODEL_DOF).cast<double>();

    if (qdot_estimation_switch)
    {
        calculateJointVelMlpInput();

        calculateJointVelMlpOutput();
        q_dot_ = nn_estimated_q_dot_fast_;
    }
    else
    {
        q_dot_ = Map<VectorQf>(q_dot_a_, MODEL_DOF).cast<double>();
    }

    q_ext_ = Map<VectorQf>(q_ext_a, MODEL_DOF).cast<double>();

    q_virtual_local_.segment(6, MODEL_DOF) = q_;
    q_dot_virtual_local_.segment(6, MODEL_DOF) = q_dot_;

    torque_elmo_ = Map<VectorQf>(torqueActual_a_, MODEL_DOF).cast<double>();

    if (dc_.useSimVirtual)
    {
        q_virtual_local_(0) = dc_.tc_shm_->pos_virtual[0];
        q_virtual_local_(1) = dc_.tc_shm_->pos_virtual[1];
        q_virtual_local_(2) = dc_.tc_shm_->pos_virtual[2];

        q_dot_virtual_local_(0) = dc_.tc_shm_->vel_virtual[0];
        q_dot_virtual_local_(1) = dc_.tc_shm_->vel_virtual[1];
        q_dot_virtual_local_(2) = dc_.tc_shm_->vel_virtual[2];
    }
    else
    {
        q_virtual_local_(0) = 0.0; // dc_.tc_shm_->pos_virtual[0];
        q_virtual_local_(1) = 0.0; // dc_.tc_shm_->pos_virtual[1];
        q_virtual_local_(2) = 0.0; // dc_.tc_shm_->pos_virtual[2];

        q_dot_virtual_local_(0) = 0.0;
        q_dot_virtual_local_(1) = 0.0;
        q_dot_virtual_local_(2) = 0.0;
    }

    if (dc_.tc_shm_->imuWriting)
    {
        usleep(10);
    }

    q_dot_virtual_local_(3) = dc_.tc_shm_->vel_virtual[3];
    q_dot_virtual_local_(4) = dc_.tc_shm_->vel_virtual[4];
    q_dot_virtual_local_(5) = dc_.tc_shm_->vel_virtual[5];

    q_virtual_local_(3) = dc_.tc_shm_->pos_virtual[3];
    q_virtual_local_(4) = dc_.tc_shm_->pos_virtual[4];
    q_virtual_local_(5) = dc_.tc_shm_->pos_virtual[5];
    q_virtual_local_(MODEL_DOF_VIRTUAL) = dc_.tc_shm_->pos_virtual[6];

    // memcpy(joint_state_, dc_.tc_shm_->status, sizeof(int) * MODEL_DOF);
    memcpy(state_elmo_, dc_.tc_shm_->ecat_status, sizeof(int8_t) * MODEL_DOF);
    memcpy(state_safety_, dc_.tc_shm_->safety_status, sizeof(int8_t) * MODEL_DOF);
    memcpy(state_zp_, dc_.tc_shm_->zp_status, sizeof(int8_t) * MODEL_DOF);

    // Position Hold On Safety
    //  for (int i = 0; i < MODEL_DOF; i++)
    //  {
    //      if (state_safety_[i] != state_safety_before_[i])
    //      {
    //          if (state_safety_[i] != 0)
    //          {
    //              dc_.positionControlSwitch = true;
    //              std::cout << "Safety Activated ! To Position Hold" << std::endl;
    //          }
    //      }
    //  }

    // RF_CF_FT.setZ

    // dc_.tc_shm_->pos
}

void StateManager::GetSensorData()
{
    rd_.imu_lin_acc(0) = dc_.tc_shm_->imu_acc[0];
    rd_.imu_lin_acc(1) = dc_.tc_shm_->imu_acc[1];
    rd_.imu_lin_acc(2) = dc_.tc_shm_->imu_acc[2];

    for (int i = 0; i < 6; i++)
    {
        LF_FT(i) = dc_.tc_shm_->ftSensor[i];
        RF_FT(i) = dc_.tc_shm_->ftSensor[i + 6];
    }

    for (int i = 0; i < 6; i++)
    {
        LH_FT(i) = dc_.tc_shm_->ftSensor2[i];
        RH_FT(i) = dc_.tc_shm_->ftSensor2[i + 6];
    }

    static Vector6d LF_FT_LPF = LF_FT;
    static Vector6d RF_FT_LPF = RF_FT;

    static Vector6d LH_FT_LPF = LH_FT;
    static Vector6d RH_FT_LPF = RH_FT;

    for (int i = 0; i < 6; i++)
    {
        LF_FT_LPF(i) = DyrosMath::lpf(LF_FT(i), LF_FT_LPF(i), 2000, 60);
        RF_FT_LPF(i) = DyrosMath::lpf(RF_FT(i), RF_FT_LPF(i), 2000, 60);
    }

    for (int i = 0; i < 6; i++)
    {
        LH_FT_LPF(i) = LH_FT(i); // DyrosMath::lpf(LH_FT(i), LH_FT_LPF(i), 2000, 10);
        RH_FT_LPF(i) = RH_FT(i); // DyrosMath::lpf(RH_FT(i), RH_FT_LPF(i), 2000, 10);
    }

    double foot_plate_mass = 2.326;
    // double hand_plate_mass = 0.8;

    Matrix6d adt;
    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-(link_local_[Right_Foot].contact_point - link_local_[Right_Foot].sensor_point)) * Matrix3d::Identity();
    Matrix6d rotrf;
    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = link_local_[Right_Foot].rotm;
    rotrf.block(3, 3, 3, 3) = link_local_[Right_Foot].rotm;
    Vector3d RF_com(-0.0162, 0.00008, -0.1209);

    Vector3d com2cp = link_local_[Right_Foot].sensor_point - RF_com;

    Matrix6d adt2;
    adt2.setIdentity();
    adt2.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();

    Vector6d Wrench_foot_plate;
    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = foot_plate_mass * GRAVITY;

    RF_CF_FT = rotrf * adt * RF_FT_LPF - adt2 * Wrench_foot_plate;
    // rd_gl_.ee_[1].contact_force_ft = RF_CF_FT;

    // RF_CF_FT_local = rotrf.inverse() * RF_CF_FT;

    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-(link_local_[Left_Foot].contact_point - link_local_[Left_Foot].sensor_point)) * Matrix3d::Identity();

    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = link_local_[Left_Foot].rotm;
    rotrf.block(3, 3, 3, 3) = link_local_[Left_Foot].rotm;

    Vector3d LF_com(-0.0162, -0.00008, -0.1209);

    com2cp = link_local_[Left_Foot].contact_point - LF_com;

    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = foot_plate_mass * GRAVITY;

    LF_CF_FT = rotrf * adt * LF_FT_LPF - adt2 * Wrench_foot_plate;

    Vector3d Hand_com(0.0, 0.0, -0.1542);

    Matrix3d adt3;
    adt3 << 1., 0., 0., 0., -1., 0., 0., 0., -1.;
    Matrix3d adt4;
    adt4 << -1., 0., 0., 0., 1., 0., 0., 0., -1.; //<< 0., 1., 0., 1., 0., 0., 0., 0., -1.;

    Matrix3d pelv_imu_yaw;
    pelv_imu_yaw = DyrosMath::rotateWithZ(-rd_.yaw);

    Matrix3d rotrh, rotlh;
    rotrh = link_local_[Right_Hand].rotm;
    rotlh = link_local_[Left_Hand].rotm;

    /*if (handft_init <= 1000)
    {
        adt2_temp.setZero();
        adt2_temp = (rotrh * adt3).inverse();

        adt1_temp.setZero();
        adt1_temp = (rotlh * adt4).inverse();
        ft_calibd_r = RH_FT_LPF;
        ft_calibd_l = LH_FT_LPF;
        handft_init++;
    }
     else if(sqrt((link_local_[Right_Hand].v).transpose()*(link_local_[Right_Hand].v)) < 0.1 && sqrt((link_local_[Right_Hand].w).transpose()*(link_local_[Right_Hand].w)) < 0.2 && hand_grasp_r == false)
     {
         handft_reinit_r++;
         if(handft_reinit_r  == 1000)
         {
             adt2_temp.setZero();
             adt2_temp = (rotrh * adt3).inverse();
             ft_calibd_r = RH_FT_LPF;
             handft_reinit_r = 0;
         }
     }
     else
     {
        handft_reinit_r = 0;
     }

     if(sqrt((link_local_[Left_Hand].v).transpose()*(link_local_[Left_Hand].v)) < 0.1 && sqrt((link_local_[Left_Hand].w).transpose()*(link_local_[Left_Hand].w)) < 0.2 && hand_grasp_l == false)
     {
         handft_reinit_l++;
         if(handft_reinit_l  == 1000)
         {
             adt1_temp.setZero();
             adt1_temp = (rotlh * adt4).inverse();
             ft_calibd_l = LH_FT_LPF;
             handft_reinit_l = 0;
         }
     }
     else
     {
         handft_reinit_l = 0;
     }*/

    if (dc_.fthandcalibreset)
    {
        LH_CALIB.setZero();
        RH_CALIB.setZero();
        handFtCalib_mode = 0;
        dc_.fthandcalibreset = false;
        hand_calib_init = false;
    }

    if (dc_.handft_calib_signal_)
    {
        if (handFtCalib_mode == 0)
        {
            LH_CALIB.setZero();
            RH_CALIB.setZero();
        }

        LH_CALIB.block<1, 3>(handFtCalib_mode, 0) = LH_FT.segment<3>(0).transpose();
        RH_CALIB.block<1, 3>(handFtCalib_mode, 0) = RH_FT.segment<3>(0).transpose();
        handFtCalib_mode++;
        dc_.handft_calib_signal_ = false;
        std::cout << RH_CALIB << std::endl;
        std::cout << "hand FT Calib mode : " << handFtCalib_mode << std::endl;

        if (handFtCalib_mode == 6)
            handFtCalib_mode = 0;
    }

    Eigen::Vector3d LH_ori, LH_ori_ready, RH_ori, RH_ori_ready;

    LH_ori = DyrosMath::rot2Euler(link_local_[Pelvis].rotm.inverse() * link_local_[Left_Hand].rotm);
    RH_ori = DyrosMath::rot2Euler(link_local_[Pelvis].rotm.inverse() * link_local_[Right_Hand].rotm);

    LH_ori_ready << -2.44706, 0.531159, 0.0138229;
    RH_ori_ready << 2.44555, 0.527625, -0.0145275;

    if (dc_.fthandzeroSwtich && hand_calib_init == false)
    {
        std::cout << "STATUS : FT HAND BIAS INIT" << std::endl;
        adt2_temp.setZero();
        adt2_temp = (rotrh * adt3).inverse();

        adt1_temp.setZero();
        adt1_temp = (rotlh * adt4).inverse();
        ft_calibd_r = RH_FT_LPF;
        ft_calibd_r(0) = RH_CALIB.block<6, 1>(0, 0).sum() / 6.0;
        ft_calibd_r(1) = RH_CALIB.block<6, 1>(0, 1).sum() / 6.0;
        ft_calibd_r(2) = RH_CALIB.block<6, 1>(0, 2).sum() / 6.0;
        ft_calibd_l = LH_FT_LPF;
        ft_calibd_l(0) = LH_CALIB.block<6, 1>(0, 0).sum() / 6.0;
        ft_calibd_l(1) = LH_CALIB.block<6, 1>(0, 1).sum() / 6.0;
        ft_calibd_l(2) = LH_CALIB.block<6, 1>(0, 2).sum() / 6.0;
        dc_.fthandzeroSwtich = false;

        ft_calibd_rt = -ft_calibd_r + RH_FT_LPF;
        ft_calibd_lt = -ft_calibd_l - LH_FT_LPF;

        Eigen::Vector6d l_bias, r_bias;
        l_bias(0) = sqrt((ft_calibd_l(0) - LH_CALIB(0, 0)) * (ft_calibd_l(0) - LH_CALIB(0, 0)) + (ft_calibd_l(1) - LH_CALIB(0, 1)) * (ft_calibd_l(1) - LH_CALIB(0, 1)) + (ft_calibd_l(2) - LH_CALIB(0, 2)) * (ft_calibd_l(2) - LH_CALIB(0, 2)));
        l_bias(1) = sqrt((ft_calibd_l(0) - LH_CALIB(1, 0)) * (ft_calibd_l(0) - LH_CALIB(1, 0)) + (ft_calibd_l(1) - LH_CALIB(1, 1)) * (ft_calibd_l(1) - LH_CALIB(1, 1)) + (ft_calibd_l(2) - LH_CALIB(1, 2)) * (ft_calibd_l(2) - LH_CALIB(1, 2)));
        l_bias(2) = sqrt((ft_calibd_l(0) - LH_CALIB(2, 0)) * (ft_calibd_l(0) - LH_CALIB(2, 0)) + (ft_calibd_l(1) - LH_CALIB(2, 1)) * (ft_calibd_l(1) - LH_CALIB(2, 1)) + (ft_calibd_l(2) - LH_CALIB(2, 2)) * (ft_calibd_l(2) - LH_CALIB(2, 2)));
        l_bias(3) = sqrt((ft_calibd_l(0) - LH_CALIB(3, 0)) * (ft_calibd_l(0) - LH_CALIB(3, 0)) + (ft_calibd_l(1) - LH_CALIB(3, 1)) * (ft_calibd_l(1) - LH_CALIB(3, 1)) + (ft_calibd_l(2) - LH_CALIB(3, 2)) * (ft_calibd_l(2) - LH_CALIB(3, 2)));
        l_bias(4) = sqrt((ft_calibd_l(0) - LH_CALIB(4, 0)) * (ft_calibd_l(0) - LH_CALIB(4, 0)) + (ft_calibd_l(1) - LH_CALIB(4, 1)) * (ft_calibd_l(1) - LH_CALIB(4, 1)) + (ft_calibd_l(2) - LH_CALIB(4, 2)) * (ft_calibd_l(2) - LH_CALIB(4, 2)));
        l_bias(5) = sqrt((ft_calibd_l(0) - LH_CALIB(5, 0)) * (ft_calibd_l(0) - LH_CALIB(5, 0)) + (ft_calibd_l(1) - LH_CALIB(5, 1)) * (ft_calibd_l(1) - LH_CALIB(5, 1)) + (ft_calibd_l(2) - LH_CALIB(5, 2)) * (ft_calibd_l(2) - LH_CALIB(5, 2)));

        r_bias(0) = sqrt((ft_calibd_r(0) - RH_CALIB(0, 0)) * (ft_calibd_r(0) - RH_CALIB(0, 0)) + (ft_calibd_r(1) - RH_CALIB(0, 1)) * (ft_calibd_r(1) - RH_CALIB(0, 1)) + (ft_calibd_r(2) - RH_CALIB(0, 2)) * (ft_calibd_r(2) - RH_CALIB(0, 2)));
        r_bias(1) = sqrt((ft_calibd_r(0) - RH_CALIB(1, 0)) * (ft_calibd_r(0) - RH_CALIB(1, 0)) + (ft_calibd_r(1) - RH_CALIB(1, 1)) * (ft_calibd_r(1) - RH_CALIB(1, 1)) + (ft_calibd_r(2) - RH_CALIB(1, 2)) * (ft_calibd_r(2) - RH_CALIB(1, 2)));
        r_bias(2) = sqrt((ft_calibd_r(0) - RH_CALIB(2, 0)) * (ft_calibd_r(0) - RH_CALIB(2, 0)) + (ft_calibd_r(1) - RH_CALIB(2, 1)) * (ft_calibd_r(1) - RH_CALIB(2, 1)) + (ft_calibd_r(2) - RH_CALIB(2, 2)) * (ft_calibd_r(2) - RH_CALIB(2, 2)));
        r_bias(3) = sqrt((ft_calibd_r(0) - RH_CALIB(3, 0)) * (ft_calibd_r(0) - RH_CALIB(3, 0)) + (ft_calibd_r(1) - RH_CALIB(3, 1)) * (ft_calibd_r(1) - RH_CALIB(3, 1)) + (ft_calibd_r(2) - RH_CALIB(3, 2)) * (ft_calibd_r(2) - RH_CALIB(3, 2)));
        r_bias(4) = sqrt((ft_calibd_r(0) - RH_CALIB(4, 0)) * (ft_calibd_r(0) - RH_CALIB(4, 0)) + (ft_calibd_r(1) - RH_CALIB(4, 1)) * (ft_calibd_r(1) - RH_CALIB(4, 1)) + (ft_calibd_r(2) - RH_CALIB(4, 2)) * (ft_calibd_r(2) - RH_CALIB(4, 2)));
        r_bias(5) = sqrt((ft_calibd_r(0) - RH_CALIB(5, 0)) * (ft_calibd_r(0) - RH_CALIB(5, 0)) + (ft_calibd_r(1) - RH_CALIB(5, 1)) * (ft_calibd_r(1) - RH_CALIB(5, 1)) + (ft_calibd_r(2) - RH_CALIB(5, 2)) * (ft_calibd_r(2) - RH_CALIB(5, 2)));

        hand_plate_mass_l = l_bias.sum() / 6.0;
        hand_plate_mass_r = r_bias.sum() / 6.0;

        hand_calib_init = true;

        std::cout << "hand_plate_mass_l : " << hand_plate_mass_l << "hand_plate_mass_r : " << hand_plate_mass_r << std::endl;
    }
    else if (dc_.fthandzeroSwtich == true && hand_calib_init == true)
    {
        Eigen::Vector3d L_hand_Ready, R_hand_Ready, L_hand_Ready_temp, R_hand_Ready_temp;
        L_hand_Ready << -4.5326825, 4.74047321, -6.18765972;
        R_hand_Ready << 4.90852058, 5.48995543, -7.030894;

        R_hand_Ready_temp = RH_FT_LPF.segment<3>(0) - ft_calibd_r.segment<3>(0) - R_hand_Ready;
        L_hand_Ready_temp = LH_FT_LPF.segment<3>(0) - ft_calibd_l.segment<3>(0) - L_hand_Ready;

        ft_calibd_r.segment<3>(0) = ft_calibd_r.segment<3>(0) + R_hand_Ready_temp;
        ft_calibd_l.segment<3>(0) = ft_calibd_l.segment<3>(0) + L_hand_Ready_temp;

        //\tick_ft++;
        // if (tick_ft % 3000 == 0)
        std::cout << "STATUS : FT HAND BIAS "
                  << "R_hand_Ready_temp : " << RH_ori.transpose() << " L_hand_Ready_temp : " << LH_ori.transpose() << std::endl;

        dc_.fthandzeroSwtich = false;
    }

    Vector3d Hand_FT_plate_z;
    Hand_FT_plate_z.setZero();
    Hand_FT_plate_z(2) = -hand_plate_mass_l;
    LH_CF_FT.segment<3>(0) = rotlh * adt4 * (LH_FT_LPF.segment<3>(0) - ft_calibd_l.segment<3>(0) /*+ adt1_temp * Hand_FT_plate_z*/) - Hand_FT_plate_z;

    Hand_FT_plate_z.setZero();
    Hand_FT_plate_z(2) = -hand_plate_mass_r;
    RH_CF_FT.segment<3>(0) = rotrh * adt3 * (RH_FT_LPF.segment<3>(0) - ft_calibd_r.segment<3>(0) /*+ adt2_temp * Hand_FT_plate_z*/) - Hand_FT_plate_z;

    RH_CF_FT.segment<3>(0) = pelv_imu_yaw * RH_CF_FT.segment<3>(0);
    LH_CF_FT.segment<3>(0) = pelv_imu_yaw * LH_CF_FT.segment<3>(0);
    // RH_CF_FT = RH_FT;
    // LH_CF_FT = LH_FT;
    // printf("hand FT : %6.3f %6.3f %6.3f %6.3f \n", LH_CF_FT(0), LH_CF_FT(1), LH_CF_FT(2), RH_CF_FT(2));
    // printf("hand FT : %6.3f %6.3f \n", LH_FT(2), RH_FT(2));
    // if(handft_init %200 == 0)
    // {

    // }
}

void StateManager::ConnectSim()
{
}

void StateManager::GetSimData()
{
    ros::spinOnce();
}

void StateManager::MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2)
{
    dc_.tc_shm_->t_cnt = currentCount;

    lat = nanoseconds1;
    total1 += lat;
    lavg = total1 / currentCount;
    if (lmax < lat)
    {
        lmax = lat;
    }
    if (lmin > lat)
    {
        lmin = lat;
    }
    // int sdev = (sat - savg)
    total_dev1 += sqrt(((lat - lavg) * (lat - lavg)));
    ldev = total_dev1 / currentCount;

    dc_.tc_shm_->lat_avg = lavg;
    dc_.tc_shm_->lat_max = lmax;
    dc_.tc_shm_->lat_min = lmin;
    dc_.tc_shm_->lat_dev = ldev;

    sat = nanoseconds2;
    total2 += sat;
    savg = total2 / currentCount;
    if (smax < sat)
    {
        smax = sat;
    }
    if (smin > sat)
    {
        smin = sat;
    }
    // int sdev = (sat - savg)
    total_dev2 += sqrt(((sat - savg) * (sat - savg)));
    sdev = total_dev2 / currentCount;

    dc_.tc_shm_->send_avg = savg;
    dc_.tc_shm_->send_max = smax;
    dc_.tc_shm_->send_min = smin;
    dc_.tc_shm_->send_dev = sdev;
}

void StateManager::StoreState(RobotData &rd_dst)
{
    rd_dst.mtx.lock();

    memcpy(&rd_dst.model_, &model_global_, sizeof(RigidBodyDynamics::Model));

    for (int i = 0; i < (LINK_NUMBER + 1); i++)
    {
        memcpy(&rd_dst.link_[i].jac, &link_[i].jac, sizeof(Matrix6Vf));
        memcpy(&rd_dst.link_[i].jac_com, &link_[i].jac_com, sizeof(Matrix6Vf));

        memcpy(&rd_dst.link_[i].xpos, &link_[i].xpos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].xipos, &link_[i].xipos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].rotm, &link_[i].rotm, sizeof(Matrix3d));
        memcpy(&rd_dst.link_[i].v, &link_[i].v, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].vi, &link_[i].vi, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].w, &link_[i].w, sizeof(Vector3d));

        // xpos xipos rotm v w
    }

    memcpy(&rd_dst.A_, &A_, sizeof(MatrixVVd));
    memcpy(&rd_dst.A_inv_, &A_inv_, sizeof(MatrixVVd));
    memcpy(&rd_dst.Motor_inertia, &Motor_inertia, sizeof(MatrixVVd));
    memcpy(&rd_dst.Motor_inertia_inverse, &Motor_inertia_inverse, sizeof(MatrixVVd));
    memcpy(&rd_dst.q_, &q_, sizeof(VectorQd));
    memcpy(&rd_dst.q_dot_, &q_dot_, sizeof(VectorQd));
    memcpy(&rd_dst.q_ext_, &q_ext_, sizeof(VectorQd));
    memcpy(&rd_dst.q_virtual_, &q_virtual_, sizeof(VectorQVQd));
    memcpy(&rd_dst.q_dot_virtual_, &q_dot_virtual_, sizeof(VectorVQd));
    memcpy(&rd_dst.q_ddot_virtual_, &q_ddot_virtual_, sizeof(VectorVQd));
    memcpy(&rd_dst.torque_elmo_, &torque_elmo_, sizeof(VectorQd));

    rd_dst.CMM = rd_.CMM;

    rd_dst.roll = rd_.roll;
    rd_dst.pitch = rd_.pitch;
    rd_dst.yaw = rd_.yaw;

    if (!rd_dst.firstCalc)
    {

        memcpy(&rd_dst.link_, link_, (LINK_NUMBER + 1) * sizeof(LinkData));

        rd_dst.firstCalc = true;
    }

    rd_dst.control_time_ = control_time_;
    rd_dst.control_time_us_ = control_time_ * 1000000;

    rd_dst.tp_state_ = rd_.tp_state_;

    rd_dst.LF_FT = LF_FT;
    rd_dst.RF_FT = RF_FT;

    rd_dst.LF_CF_FT = LF_CF_FT;
    rd_dst.RF_CF_FT = RF_CF_FT;

    rd_dst.LH_FT = LH_FT;
    rd_dst.RH_FT = RH_FT;

    rd_dst.LH_CF_FT = LH_CF_FT;
    rd_dst.RH_CF_FT = RH_CF_FT;

    mb();

    dc_.triggerThread1 = true;

    mb();

    rd_dst.mtx.unlock();
}

void StateManager::UpdateKinematics_local(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    // ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
     * 0 ~ 2 : XYZ cartesian coordinates
     * 3 ~ 5 : XYZ Quaternion
     * 6 ~ MODEL_DOF + 5 : joint position
     * model dof + 6 ( last component of q_virtual) : w of Quaternion
     * */

    // local kinematics update sequence

    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);

    link_p[Pelvis].UpdatePosition(model_l, q_virtual_f);
    link_p[Right_Foot].UpdatePosition(model_l, q_virtual_f);
    link_p[Left_Foot].UpdatePosition(model_l, q_virtual_f);
    link_p[Right_Hand].UpdatePosition(model_l, q_virtual_f);
    link_p[Left_Hand].UpdatePosition(model_l, q_virtual_f);

    link_p[Pelvis].UpdateJacobian(model_l, q_virtual_f);
    link_p[Right_Foot].UpdateJacobian(model_l, q_virtual_f);
    link_p[Left_Foot].UpdateJacobian(model_l, q_virtual_f);
    link_p[Right_Hand].UpdateJacobian(model_l, q_virtual_f);
    link_p[Left_Hand].UpdateJacobian(model_l, q_virtual_f);

    link_p[Pelvis].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Right_Foot].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Left_Foot].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Right_Hand].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
    link_p[Left_Hand].UpdateVW(model_l, q_virtual_f, q_dot_virtual_f);
}

void StateManager::UpdateKinematics(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    // ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
     * 0 ~ 2 : XYZ cartesian coordinates
     * 3 ~ 5 : XYZ Quaternion
     * 6 ~ MODEL_DOF + 5 : joint position
     * model dof + 6 ( last component of q_virtual) : w of Quaternion
     * */

    // Update Kinematics : Total 127 us
    static int uk_loop = 0;

    auto t1 = std::chrono::steady_clock::now();

    // sector 1 Start : rbdl update
    A_temp_.setZero();
    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_l, q_virtual_f, A_temp_, false);
    A_ = A_temp_;
    // sector 1 End : 30 us

    auto t2 = std::chrono::steady_clock::now();

    // sector 2 start : link pos/vel/jac update
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_p[i].UpdatePosition(model_l, q_virtual_f);
        link_p[i].UpdateJacobian(model_l, q_virtual_f, q_dot_virtual_f);
    }
    // sector 2 end : 57 us

    auto t3 = std::chrono::steady_clock::now();
    // sector 3 start : mass matrix inverse
    A_inv_ = A_.inverse();
    // A_inv_ = A_.llt().solve(Eigen::MatrixVVd::Identity());
    // sector 3 end : 39 us

    auto t4 = std::chrono::steady_clock::now();
    // sector 4 start : com calculation
    Eigen::Matrix3Vf jacobian_com;
    Eigen::Matrix3Vf jacobian_com_r;
    jacobian_com_r.setZero();

    jacobian_com.setZero();

    for (int i = 0; i < LINK_NUMBER; i++)
    {
        jacobian_com += link_p[i].jac_com.topRows(3) * link_p[i].mass / total_mass_;

        jacobian_com_r += link_p[i].jac_com.bottomRows(3) * link_p[i].mass / total_mass_;
    }
    link_p[COM_id].mass = total_mass_;
    link_p[COM_id].xpos.setZero();
    for (int i = 0; i < LINK_NUMBER; i++)
        link_p[COM_id].xpos += link_p[i].xipos * link_p[i].mass / total_mass_;

    link_p[COM_id].xipos = link_p[COM_id].xpos;
    // RigidBodyDynamics::CalcCenterOfMass(model_l, )

    // RigidBodyDynamics::UpdateKinematicsCustom()

    link_p[COM_id].v = jacobian_com.cast<double>() * q_dot_virtual_f;
    link_p[COM_id].vi = link_p[COM_id].v;
    link_p[COM_id].w = link_p[Pelvis].w;
    link_p[COM_id].rotm = link_p[Pelvis].rotm;
    link_p[COM_id].jac.setZero(6, MODEL_DOF_VIRTUAL);
    // link_p[COM_id].jac.block(0, 0, 2, MODEL_DOF + 6) = jacobian_com.block(0, 0, 2, MODEL_DOF + 6) / com_.mass;
    // link_p[COM_id].jac.block(2, 0, 4, MODEL_DOF + 6) = link_p[Pelvis].jac.block(2, 0, 4, MODEL_DOF + 6);
    link_p[COM_id].jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com.block(0, 0, 3, MODEL_DOF_VIRTUAL);
    link_p[COM_id].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = link_p[Pelvis].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);

    link_p[COM_id].jac_com.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com;

    link_p[COM_id].jac_com.block(3, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com_r;

    Eigen::Matrix3d i_com_;
    i_com_.setZero();

    Eigen::Vector3d x_com_;

    Eigen::Matrix3d c_product_;

    for (int i = 0; i < LINK_NUMBER; i++)
    {
        x_com_ = link_p[i].xpos - link_p[COM_id].xpos;

        c_product_.setZero();

        c_product_.block(0, 0, 3, 1) = x_com_ * x_com_(0);
        c_product_.block(0, 1, 3, 1) = x_com_ * x_com_(1);
        c_product_.block(0, 2, 3, 1) = x_com_ * x_com_(2);

        i_com_ += link_p[i].rotm * link_p[i].inertia * link_[i].rotm.transpose() + link_p[i].mass * (Eigen::Matrix3d::Identity() * (x_com_(0) * x_com_(0) + x_com_(1) * x_com_(1) + x_com_(2) * x_com_(2)) - c_product_);
    }

    link_[COM_id].inertia = i_com_;

    auto t5 = std::chrono::steady_clock::now();
    uk_loop++;

    static long d1 = 0;
    static long d2 = 0;
    static long d3 = 0;
    static long d4 = 0;

    static int max[4] = {0, 0, 0, 0};
    static int min[4] = {1000, 1000, 1000, 1000};
    static int dur[4];

    dur[0] = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    dur[1] = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    dur[2] = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
    dur[3] = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

    d1 += dur[0];
    d2 += dur[1];
    d3 += dur[2];
    d4 += dur[3];

    for (int i = 0; i < 4; i++)
    {
        if (max[i] < dur[i])
        {
            max[i] = dur[i];
        }

        if (min[i] > dur[i])
        {
            min[i] = dur[i];
        }
    }

    if (uk_loop >= 2000)
    {
        uk_loop = 0;

        // std::cout << "d1 : " << d1 / 2000 << " min : " << min[0] << " max : " << max[0] << " | d2 : " << d2 / 2000 << " min : " << min[1] << " max : " << max[1] << " | d3 : " << d3 / 2000 << " min : " << min[2] << " max : " << max[2] << " | d4 : " << d4 / 2000 << " min : " << min[3] << " max : " << max[3] << std::endl;

        d1 = 0;
        d2 = 0;
        d3 = 0;
        d4 = 0;
    }
    // link_p[COM_id].xpos(2) = link_p[Pelvis].xpos(2);
    //  sector 4 end : 3 us
}

void StateManager::UpdateCMM(RobotData &robotd_, LinkData *link_p)
{
    robotd_.CMM.setZero();

    Eigen::Matrix<double, 6, 6> trans_temp;

    Eigen::Matrix<double, 6, 6> inertia_temp;

    inertia_temp.setZero();

    for (int i = 0; i < LINK_NUMBER; i++)
    {
        trans_temp.setIdentity();
        inertia_temp.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * link_p[i].mass;
        // inertia_temp.block(3, 0, 3, 3) = link_p[i].mass * DyrosMath::skm(link_p[i].xipos - link_p[i].xpos);
        // inertia_temp.block(0, 3, 3, 3) = link_p[i].mass * DyrosMath::skm(link_p[i].xipos - link_p[i].xpos).transpose();
        inertia_temp.block(3, 3, 3, 3) = link_p[i].rotm * link_p[i].inertia * link_p[i].rotm.transpose();

        trans_temp.setIdentity();
        trans_temp.block(0, 3, 3, 3) = DyrosMath::skm(link_p[i].xipos - link_p[COM_id].xpos);
        robotd_.CMM = robotd_.CMM + trans_temp * inertia_temp * link_p[i].JacCOM();
    }
}

void StateManager::StateEstimate()
{
    if (rd_gl_.semode && (!rd_gl_.signal_yaw_init))
    {
        static bool contact_right, contact_left;
        static Eigen::Vector3d RF_contact_pos_holder, LF_contact_pos_holder;
        static Eigen::Vector3d RF_contact_pos_mod, LF_contact_pos_mod;
        static Eigen::Vector3d RF_CP_est_holder, LF_CP_est_holder;
        static Eigen::Vector3d RF_CP_est_holder_before, LF_CP_est_holder_before;
        static Eigen::Vector3d RF_CP_est_before, LF_CP_est_before;
        static Eigen::Vector3d imu_init;

        static Eigen::Vector3d pelv_v_before;
        static Eigen::Vector3d pelv_v;
        static Eigen::Vector3d pelv_anga;
        static Eigen::Vector3d pelv_x_before;
        static Eigen::Vector3d pelv_x;

        static double dr_static, dl_static;

        static bool ss_switch2 = false;

        if (dc_.stateEstimateSwitch)
        {
            dc_.stateEstimateSwitch = false;

            contact_right = false;
            contact_left = false;
            std::cout << " STATE : State Estimation Initialized" << std::endl;
            RF_contact_pos_holder.setZero(); // - RF_contactpoint_internal_pos(2);
            LF_contact_pos_holder.setZero(); // - LF_contactpoint_internal_pos(2);
            RF_contact_pos_mod.setZero();
            LF_contact_pos_mod.setZero();
            RF_CP_est_holder.setZero();
            LF_CP_est_holder.setZero();
            RF_CP_est_holder_before.setZero();
            LF_CP_est_holder_before.setZero();

            RF_CP_est_before.setZero();
            LF_CP_est_before.setZero();
            pelv_v_before.setZero();
            pelv_x_before.setZero();
            rd_.imu_ang_vel_before.setZero();
            imu_init = link_local_[Pelvis].rotm * rd_.imu_lin_acc;
            dr_static = 0.5;
            dl_static = 0.5;
            ss_switch2 = true;
        }

        RF_CP_est.setZero();
        LF_CP_est.setZero();

        RF_contact_pos_mod = RF_CP_est - RF_CP_est_before;
        LF_contact_pos_mod = LF_CP_est - LF_CP_est_before;

        RF_CP_est_before = RF_CP_est;
        LF_CP_est_before = LF_CP_est;

        Eigen::Vector3d RF_contactpoint_base_pos = link_local_[Right_Foot].contact_point;
        Eigen::Vector3d LF_contactpoint_base_pos = link_local_[Left_Foot].contact_point;
        Eigen::Vector3d RF_contactpoint_internal_pos = link_local_[Right_Foot].contact_point + RF_CP_est;
        Eigen::Vector3d LF_contactpoint_internal_pos = link_local_[Left_Foot].contact_point + LF_CP_est;
        Eigen::Vector3d mod_base_pos;
        Eigen::Vector3d mod_base_vel;
        Eigen::Vector3d rf_cp_m, lf_cp_m;
        Eigen::Vector3d RF_fixed_contact_pos, LF_fixed_contact_pos, RF_global_contact_pos, LF_global_contact_pos;
        Eigen::Vector3d RF_global_hold_pos, LF_global_hold_pos;
        Eigen::Vector6d RF_global_contact_vel, LF_global_contact_vel;
        Eigen::Vector6d RF_fixed_contact_vel, LF_fixed_contact_vel;
        Eigen::Vector3d RF_P_cpm, LF_P_cpm;

        link_[Right_Foot].GetPointPos(model_global_, q_virtual_, q_dot_virtual_, RF_contactpoint_internal_pos, RF_global_contact_pos, RF_global_contact_vel);
        link_[Left_Foot].GetPointPos(model_global_, q_virtual_, q_dot_virtual_, LF_contactpoint_internal_pos, LF_global_contact_pos, LF_global_contact_vel);

        link_local_[Right_Foot].GetPointPos(model_local_, q_virtual_local_yaw_initialized, q_dot_virtual_local_, RF_contactpoint_internal_pos, RF_fixed_contact_pos, RF_fixed_contact_vel);
        link_local_[Left_Foot].GetPointPos(model_local_, q_virtual_local_yaw_initialized, q_dot_virtual_local_, LF_contactpoint_internal_pos, LF_fixed_contact_pos, LF_fixed_contact_vel);

        bool local_RF_Contact, local_LF_contact;

        // if (dc.sebyft)
        // {
        //     //local_LF_contact = LF_Contact;
        //     //local_RF_Contact = RF_Contact;
        // }
        // else
        {
            local_LF_contact = rd_gl_.ee_[0].contact;
            local_RF_Contact = rd_gl_.ee_[1].contact;
        }

        bool left_change, right_change;
        left_change = false;
        right_change = false;
        if (contact_right != local_RF_Contact)
        {
            right_change = true;
            if (local_RF_Contact)
            {
                // std::cout << control_time_ << " STATE : right foot contact initialized" << std::endl;
                RF_contact_pos_holder = RF_global_contact_pos;
                RF_CP_est_holder_before = RF_CP_est_holder;
                RF_CP_est_holder = RF_CP_est;
            }
            else
            {
                // std::cout << control_time_ << " STATE : right foot contact disabled" << std::endl;
            }
        }
        if (contact_left != local_LF_contact)
        {
            left_change = true;
            if (local_LF_contact)
            {
                // std::cout << control_time_ << " STATE : left foot contact initialized" << std::endl;
                LF_contact_pos_holder = LF_global_contact_pos;
                LF_CP_est_holder_before = LF_CP_est_holder;
                LF_CP_est_holder = LF_CP_est;
            }
            else
            {
                // std::cout << control_time_ << " STATE : left foot contact disabled" << std::endl;
            }
        }

        if (ss_switch2)
        {

            LF_contact_pos_holder(2) = 0.0;
            RF_contact_pos_holder(2) = 0.0;
            ss_switch2 = false;
        }

        // imu pos estimation part (useless for now... )
        /*
        imu_lin_acc_lpf = DyrosMath::lpf(imu_lin_acc, imu_lin_acc_before, 2000, 20);
        imu_lin_acc_before = imu_lin_acc_lpf;
        pelv_lin_acc = dc.link_[Pelvis].rotm.inverse() * imu_lin_acc_lpf;
        double dt_i = 1.0 / 2000.0;
        Vector3d temp;
        temp = dc.tocabi_.imu_vel_ + dt_i * pelv_lin_acc;
        dc.tocabi_.imu_vel_ = temp;
        temp = dc.tocabi_.imu_pos_ + (dt_i * dt_i / 0.5) * pelv_lin_acc + dc.tocabi_.imu_vel_ * dt_i;
        dc.tocabi_.imu_pos_ = temp;*/
        // imu estimate end

        RF_P_cpm = link_local_[Right_Foot].rotm * (RF_CP_est - RF_CP_est_holder);
        LF_P_cpm = link_local_[Left_Foot].rotm * (LF_CP_est - LF_CP_est_holder);

        RF_contact_pos_holder = RF_contact_pos_holder + RF_P_cpm;
        LF_contact_pos_holder = LF_contact_pos_holder + LF_P_cpm;

        // Vector3d es_zmp;

        // es_zmp = dc.tocabi_.com_.pos - dc.tocabi_.com_.pos(2)/9.81*

        contact_right = local_RF_Contact;
        contact_left = local_LF_contact;

        rf_cp_m = RF_fixed_contact_pos - RF_contact_pos_holder;
        lf_cp_m = LF_fixed_contact_pos - LF_contact_pos_holder;

        double dr, dl;
        // dr =

        dr = DyrosMath::minmax_cut(RF_CF_FT(2) / (-total_mass_ * GRAVITY), 0.0, 1.0); // * dc.tocabi_.ee_[1].contact_accuracy;
        dl = DyrosMath::minmax_cut(LF_CF_FT(2) / (-total_mass_ * GRAVITY), 0.0, 1.0); // * dc.tocabi_.ee_[0].contact_accuracy;

        if (dr == 1)
        {
            dl = 0;
        }
        else if (dr == 0)
        {
            dl = 1;
        }
        else
        {
            if (dl == 1)
            {
                dr = 0;
            }
            else if (dl == 0)
            {
                dr = 1;
            }
        }

        dr_static = DyrosMath::lpf(dr, dr_static, 2000, 20);
        dl_static = DyrosMath::lpf(dl, dl_static, 2000, 20);

        rf_s_ratio = dr_static / (dr_static + dl_static);
        lf_s_ratio = dl_static / (dl_static + dr_static);

        lf_s_ratio = DyrosMath::minmax_cut(lf_s_ratio, 0.0, 1.0);

        if (lf_s_ratio == 0)
        {
            rf_s_ratio = 1;
        }
        else if (lf_s_ratio == 1)
        {
            rf_s_ratio = 0;
        }
        else
        {
            rf_s_ratio = DyrosMath::minmax_cut(rf_s_ratio, 0.0, 1.0);

            if (rf_s_ratio == 0)
            {
                lf_s_ratio = 1;
            }
            else if (rf_s_ratio == 1)
            {
                lf_s_ratio = 0;
            }
        }

        if (dc_.avatarMode)
        {
            rf_s_ratio = 0.5;
            lf_s_ratio = 0.5;
        }

        if (contact_right && contact_left)
        {
            mod_base_pos = rf_cp_m * rf_s_ratio + lf_cp_m * lf_s_ratio;
            // mod_base_pos(2) = mod_base_pos(2) + ((link_[Right_Foot].xpos(2) + link_[Right_Foot].contact_point(2)) * rf_s_ratio/ (rf_s_ratio + lf_s_ratio) + (link_[Left_Foot].xpos(2) + link_[Left_Foot].contact_point(2)) * lf_s_ratio / (rf_s_ratio + lf_s_ratio));
            mod_base_vel = -RF_fixed_contact_vel.segment(3, 3) * rf_s_ratio - LF_fixed_contact_vel.segment(3, 3) * lf_s_ratio;
        }
        else if (contact_right && (!contact_left))
        {
            mod_base_pos = rf_cp_m;
            mod_base_vel = -RF_fixed_contact_vel.segment(3, 3);
        }
        else if (contact_left && (!contact_right))
        {
            mod_base_pos = lf_cp_m;
            mod_base_vel = -LF_fixed_contact_vel.segment(3, 3);
        }
        else
        {
            std::cout << "whatthefuck" << std::endl;
        }

        // if (dc.single_foot_only)
        // {
        //     mod_base_pos = rf_cp_m;
        //     mod_base_vel = -RF_fixed_contact_vel.segment(3, 3);
        // }

        // Pelvis Velocity Complementary filter
        // v = alpha *(pelv_imu_acc * dt + v_before) + (1-alpha)*mb_v

        Vector3d imu_acc_dat;
        imu_acc_dat = link_local_[Pelvis].rotm * rd_.imu_lin_acc;

        imu_acc_dat = imu_acc_dat - imu_init;

        double dt = 0.0005;
        double tau = 0.4;
        double alpha = tau / (tau + dt);

        pelv_v = alpha * (imu_acc_dat * dt + pelv_v_before) + (1 - alpha) * mod_base_vel;
        pelv_v_before = pelv_v;
        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;

        // std::cout<<"pelv_v es : "<<pelv_v_before.transpose()<<" imu acc : "<<imu_acc_dat.transpose()<<"  imu init : "<<imu_init.transpose() <<" imu lin acc : "<<imu_lin_acc.transpose() <<" mod base : "<<mod_base_vel.transpose()<<std::endl;

        pelv_x = alpha * (pelv_v * dt + imu_acc_dat * dt * dt * 0.5 + pelv_x_before) + (1 - alpha) * (-mod_base_pos);
        pelv_x_before = pelv_x;

        pelv_anga = (q_dot_virtual_.segment<3>(3) - rd_.imu_ang_vel_before) * 2000;
        rd_.imu_ang_vel_before = q_dot_virtual_.segment<3>(3);

        // mod_base_vel

        pelvis_velocity_estimate_ = pelv_v;
        static Vector3d base_vel_lpf = mod_base_vel;
        base_vel_lpf = DyrosMath::lpf(mod_base_vel, base_vel_lpf, 2000, 3);

        for (int i = 0; i < 3; i++)
        {
            q_virtual_(i) = -mod_base_pos(i);
            q_dot_virtual_(i) = pelv_v(i);
            q_dot_virtual_(i) = mod_base_vel(i);

            // q_dot_virtual_(i) = base_vel_lpf(i);

            // q_ddot_virtual_(i) = imu_acc_dat(i);

            q_ddot_virtual_(i) = rd_.imu_lin_acc(i); // dg test
            q_ddot_virtual_(i + 3) = pelv_anga(i);
        }

        // fr_msg.data[6] = pelv_v[0];
        // fr_msg.data[7] = pelv_v[1];
        // fr_msg.data[8] = pelv_v[2];
        // fr_msg.data[9] = imu_acc_dat[0];
        // fr_msg.data[10] = imu_acc_dat[1];
        // fr_msg.data[11] = imu_acc_dat[2];

        // acceleration calculation!
        // q_ddot_virtual_ = (q_dot_virtual_ - q_dot_virtual_before) / ((double)dc.ctime / 1000000.0);
        // q_dot_virtual_before = q_dot_virtual_;

        static Vector3d pelv_pos_before;

        Vector3d currentPelvPos = q_virtual_.segment(0, 3);

        Vector3d pos_err = currentPelvPos - pelv_pos_before;
        static Vector3d rf1, rf2, rf3, rf4;
        static bool rf_b, lf_b;

        static double rfzb, lfzb;
        bool problem_is_here = false;
        static VectorQVQd q_v_before;
        static bool err_before = true;

        static Vector4d quat_before;

        // if (dc.torqueOn && (control_time_ > (dc.torqueOnTime + 5.0)))
        // {
        //     if (((currentPelvPos(0) == 0) && (currentPelvPos(1) == 0) && (currentPelvPos(2) == 0)) || ((pelv_pos_before(0) == 0) && (pelv_pos_before(1) == 0) && (pelv_pos_before(2) == 0)))
        //     {
        //     }
        //     else
        //     {

        //         for (int i = 0; i < 3; i++)
        //         {
        //             if (pos_err(i) * 2000 > 50.0)
        //                 problem_is_here = true;
        //         }
        //     }
        // }
        // if (problem_is_here)
        // {
        //     std::cout << cred << "WARNING :: PELV POSITION TRACKING ERROR :: BEFORE : " << pelv_pos_before.transpose() << "  NOW : " << currentPelvPos.transpose() << creset << std::endl;

        //     std::cout << "INFO : " << -mod_base_pos.transpose() << " RF ratio : " << rf_s_ratio << " LF ratio : " << lf_s_ratio << " RF ratio bf : " << rf_b << " LF ratio bf : " << lf_b << std::endl;
        //     std::cout << " RF fix cp : " << RF_fixed_contact_pos.transpose() << " RF cp hd : " << RF_contact_pos_holder.transpose() << " LF fix cp : " << LF_fixed_contact_pos.transpose() << " RF cp hdl : " << LF_contact_pos_holder.transpose() << std::endl;

        //     std::cout << " RF fix cp : " << rf1.transpose() << " RF cp hd : " << rf2.transpose() << " LF fix cp : " << rf3.transpose() << " RF cp hdl : " << rf4.transpose() << std::endl;

        //     std::cout << q_virtual_local_.transpose() << std::endl;
        //     std::cout << q_v_before.transpose() << std::endl;

        //     std::cout << "imu now : " << imu_quat.transpose() << "\t imu before : " << quat_before.transpose() << std::endl;

        //     std::cout << "lf z : " << LF_CF_FT(2) / (-com_.mass * GRAVITY) << "\t"
        //               << " rf z : " << RF_CF_FT(2) / (-com_.mass * GRAVITY) << " lfz before : " << lfzb << " rfz before : " << rfzb << std::endl;

        //     //q_virtual_.segment(0, 3) = pelv_pos_before;
        //     //currentPelvPos = pelv_pos_before;
        // }
        q_v_before = q_virtual_local_;
        rf_b = rf_s_ratio;
        lf_b = lf_s_ratio;
        rf1 = RF_fixed_contact_pos;
        rf2 = RF_contact_pos_holder;
        rf3 = LF_fixed_contact_pos;
        rf4 = LF_contact_pos_holder;
        pelv_pos_before = currentPelvPos;
        // quat_before = imu_quat;
        rfzb = RF_CF_FT(2) / (-total_mass_ * GRAVITY);
        lfzb = LF_CF_FT(2) / (-total_mass_ * GRAVITY);
    }
    else
    {
        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;
        q_ddot_virtual_ = q_ddot_virtual_local_;
    }
}

void StateManager::CalcNonlinear()
{
    // RigidBodyDynamics::NonlinearEffects(model_local_,)
}

void StateManager::PublishData()
{
    geometry_msgs::TransformStamped ts;

    ts.header.stamp = ros::Time::now();

    ts.header.frame_id = "world";
    ts.child_frame_id = "Pelvis_Link";

    ts.transform.rotation.x = q_virtual_local_[3];
    ts.transform.rotation.y = q_virtual_local_[4];
    ts.transform.rotation.z = q_virtual_local_[5];
    ts.transform.rotation.w = q_virtual_local_[MODEL_DOF_VIRTUAL];

    ts.transform.translation.x = q_virtual_(0);
    ts.transform.translation.y = q_virtual_(1);
    ts.transform.translation.z = q_virtual_(2);

    br.sendTransform(ts);

    joint_state_msg_.header.stamp = ros::Time::now();
    for (int i = 0; i < MODEL_DOF; i++)
    {
        joint_state_msg_.position[i] = q_virtual_local_[i + 6];
        joint_state_msg_.velocity[i] = q_dot_virtual_local_[i + 6];
        joint_state_msg_.effort[i] = rd_gl_.torque_elmo_[i];
    }
    joint_state_pub_.publish(joint_state_msg_);

    point_pub_msg_.header.stamp = ros::Time::now();

    point_pub_msg_.polygon.points[0].x = link_[COM_id].xpos(0);
    point_pub_msg_.polygon.points[0].y = link_[COM_id].xpos(1);
    point_pub_msg_.polygon.points[0].z = link_[COM_id].xpos(2);

    point_pub_msg_.polygon.points[1].x = link_[Right_Foot].xpos(0);
    point_pub_msg_.polygon.points[1].y = link_[Right_Foot].xpos(1);
    point_pub_msg_.polygon.points[1].z = link_[Right_Foot].xpos(2);

    point_pub_msg_.polygon.points[2].x = link_[Left_Foot].xpos(0);
    point_pub_msg_.polygon.points[2].y = link_[Left_Foot].xpos(1);
    point_pub_msg_.polygon.points[2].z = link_[Left_Foot].xpos(2);

    point_pub_msg_.polygon.points[3].x = link_[Pelvis].xpos(0);
    point_pub_msg_.polygon.points[3].y = link_[Pelvis].xpos(1);
    point_pub_msg_.polygon.points[3].z = link_[Pelvis].xpos(2);

    double tr_, tp_, ty_;

    DyrosMath::rot2Euler_tf2(link_[Pelvis].rotm, tr_, tp_, ty_);

    point_pub_msg_.polygon.points[4].x = tr_; // rpy
    point_pub_msg_.polygon.points[4].y = tp_; // rpy
    point_pub_msg_.polygon.points[4].z = ty_; // rpy

    point_pub_msg_.polygon.points[5].x = link_[Right_Hand].xpos(0);
    point_pub_msg_.polygon.points[5].y = link_[Right_Hand].xpos(1);
    point_pub_msg_.polygon.points[5].z = link_[Right_Hand].xpos(2);

    point_pub_msg_.polygon.points[6].x = link_[Left_Hand].xpos(0);
    point_pub_msg_.polygon.points[6].y = link_[Left_Hand].xpos(1);
    point_pub_msg_.polygon.points[6].z = link_[Left_Hand].xpos(2);

    point_pub_msg_.polygon.points[7].x = rd_gl_.zmp_global_(0);
    point_pub_msg_.polygon.points[7].y = rd_gl_.zmp_global_(1);
    point_pub_msg_.polygon.points[7].z = 0.0;

    DyrosMath::rot2Euler_tf2(link_[Left_Foot].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[8].x = tr_;
    point_pub_msg_.polygon.points[8].y = tp_;
    point_pub_msg_.polygon.points[8].z = ty_;

    DyrosMath::rot2Euler_tf2(link_[Right_Foot].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[9].x = tr_;
    point_pub_msg_.polygon.points[9].y = tp_;
    point_pub_msg_.polygon.points[9].z = ty_;

    DyrosMath::rot2Euler_tf2(link_[Upper_Body].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[10].x = tr_;
    point_pub_msg_.polygon.points[10].y = tp_;
    point_pub_msg_.polygon.points[10].z = ty_;

    // geometry_msgs::Point32 p_vel, p_vel_virtual_;

    point_pub_msg_.polygon.points[11].x = link_[Pelvis].v(0);
    point_pub_msg_.polygon.points[11].y = link_[Pelvis].v(1);
    point_pub_msg_.polygon.points[11].z = link_[Pelvis].v(2);

    point_pub_msg_.polygon.points[12].x = dc_.tc_shm_->vel_virtual[0];
    point_pub_msg_.polygon.points[12].y = dc_.tc_shm_->vel_virtual[1];
    point_pub_msg_.polygon.points[12].z = dc_.tc_shm_->vel_virtual[2];

    point_pub_msg_.polygon.points[13].x = rd_gl_.ee_[0].xpos_contact(0) - LF_CF_FT(4) / LF_CF_FT(2);
    point_pub_msg_.polygon.points[13].y = rd_gl_.ee_[0].xpos_contact(1) + LF_CF_FT(3) / LF_CF_FT(2);
    point_pub_msg_.polygon.points[13].z = LF_CF_FT(2);

    // std::cout << LF_CF_FT.transpose() << rd_gl_.ee_[0].xpos_contact.transpose() << std::endl;

    point_pub_msg_.polygon.points[14].x = rd_gl_.ee_[1].xpos_contact(0) - RF_CF_FT(4) / RF_CF_FT(2);
    point_pub_msg_.polygon.points[14].y = rd_gl_.ee_[1].xpos_contact(1) + RF_CF_FT(3) / RF_CF_FT(2);
    point_pub_msg_.polygon.points[14].z = RF_CF_FT(2);

    point_pub_msg_.polygon.points[15].x = LF_CF_FT(0);
    point_pub_msg_.polygon.points[15].y = LF_CF_FT(1);
    point_pub_msg_.polygon.points[15].z = LF_CF_FT(2);

    // std::cout << LF_CF_FT.transpose() << rd_gl_.ee_[0].xpos_contact.transpose() << std::endl;

    point_pub_msg_.polygon.points[16].x = LF_CF_FT(3);
    point_pub_msg_.polygon.points[16].y = LF_CF_FT(4);
    point_pub_msg_.polygon.points[16].z = LF_CF_FT(5);

    point_pub_msg_.polygon.points[17].x = RF_CF_FT(0);
    point_pub_msg_.polygon.points[17].y = RF_CF_FT(1);
    point_pub_msg_.polygon.points[17].z = RF_CF_FT(2);

    // std::cout << LF_CF_FT.transpose() << rd_gl_.ee_[0].xpos_contact.transpose() << std::endl;

    point_pub_msg_.polygon.points[18].x = RF_CF_FT(3);
    point_pub_msg_.polygon.points[18].y = RF_CF_FT(4);
    point_pub_msg_.polygon.points[18].z = RF_CF_FT(5);

    point_pub_msg_.polygon.points[19].x = link_local_[Left_Foot].v(0);
    point_pub_msg_.polygon.points[19].y = link_local_[Left_Foot].v(1);
    point_pub_msg_.polygon.points[19].z = link_local_[Left_Foot].v(2);

    // static double com_pos_before = 0;

    point_pub_msg_.polygon.points[20].x = link_local_[Right_Foot].v(0);
    point_pub_msg_.polygon.points[20].y = link_local_[Right_Foot].v(1);
    point_pub_msg_.polygon.points[20].z = link_local_[Right_Foot].v(2);

    point_pub_msg_.polygon.points[21].x = rd_gl_.link_[COM_id].x_traj(0);
    point_pub_msg_.polygon.points[21].y = rd_gl_.link_[COM_id].x_traj(1);
    point_pub_msg_.polygon.points[21].z = rd_gl_.link_[COM_id].x_traj(2);

    point_pub_msg_.polygon.points[22].x = rd_gl_.zmp_global_(0) - link_[COM_id].xpos(0);
    point_pub_msg_.polygon.points[22].y = rd_gl_.zmp_global_(1) - link_[COM_id].xpos(1);
    point_pub_msg_.polygon.points[22].z = rd_gl_.link_[COM_id].v_traj(2);

    point_pub_msg_.polygon.points[23].x = rd_gl_.link_[Pelvis].xpos(1);
    point_pub_msg_.polygon.points[23].y = rd_gl_.link_[Pelvis].v(1);
    point_pub_msg_.polygon.points[23].z = rd_gl_.task_force_(1);

    // com_pos_before = link_[COM_id].xpos(1);

    point_pub_.publish(point_pub_msg_);

    Eigen::Quaterniond q_head_(DyrosMath::rotateWithZ(-rd_.yaw) * link_[Head].rotm);

    head_pose_msg_.orientation.w = q_head_.w();
    head_pose_msg_.orientation.x = q_head_.x();
    head_pose_msg_.orientation.y = q_head_.y();
    head_pose_msg_.orientation.z = q_head_.z();

    head_pose_pub_.publish(head_pose_msg_);

    // head_pose_msg_.orientation.

    //

    bool query_elmo_pub_ = false;

    static int gui_pub_cnt = 0;
    gui_pub_cnt++;
    if (gui_pub_cnt % 6 == 0)
    {
        query_elmo_pub_ = true;
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        elmo_status_msg_.data[i] = state_elmo_[i];

        elmo_status_msg_.data[i + 33] = state_zp_[i];

        if (dc_.tc_shm_->safety_disable)
        {
            elmo_status_msg_.data[i + 66] = 9;
        }
        else
        {
            elmo_status_msg_.data[i + 66] = state_safety_[i];
        }

        if (state_elmo_[i] != state_elmo_before_[i])
        {
            query_elmo_pub_ = true;
        }

        if (state_safety_[i] != state_safety_before_[i])
        {
            query_elmo_pub_ = true;
        }

        if (state_zp_[i] != state_zp_before_[i])
        {
            query_elmo_pub_ = true;
        }

        state_elmo_before_[i] = state_elmo_[i];
        state_zp_before_[i] = state_zp_[i];
    }

    if (dc_.tc_shm_->lower_disabled)
    {
        for (int i = 0; i < 12; i++)
        {
            elmo_status_msg_.data[i + 66] = 6;
        }
    }

    if (dc_.locklower)
    {
        for (int i = 0; i < 12; i++)
        {
            elmo_status_msg_.data[i + 66] = 7;
        }
    }

    if (query_elmo_pub_)
    {
        elmo_status_pub_.publish(elmo_status_msg_);
    }

    if (dc_.simMode)
    {
        syspub_msg.data[0] = 3;
        syspub_msg.data[1] = 3;
        syspub_msg.data[2] = 3;
        syspub_msg.data[3] = 3;
    }
    else
    {
        syspub_msg.data[0] = dc_.tc_shm_->imu_state;
        if (dc_.tc_shm_->initializeModeUpper && !dc_.tc_shm_->controlModeUpper)
        {
            syspub_msg.data[1] = 1;
        }
        else if (dc_.tc_shm_->controlModeUpper && dc_.tc_shm_->initializeModeUpper)
        {
            syspub_msg.data[1] = 2;
        }
        else if (!dc_.tc_shm_->controlModeUpper && !dc_.tc_shm_->initializeModeUpper)
        {
            syspub_msg.data[1] = 0;
        }

        if (dc_.tc_shm_->initializeModeLower && !dc_.tc_shm_->controlModeLower)
        {
            syspub_msg.data[3] = 1;
        }
        else if (dc_.tc_shm_->controlModeLower && dc_.tc_shm_->initializeModeLower)
        {
            syspub_msg.data[3] = 2;
        }
        else if (!dc_.tc_shm_->controlModeLower && !dc_.tc_shm_->initializeModeLower)
        {
            syspub_msg.data[3] = 0;
        }

        syspub_msg.data[2] = dc_.tc_shm_->ft_state;
        // syspub_msg.data[4] = dc_.tc_shm_->
    }
    syspub_msg.data[4] = rd_gl_.semode;
    if (!rd_gl_.pc_mode && rd_gl_.tc_run) // tc on warn error off
    {
        syspub_msg.data[5] = 0; // triangle
    }
    else
    {
        syspub_msg.data[5] = 3; // off
    }

    if (!rd_gl_.pc_mode && rd_gl_.tc_run && rd_gl_.tc_.mode == 13)
    {
        syspub_msg.data[6] = 1; // AVATAR MODE!
    }
    else if (!rd_gl_.pc_mode && rd_gl_.tc_run && rd_gl_.tc_.mode == 12)
    {
        syspub_msg.data[6] = 2; // AVATAR MODE!
    }
    else
    {
        syspub_msg.data[6] = 3; // AVATAR MODE!
    }

    gui_state_pub_.publish(syspub_msg);

    com_status_msg_.data[0] = dc_.tc_shm_->lat_avg / 1000.0;
    com_status_msg_.data[1] = dc_.tc_shm_->lat_max / 1000.0;
    com_status_msg_.data[2] = dc_.tc_shm_->send_avg / 1000.0;
    com_status_msg_.data[3] = dc_.tc_shm_->send_max / 1000.0;
    com_status_msg_.data[4] = dc_.tc_shm_->send_ovf;

    com_status_msg_.data[5] = dc_.tc_shm_->lat_avg2 / 1000.0;
    com_status_msg_.data[6] = dc_.tc_shm_->lat_max2 / 1000.0;
    com_status_msg_.data[7] = dc_.tc_shm_->send_avg2 / 1000.0;
    com_status_msg_.data[8] = dc_.tc_shm_->send_max2 / 1000.0;
    com_status_msg_.data[9] = dc_.tc_shm_->send_ovf2;

    com_status_msg_.data[10] = dc_.tc_shm_->statusCount;
    com_status_msg_.data[11] = dc_.tc_shm_->statusCount2;

    com_status_msg_.data[12] = dc_.tc_shm_->watchCount;
    com_status_msg_.data[13] = dc_.tc_shm_->watchCount2;

    com_status_msg_.data[14] = torqueRatio;

    com_status_msg_.data[15] = dc_.stm_cnt;
    com_status_msg_.data[16] = dc_.tcm_cnt;

    com_status_pub_.publish(com_status_msg_);
    //
    // memcpy(joint_state_before_, joint_state_, sizeof(int) * MODEL_DOF);

    for (int i = 0; i < 6; i++)
    {
        hand_ft_org_msg_.data[i] = LH_FT[i];
        hand_ft_org_msg_.data[i + 6] = RH_FT[i];
    }
    hand_ft_pub_org_.publish(hand_ft_org_msg_);
}

void StateManager::handrcurrentCallback(const std_msgs::Int16MultiArrayConstPtr &msg)
{
    handr_current.resize(msg->data.size());
    bool detectr = false;
    for (int i = 0; i < msg->data.size(); i++)
    {
        handr_current(i) = msg->data[i];
        if (abs(handr_current(i)) > 300)
        {
            detectr = true;
            break;
        }
        else
        {

            detectr = false;
        }
    }
    if (detectr == true)
    {
        hand_grasp_r = true;
    }
    else
    {
        hand_grasp_r = false;
    }
}

void StateManager::handlcurrentCallback(const std_msgs::Int16MultiArrayConstPtr &msg)
{
    handl_current.resize(msg->data.size());
    bool detectl = false;
    for (int i = 0; i < msg->data.size(); i++)
    {
        handl_current(i) = msg->data[i];
        if (abs(handl_current(i)) > 300)
        {
            detectl = true;
            break;
        }
        else
        {

            detectl = false;
        }
    }
    if (detectl == true)
    {
        hand_grasp_l = true;
    }
    else
    {
        hand_grasp_l = false;
    }
}

void StateManager::SimCommandCallback(const std_msgs::StringConstPtr &msg)
{

    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        // parameterInitialize();
        sim_time_before_ = 0.0;

        mujoco_ready = true;

        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        mujoco_init_receive = true;
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_.publish(rst_msg_);
        sim_time_ = 0.0;
        control_time_ = 0.0;
        // dc.semode_init = true;
        mujoco_reset = true;
    }

    if (buf == "terminate")
    {
        dc_.tc_shm_->shutdown = true;
    }
}
void StateManager::StopCallback(const std_msgs::StringConstPtr &msg)
{

    if (msg->data == "stop_tocabi")
    {
        dc_.tc_shm_->shutdown = true;
    }
}

void StateManager::GuiCommandCallback(const std_msgs::StringConstPtr &msg)
{
    // std::cout << "Received msg from GUI : " << msg->data << std::endl;
    // Receiving Command from GUI!

    if (msg->data == "torqueon")
    {
        StatusPub("%f Torque ON", control_time_);
        dc_.torqueOnSwitch = true;
    }
    else if (msg->data == "torqueoff")
    {
        StatusPub("%f Torque Off", control_time_);
        dc_.torqueOffSwitch = true;
    }
    else if (msg->data == "E0")
    {
        std::cout << " CNTRL : Emergency Switch pressed! " << std::endl;

        dc_.emergencySwitch = true;
        dc_.tc_shm_->emergencyOff = true;
    }
    else if (msg->data == "E1")
    {
        std::cout << " CNTRL : Emergency Stop Activated !" << std::endl;
        dc_.E1Switch = true;
    }
    else if (msg->data == "E2")
    {
        std::cout << "Emergency Damping mode Active ! " << std::endl;
        // dc_.E2Switch = true;
        std::cout << "E2 Not supported..." << std::endl;
    }
    else if (msg->data == "gravity")
    {
        StatusPub("%f Gravity Control", control_time_);
        rd_gl_.tc_run = false;
        rd_gl_.pc_mode = false;
    }
    else if (msg->data == "inityaw")
    {
        StatusPub("%f Init Yaw", control_time_);
        dc_.inityawSwitch = true;
    }
    else if (msg->data == "ftcalib")
    {
        dc_.ftcalibSwtich = true;
    }
    else if (msg->data == "handftcalib")
    {
        dc_.handft_calib_signal_ = true;
    }
    else if (msg->data == "handftzero")
    {
        dc_.fthandzeroSwtich = true;
    }
    else if (msg->data == "imureset")
    {
        dc_.imuResetSwtich = true;
    }
    else if (msg->data == "handftcalibreset")
    {
        dc_.fthandcalibreset = true;
    }
    else if (msg->data == "stateestimation")
    {
        if (rd_gl_.semode)
        {
            std::cout << " STATE : stateestimation off" << std::endl;
            StatusPub("%f StateEstimate Off", control_time_);
            rd_gl_.semode = false;
        }
        else
        {
            std::cout << " STATE : stateestimation on" << std::endl;
            StatusPub("%f StateEstimate ON", control_time_);
            dc_.inityawSwitch = true;

            dc_.stateEstimateSwitch = true;
            rd_gl_.semode = true;
            dc_.useSimVirtual = false;
        }
    }
    else if (msg->data == "safetyreset")
    {
        dc_.safetyResetSwitch = true;
        dc_.tc_shm_->safety_reset_lower_signal = true;
        dc_.tc_shm_->safety_reset_upper_signal = true;
    }
    else if (msg->data == "safetydisable")
    {
        dc_.tc_shm_->safety_disable = !dc_.tc_shm_->safety_disable;

        if (dc_.tc_shm_->safety_disable)
        {
            std::cout << " CNTRL : safety checking disabled!" << std::endl;
            StatusPub("%f safety disabled", control_time_);
            dc_.tc_shm_->safety_reset_lower_signal = true;
            dc_.tc_shm_->safety_reset_upper_signal = true;
        }
        else
        {

            StatusPub("%f safety enabled", control_time_);
            std::cout << " CNTRL : safety checking enabled!" << std::endl;
        }
    }
    else if (msg->data == "ecatinit")
    {
        StatusPub("%f Initialize upper", control_time_);
        dc_.tc_shm_->upper_init_signal = true;
    }
    else if (msg->data == "disablelower")
    {
        dc_.tc_shm_->lower_disabled = !dc_.tc_shm_->lower_disabled;
        if (dc_.tc_shm_->lower_disabled)
        {
            std::cout << "lowerbody disable" << std::endl;
            dc_.locklower = false;
        }
        else
        {
            std::cout << "lowerbody activate" << std::endl;
            dc_.locklower = false;
        }
    }
    else if (msg->data == "locklower")
    {
        if (dc_.tc_shm_->lower_disabled)
        {
            std::cout << "Cannot activate LOCK LOWER : LOWER DISABLED" << std::endl;
        }
        else
        {

            dc_.locklower = !dc_.locklower;
            if (dc_.locklower)
            {
                dc_.qlock_des = rd_gl_.q_desired.segment(0, 12);
                std::cout << "locklower activate" << std::endl;
            }
            else
            {
                std::cout << "locklower disable" << std::endl;
            }
        }
    }
    else if (msg->data == "ecatinitlower")
    {
        StatusPub("%f Initialize lower", control_time_);
        dc_.tc_shm_->low_init_signal = true;
    }
    else if (msg->data == "ecatinitwaist")
    {
        StatusPub("%f Initialize Waist", control_time_);
        dc_.tc_shm_->waist_init_signal = true;
    }
    else if (msg->data == "simvirtualjoint")
    {
        dc_.useSimVirtual = !dc_.useSimVirtual;

        if (dc_.useSimVirtual)
        {
            StatusPub("%f sim vel mode on", control_time_);
            rd_gl_.semode = false;
        }
        else
        {

            StatusPub("%f sim vel mode off", control_time_);
        }
    }
    else if (msg->data == "positioncontrol")
    {
        StatusPub("%f Position Control Activate", control_time_);
        dc_.rd_.positionControlSwitch = true;
    }
    else if (msg->data == "forceload")
    {
        dc_.tc_shm_->force_load_saved_signal = true;
    }
    else if (msg->data == "startlog")
    {
        dc_.logdata_start = true;
    }
    else if (msg->data == "stoplog")
    {
        dc_.logdata_stop = true;
    }
    else if (msg->data == "qdot_est")
    {
        qdot_estimation_switch = !qdot_estimation_switch;

        if (qdot_estimation_switch)
        {
            std::cout << "turn on qdot est" << std::endl;
        }
        else
        {
            std::cout << "turn off qdot est" << std::endl;
        }
    }
    else if (msg->data == "forcesegfault")
    {
        // CAUTION
        int *foo = NULL;
        *foo = 1;
        // CAUTION
    }

    // Controlling GUI
}

void StateManager::StatusPub(const char *str, ...)
{
    va_list lst;
    va_start(lst, str);

    char text_[256];

    vsnprintf(text_, 255, str, lst);

    std::string str_(text_);

    std_msgs::String str_msg_;
    str_msg_.data = str_;

    status_pub_.publish(str_msg_);

    // std::cout<<str_;

    va_end(lst);
}

void StateManager::initializeJointMLP()
{
    // network weights
    W1.setZero(40, 100);
    W2.setZero(100, 100);
    W3.setZero(100, 1);
    b1.setZero(100);
    b2.setZero(100);
    // b3.setZero(1);
    h1.setZero(100);
    h2.setZero(100);

    q_dot_buffer_slow_.setZero(40, 33);
    q_dot_buffer_fast_.setZero(40, 33);
    q_dot_buffer_thread_.setZero(40, 33);

    nn_estimated_q_dot_slow_.setZero();
    nn_estimated_q_dot_fast_.setZero();
    nn_estimated_q_dot_thread_.setZero();
    nn_estimated_q_dot_pre_.setZero();
}
void StateManager::loadJointVelNetwork(std::string folder_path)
{
    std::string W_1_path("weight_0801[0].txt");
    std::string W_2_path("weight_0801[2].txt");
    std::string W_3_path("weight_0801[4].txt");

    std::string b_1_path("weight_0801[1].txt");
    std::string b_2_path("weight_0801[3].txt");
    std::string b_3_path("weight_0801[5].txt");

    W_1_path = folder_path + W_1_path;
    W_2_path = folder_path + W_2_path;
    W_3_path = folder_path + W_3_path;

    b_1_path = folder_path + b_1_path;
    b_2_path = folder_path + b_2_path;
    b_3_path = folder_path + b_3_path;

    joint_vel_net_weights_file_[0].open(W_1_path, ios::in);
    joint_vel_net_weights_file_[1].open(b_1_path, ios::in);
    joint_vel_net_weights_file_[2].open(W_2_path, ios::in);
    joint_vel_net_weights_file_[3].open(b_2_path, ios::in);
    joint_vel_net_weights_file_[4].open(W_3_path, ios::in);
    joint_vel_net_weights_file_[5].open(b_3_path, ios::in);

    int index = 0;
    float temp;

    // W1
    if (!joint_vel_net_weights_file_[0].is_open())
    {
        std::cout << "Can not find the weight[0].txt file" << std::endl;
    }
    for (int i = 0; i < 40; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            joint_vel_net_weights_file_[0] >> W1(i, j);
        }
    }
    joint_vel_net_weights_file_[0].close();
    // cout<<"W1: \n"<<W1<<endl;

    // b1
    if (!joint_vel_net_weights_file_[1].is_open())
    {
        std::cout << "Can not find the weight[1].txt file" << std::endl;
    }
    for (int i = 0; i < 100; i++)
    {
        joint_vel_net_weights_file_[1] >> b1(i);
    }
    joint_vel_net_weights_file_[1].close();
    // cout<<"b1: \n"<<b1.transpose()<<endl;

    // W2
    if (!joint_vel_net_weights_file_[2].is_open())
    {
        std::cout << "Can not find the weight[2].txt file" << std::endl;
    }
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            joint_vel_net_weights_file_[2] >> W2(i, j);
        }
    }
    joint_vel_net_weights_file_[2].close();
    // cout<<"W2: \n"<<W2<<endl;

    // b2
    if (!joint_vel_net_weights_file_[3].is_open())
    {
        std::cout << "Can not find the weight[3].txt file" << std::endl;
    }
    for (int i = 0; i < 100; i++)
    {
        joint_vel_net_weights_file_[3] >> b2(i);
    }
    joint_vel_net_weights_file_[3].close();
    // cout<<"b2: \n"<<b2.transpose()<<endl;

    // W3
    if (!joint_vel_net_weights_file_[4].is_open())
    {
        std::cout << "Can not find the weight[4].txt file" << std::endl;
    }
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            joint_vel_net_weights_file_[4] >> W3(i, j);
        }
    }
    joint_vel_net_weights_file_[4].close();
    // cout<<"W3: \n"<<W3<<endl;

    // b3
    if (!joint_vel_net_weights_file_[5].is_open())
    {
        std::cout << "Can not find the weight[5].txt file" << std::endl;
    }
    for (int i = 0; i < 1; i++)
    {
        joint_vel_net_weights_file_[5] >> b3;
    }
    joint_vel_net_weights_file_[5].close();
    // cout<<"b3: \n"<<b3<<endl;
}
void StateManager::calculateJointVelMlpInput()
{
    // 최근 20개 속도 임시 저장하는 변수
    for (int i = 0; i < MODEL_DOF; i++)
    {
        for (int j = 1; j < 40; j++)
        {
            q_dot_buffer_slow_(40 - j, i) = q_dot_buffer_slow_(39 - j, i);
        }
    }

    VectorQd qdot_actual_double = Map<VectorQf>(q_dot_a_, MODEL_DOF).cast<double>();

    // q_dot_ = Map<VectorQf>(q_dot_a_, MODEL_DOF).cast<double>();

    for (int joint = 0; joint < MODEL_DOF; joint++)
    {
        q_dot_buffer_slow_(0, joint) = qdot_actual_double(joint) * 101;
    }

    if (atb_mlp_input_update_ == false)
    {
        atb_mlp_input_update_ = true;
        q_dot_buffer_thread_ = q_dot_buffer_slow_;
        atb_mlp_input_update_ = false;
    }
}
void StateManager::calculateJointVelMlpOutput()
{
    if (atb_mlp_input_update_ == false)
    {
        atb_mlp_input_update_ = true;
        q_dot_buffer_fast_ = q_dot_buffer_thread_;
        atb_mlp_input_update_ = false;
    }

    nn_estimated_q_dot_fast_.setZero();

    for (int joint = 0; joint < MODEL_DOF; joint++)
    {
        h1.setZero();
        h2.setZero();

        // Input -> Hidden Layer 1
        h1 = W1.transpose() * q_dot_buffer_fast_.block(0, joint, 40, 1) + b1;

        for (int i = 0; i < 100; i++)
        {
            if (h1(i) < 0)
            {
                h1(i) = 0;
            }
        }
        // Hidden Layer1 -> Hidden Layer2
        h2 = W2.transpose() * h1 + b2;
        for (int i = 0; i < 100; i++)
        {
            if (h2(i) < 0)
            {
                h2(i) = 0;
            }
        }

        // Hidden Layer2 -> output (Dense)
        nn_estimated_q_dot_fast_(joint) = (W3.transpose() * h2)(0) + b3;

        nn_estimated_q_dot_fast_(joint) = 0.7 * nn_estimated_q_dot_fast_(joint) + 0.3 * q_dot_buffer_fast_(0, joint);
        nn_estimated_q_dot_fast_(joint) = nn_estimated_q_dot_fast_(joint) / 101;
    }
    nn_estimated_q_dot_fast_ = DyrosMath::lpf<33>(nn_estimated_q_dot_fast_, nn_estimated_q_dot_pre_, 2000.0, 2 * M_PI * 200);
    nn_estimated_q_dot_pre_ = nn_estimated_q_dot_fast_;

    // q_dot_ = nn_estimated_q_dot_fast_;

    if (atb_mlp_output_update_ == false)
    {
        atb_mlp_output_update_ = true;
        nn_estimated_q_dot_thread_ = nn_estimated_q_dot_fast_;
        atb_mlp_output_update_ = false;
    }
}
