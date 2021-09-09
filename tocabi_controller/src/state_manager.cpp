#include "tocabi_controller/state_manager.h"
#include "fstream"
#include "algorithm"

using namespace std;
using namespace TOCABI;

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

        link_[Right_Hand].contact_point << 0, 0.0, -0.035;
        link_[Right_Hand].sensor_point << 0.0, 0.0, 0.0;
        link_[Left_Hand].contact_point << 0, 0.0, -0.035;
        link_[Left_Hand].sensor_point << 0.0, 0.0, 0.0;

        memcpy(link_local_, link_, sizeof(LinkData) * LINK_NUMBER);
    }

    if (dc_.simMode)
    {
        mujoco_sim_command_pub_ = dc_.nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100);
        mujoco_sim_command_sub_ = dc_.nh.subscribe("/mujoco_ros_interface/sim_command_sim2con", 100, &StateManager::SimCommandCallback, this);
    }

    joint_state_pub_ = dc_.nh.advertise<sensor_msgs::JointState>("/tocabi/jointstates", 100);

    joint_state_msg_.name.resize(MODEL_DOF);
    joint_state_msg_.position.resize(MODEL_DOF);
    joint_state_msg_.velocity.resize(MODEL_DOF);
    joint_state_msg_.effort.resize(MODEL_DOF);

    for (int i = 0; i < MODEL_DOF; i++)
        joint_state_msg_.name[i] = JOINT_NAME[i];

    gui_command_sub_ = dc_.nh.subscribe("/tocabi/command", 100, &StateManager::GuiCommandCallback, this);
    gui_state_pub_ = dc_.nh.advertise<std_msgs::Int8MultiArray>("/tocabi/systemstate", 100);
    point_pub_ = dc_.nh.advertise<geometry_msgs::PolygonStamped>("/tocabi/point", 100);
    status_pub_ = dc_.nh.advertise<std_msgs::String>("/tocabi/guilog", 100);
    timer_pub_ = dc_.nh.advertise<std_msgs::Float32>("/tocabi/time", 100);
    head_pose_pub_ = dc_.nh.advertise<geometry_msgs::Pose>("/tocabi/headpose", 100);

    elmo_status_pub_ = dc_.nh.advertise<std_msgs::Int8MultiArray>("/tocabi/ecatstates", 100);

    point_pub_msg_.polygon.points.resize(13);
    syspub_msg.data.resize(8);
    elmo_status_msg_.data.resize(MODEL_DOF * 3);
}

StateManager::~StateManager()
{
    cout << "StateManager Terminate" << endl;
}

void *StateManager::StateThread()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int rcv_tcnt = -1;

    //Checking Connect//

    //Check Coonnect Complete//
    rcv_tcnt = dc_.tc_shm_->statusCount;
    //cout << "first packet " << rcv_tcnt << endl;
    int cycle_count_ = rcv_tcnt;
    int stm_count_ = 0;

    int cnt = 0;
    int cnt2 = 0;
    int cnt3 = 0;
    auto time_start = std::chrono::steady_clock::now();

    timespec tv_us1;
    tv_us1.tv_sec = 0;
    tv_us1.tv_nsec = 1000;

    while (!dc_.tc_shm_->shutdown)
    {
        //////////////////////////////
        //////////State Loop//////////
        //////////////////////////////

        dc_.tc_shm_->stloopCount.store(stm_count_);

        SendCommand();

        while (!dc_.tc_shm_->triggerS1.load(std::memory_order_acquire))
        {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &tv_us1, NULL);
            if (dc_.tc_shm_->shutdown)
                break;
        }

        rd_.tp_state_ = std::chrono::steady_clock::now();
        auto t1 = rd_.tp_state_;

        dc_.tc_shm_->triggerS1 = false;
        cycle_count_++;
        stm_count_++;

        rcv_tcnt = dc_.tc_shm_->statusCount;

        GetJointData(); //0.246 us //w/o march native 0.226
        GetSensorData();

        InitYaw();

        auto dur_start_ = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_start).count();
        control_time_ = rcv_tcnt / 2000.0;

        //local kinematics update : 33.7 us // w/o march native 20 us
        UpdateKinematics_local(model_local_, link_local_, q_virtual_local_, q_dot_virtual_local_, q_ddot_virtual_local_);

        auto d1 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t1).count();

        auto t2 = chrono::steady_clock::now();
        StateEstimate();

        //global kinematics update : 127 us //w/o march native 125 us
        UpdateKinematics(model_global_, link_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);

        StoreState(rd_gl_); //6.2 us //w/o march native 8us

        //MeasureTime(stm_count_, d1, d2);

        rd_gl_.control_time_ = dur_start_ / 1000000.0;
        rd_gl_.control_time_us_ = dur_start_;
        dc_.tc_shm_->control_time_us_ = dur_start_;

        auto d2 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t2).count();

        auto t3 = chrono::steady_clock::now();
        //dc_.tc_shm_->t_cnt2 = stm_count_;
        //dc_.tc_shm_->t_cnt2 = cnt3;

        if (dc_.inityawSwitch)
            dc_.inityawSwitch = false;

        auto d3 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t2).count();

        auto t4 = chrono::steady_clock::now();

        auto d4 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t2).count();
        rd_gl_.state_ctime_total_ += (d1 + d2 + d3 + d4);
        rd_gl_.state_ctime_avg_ = rd_gl_.state_ctime_total_ / stm_count_;
        if (stm_count_ % 2000 == 0)
        {
            printf("%f, ecat cnt : %d, stm cnt : %d, dcm cnt : %d \n", control_time_, (int)dc_.tc_shm_->statusCount, stm_count_, (int)dc_.tcm_cnt);
        }

        if (d2 > 500)
        {
            printf("stm over 500, d1 : %ld, d2 : %ld, d3 : %ld, d4 : %ld\n", d1, d2, d3, d4);
        }

        for (int i = 0; i < MODEL_DOF; i++)
        {

            state_safety_before_[i] = state_safety_[i];
        }
        //printf("%d\n", rcv_tcnt);
        //printf("\x1b[A\x1b[A\33[2K\r");
        // if (rcv_tcnt % 33 == 0)
        // {
        //     printf("\33[2K\r");
        //     printf("%8d %8d avg : %7.3f max : %4d min : %4d, cnt : %4d, cnt2 : %4d, cnt3 : %4d ", rcv_tcnt, cycle_count_, avg, max, min, cnt, cnt2, cnt3);
        //     fflush(stdout);
        // }
    }
    cout << "StateManager Thread END" << endl;
}

void *StateManager::LoggerThread()
{
    bool activateLogger;

    dc_.nh.getParam("/tocabi_controller/log", activateLogger);

    //wait for both ecat are in control mode !
    while (!dc_.tc_shm_->shutdown)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (dc_.tc_shm_->controlModeLower && dc_.tc_shm_->controlModeUpper)
        {
            std::cout << "Logger : Both ECAT is now on CONTROL MODE! Logging start..." << std::endl;
            break;
        }
    }

    char torqueLogFile[] = "/home/dyros/tocabi_log/torque_elmo_log";
    char ecatStatusFile[] = "/home/dyros/tocabi_log/ecat_status_log";
    char torqueclogFile[] = "/home/dyros/tocabi_log/torque_command_log";
    char torqueActualLogFile[] = "/home/dyros/tocabi_log/torque_actual_log";
    char posLogFile[] = "/home/dyros/tocabi_log/pos_log";
    char velLogFile[] = "/home/dyros/tocabi_log/vel_log";

    ofstream torqueLog;
    torqueLog.open(torqueLogFile);
    torqueLog.fill(' ');

    ofstream torqueCommandLog;
    torqueCommandLog.open(torqueclogFile);

    ofstream torqueActualLog;
    torqueActualLog.open(torqueActualLogFile);

    ofstream ecatStatusLog;
    ecatStatusLog.open(ecatStatusFile);

    ofstream posLog;
    posLog.open(posLogFile);

    ofstream velLog;
    velLog.open(posLogFile);
    int log_count = 0;

    while (!dc_.tc_shm_->shutdown)
    {
        log_count++;
        if (log_count % 33 == 0)
        {
            PublishData();
        }
        ros::spinOnce();

        std::this_thread::sleep_for(std::chrono::microseconds(500));

        if (activateLogger)
        {
            torqueLog << (float)rd_gl_.control_time_us_ / 1000000.0 << "\t ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torqueLog << std::setfill(' ') << std::setw(5) << (int)dc_.tc_shm_->elmo_torque[i] << " ";
            }
            torqueLog << std::endl;

            torqueCommandLog << (float)rd_gl_.control_time_us_ / 1000000.0 << "\t ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torqueCommandLog << fixed << setprecision(4) << setw(8) << dc_.torque_command[i] << " ";
            }
            torqueCommandLog << std::endl;

            posLog << (float)rd_gl_.control_time_us_ / 1000000.0 << "\t ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                posLog << fixed << setprecision(4) << setw(8) << rd_gl_.q_[i] << " ";
            }
            posLog << std::endl;

            velLog << (float)rd_gl_.control_time_us_ / 1000000.0 << "\t ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                velLog << fixed << setprecision(4) << setw(8) << rd_gl_.q_dot_[i] << " ";
            }
            velLog << std::endl;

            torqueActualLog << (float)rd_gl_.control_time_us_ / 1000000.0 << "\t ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torqueActualLog << std::setfill(' ') << std::setw(6) << (int)dc_.tc_shm_->torqueActual[i] << " ";
            }
            torqueActualLog << std::endl;

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
    ecatStatusLog.close();
    torqueLog.close();

    std::cout << "Logger : END!" << std::endl;
}

void StateManager::SendCommand()
{

    static double torque_command[MODEL_DOF];
    while (dc_.t_c_)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    dc_.t_c_ = true;
    std::copy(dc_.torque_command, dc_.torque_command + MODEL_DOF, torque_command);
    static int rcv_c_count = dc_.control_command_count;

    dc_.t_c_ = false;
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
                std::cout << "Command not received for " << warning_cnt << "times " << std::endl;
                warning_cnt = 0;
            }
        }
        else if (prob_cnt != rcv_c_count)
        {
            warning_cnt = 0;
        }
    }

    rcv_c_count_before = rcv_c_count;

    const double maxTorque = 1000.0;
    const double rTime = 5.0;

    int maxTorqueCommand;

    if (dc_.torqueOnSwitch)
    {
        dc_.torqueOnSwitch = false;

        if (dc_.torqueOn)
        {
            std::cout << "torque is already on " << std::endl;
        }
        else
        {
            std::cout << "turning on ... " << std::endl;
            dc_.torqueOnTime = rd_gl_.control_time_;
            dc_.torqueOn = true;
            dc_.torqueRisingSeq = true;
        }
    }
    if (dc_.torqueOffSwitch)
    {
        dc_.torqueOffSwitch = false;

        if (dc_.torqueOn)
        {
            std::cout << "turning off ... " << std::endl;
            dc_.torqueOffTime = rd_gl_.control_time_;
            dc_.toruqeDecreaseSeq = true;
        }
        else
        {
            std::cout << "torque is already off" << std::endl;
        }
    }

    if (dc_.torqueOn)
    {
        if (dc_.torqueRisingSeq)
        {
            maxTorqueCommand = (int)(maxTorque * DyrosMath::minmax_cut((rd_gl_.control_time_ - dc_.torqueOnTime) / rTime, 0.0, 1.0));

            if (rd_gl_.control_time_ > dc_.torqueOnTime + rTime)
            {
                std::cout << "torque 100% ! " << std::endl;

                dc_.torqueRisingSeq = false;
            }
        }
        else if (dc_.toruqeDecreaseSeq)
        {

            maxTorqueCommand = (int)(maxTorque * (1 - DyrosMath::minmax_cut((rd_gl_.control_time_ - dc_.torqueOffTime) / rTime, 0.0, 1.0)));

            if (rd_gl_.control_time_ > dc_.torqueOffTime + rTime)
            {
                dc_.toruqeDecreaseSeq = false;

                std::cout << "torque 0% .. torque Off " << std::endl;

                dc_.torqueOn = false;
            }
        }
        else
        {
            maxTorqueCommand = (int)maxTorque;
        }
    }
    else
    {
        maxTorqueCommand = 0;
    }

    if (dc_.E1Switch) //Emergency stop
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
    if (dc_.E2Switch) //Emergency damping
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

            //Damping mode = true!
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
    dc_.tc_shm_->commanding.store(true);

    //std::fill(dc_.tc_shm_->commandMode, dc_.tc_shm_->commandMode + MODEL_DOF, 1);
    std::copy(torque_command, torque_command + MODEL_DOF, dc_.tc_shm_->torqueCommand);
    dc_.tc_shm_->maxTorque = maxTorqueCommand;
    static int cCount = 0;
    cCount++;
    dc_.tc_shm_->commandCount.store(cCount);
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
        std::cout << "Yaw Initialized : " << rd_.yaw << std::endl;
        rd_gl_.yaw_init = rd_.yaw;
    }

    //const tf2Scalar& r_,p_,y_;

    tf2::Quaternion q_mod;
    rd_.yaw = rd_.yaw - rd_gl_.yaw_init;

    q_mod.setRPY(rd_.roll, rd_.pitch, rd_.yaw);
    //tf2::Quaternion q_rot;
    //q_rot.setRPY(0, 0, -yaw_init);
    //q = q * q_rot;

    q_virtual_local_(3) = q_mod.getX();
    q_virtual_local_(4) = q_mod.getY();
    q_virtual_local_(5) = q_mod.getZ();
    q_virtual_local_(MODEL_DOF_VIRTUAL) = q_mod.getW();

    //q_virtual_local_ = q_virtual_local_yaw_initialized;
}

void StateManager::GetJointData()
{
    while (dc_.tc_shm_->statusWriting.load(std::memory_order_acquire))
    {
        if (dc_.tc_shm_->shutdown)
            break;
    }

    memcpy(q_a_, dc_.tc_shm_->pos, sizeof(float) * MODEL_DOF);
    memcpy(q_dot_a_, dc_.tc_shm_->vel, sizeof(float) * MODEL_DOF);

    q_ = Map<VectorQf>(q_a_, MODEL_DOF).cast<double>();
    q_dot_ = Map<VectorQf>(q_dot_a_, MODEL_DOF).cast<double>();

    q_virtual_local_.segment(6, MODEL_DOF) = q_;
    q_dot_virtual_local_.segment(6, MODEL_DOF) = q_dot_;

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
        q_virtual_local_(1) = 0.0; //dc_.tc_shm_->pos_virtual[1];
        q_virtual_local_(2) = 0.0; //dc_.tc_shm_->pos_virtual[2];

        q_dot_virtual_local_(0) = 0.0;
        q_dot_virtual_local_(1) = 0.0;
        q_dot_virtual_local_(2) = 0.0;
    }

    q_dot_virtual_local_(3) = dc_.tc_shm_->vel_virtual[3];
    q_dot_virtual_local_(4) = dc_.tc_shm_->vel_virtual[4];
    q_dot_virtual_local_(5) = dc_.tc_shm_->vel_virtual[5];

    q_virtual_local_(3) = dc_.tc_shm_->pos_virtual[3];
    q_virtual_local_(4) = dc_.tc_shm_->pos_virtual[4];
    q_virtual_local_(5) = dc_.tc_shm_->pos_virtual[5];
    q_virtual_local_(MODEL_DOF_VIRTUAL) = dc_.tc_shm_->pos_virtual[6];

    //memcpy(joint_state_, dc_.tc_shm_->status, sizeof(int) * MODEL_DOF);
    memcpy(state_elmo_, dc_.tc_shm_->ecat_status, sizeof(int8_t) * MODEL_DOF);
    memcpy(state_safety_, dc_.tc_shm_->safety_status, sizeof(int8_t) * MODEL_DOF);
    memcpy(state_zp_, dc_.tc_shm_->zp_status, sizeof(int8_t) * MODEL_DOF);
    for (int i = 0; i < MODEL_DOF; i++)
    {
        if (state_safety_[i] != state_safety_before_[i])
        {
            if (state_safety_[i] != 0)
            {
                dc_.positionControlSwitch = true;
                std::cout << "Safety Activated ! To Position Hold" << std::endl;
            }
        }
    }

    //RF_CF_FT.setZ

    //dc_.tc_shm_->pos
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

    double foot_plate_mass = 2.326;

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

    RF_CF_FT = rotrf * adt * RF_FT + adt2 * Wrench_foot_plate;
    //rd_gl_.ee_[1].contact_force_ft = RF_CF_FT;

    //RF_CF_FT_local = rotrf.inverse() * RF_CF_FT;

    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(-(link_local_[Left_Foot].contact_point - link_local_[Left_Foot].sensor_point)) * Matrix3d::Identity();

    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = link_local_[Left_Foot].rotm;
    rotrf.block(3, 3, 3, 3) = link_local_[Left_Foot].rotm;

    Vector3d LF_com(-0.0162, -0.00008, -0.1209);

    com2cp = link_local_[Left_Foot].contact_point - LF_com;

    adt2.setIdentity();
    adt2.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();
    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = foot_plate_mass * GRAVITY;

    LF_CF_FT = rotrf * adt * LF_FT + adt2 * Wrench_foot_plate;
    //dc.tocabi_.ee_[0].contact_force_ft = LF_CF_FT;

    //LF_CF_FT_local = rotrf.inverse() * LF_CF_FT;
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

    memcpy(&rd_dst.model_, &model_global_, sizeof(RigidBodyDynamics::Model));

    for (int i = 0; i < (LINK_NUMBER + 1); i++)
    {
        memcpy(&rd_dst.link_[i].jac, &link_[i].jac, sizeof(Matrix6Vf));
        memcpy(&rd_dst.link_[i].jac_com, &link_[i].jac_com, sizeof(Matrix6Vf));

        memcpy(&rd_dst.link_[i].xpos, &link_[i].xpos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].xipos, &link_[i].xipos, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].rotm, &link_[i].rotm, sizeof(Matrix3d));
        memcpy(&rd_dst.link_[i].v, &link_[i].v, sizeof(Vector3d));
        memcpy(&rd_dst.link_[i].w, &link_[i].w, sizeof(Vector3d));

        //xpos xipos rotm v w
    }

    memcpy(&rd_dst.A_, &A_, sizeof(MatrixVVd));
    memcpy(&rd_dst.A_inv_, &A_inv_, sizeof(MatrixVVd));
    memcpy(&rd_dst.Motor_inertia, &Motor_inertia, sizeof(MatrixVVd));
    memcpy(&rd_dst.Motor_inertia_inverse, &Motor_inertia_inverse, sizeof(MatrixVVd));
    memcpy(&rd_dst.q_, &q_, sizeof(VectorQd));
    memcpy(&rd_dst.q_dot_, &q_dot_, sizeof(VectorQd));
    memcpy(&rd_dst.q_virtual_, &q_virtual_, sizeof(VectorQVQd));
    memcpy(&rd_dst.q_dot_virtual_, &q_dot_virtual_, sizeof(VectorVQd));
    memcpy(&rd_dst.q_ddot_virtual_, &q_ddot_virtual_, sizeof(VectorVQd));

    rd_dst.roll = rd_.roll;
    rd_dst.pitch = rd_.pitch;
    rd_dst.yaw = rd_.yaw;

    if (!rd_dst.firstCalc)
    {

        memcpy(&rd_dst.link_, link_, (LINK_NUMBER + 1) * sizeof(LinkData));

        rd_dst.firstCalc = true;
    }

    rd_dst.control_time_ = control_time_;

    rd_dst.tp_state_ = rd_.tp_state_;
    dc_.triggerThread1 = true;
}

void StateManager::UpdateKinematics_local(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

    //local kinematics update sequence

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
    //ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
    /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

    // Update Kinematics : Total 127 us

    // sector 1 Start : rbdl update
    A_temp_.setZero();
    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_l, q_virtual_f, A_temp_, false);
    A_ = A_temp_;
    // sector 1 End : 30 us

    // sector 2 start : link pos/vel/jac update
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        link_p[i].UpdatePosition(model_l, q_virtual_f);
        link_p[i].UpdateJacobian(model_l, q_virtual_f, q_dot_virtual_f);
    }
    // sector 2 end : 57 us

    // sector 3 start : mass matrix inverse
    //A_inv_ = A_.inverse();
    A_inv_ = A_.llt().solve(Eigen::MatrixVVd::Identity());
    // sector 3 end : 39 us

    // sector 4 start : com calculation
    Eigen::Matrix3Vf jacobian_com;
    jacobian_com.setZero();
    for (int i = 0; i < LINK_NUMBER; i++)
    {
        jacobian_com += link_p[i].jac_com.topRows(3) * link_p[i].mass / total_mass_;
    }
    link_p[COM_id].mass = total_mass_;
    link_p[COM_id].xpos.setZero();
    for (int i = 0; i < LINK_NUMBER; i++)
        link_p[COM_id].xpos += link_p[i].xipos * link_p[i].mass / total_mass_;
    link_p[COM_id].v = jacobian_com.cast<double>() * q_dot_virtual_f;
    link_p[COM_id].w = link_p[Pelvis].w;
    link_p[COM_id].rotm = link_p[Pelvis].rotm;
    link_p[COM_id].jac.setZero(6, MODEL_DOF_VIRTUAL);
    //link_p[COM_id].jac.block(0, 0, 2, MODEL_DOF + 6) = jacobian_com.block(0, 0, 2, MODEL_DOF + 6) / com_.mass;
    //link_p[COM_id].jac.block(2, 0, 4, MODEL_DOF + 6) = link_p[Pelvis].jac.block(2, 0, 4, MODEL_DOF + 6);
    link_p[COM_id].jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com.block(0, 0, 3, MODEL_DOF_VIRTUAL);
    link_p[COM_id].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = link_p[Pelvis].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);

    link_p[COM_id].jac_com.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jacobian_com;
    //link_p[COM_id].xpos(2) = link_p[Pelvis].xpos(2);
    // sector 4 end : 3 us
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

        if (dc_.stateEstimateSwitch)
        {
            contact_right = false;
            contact_left = false;
            std::cout << "state Estimation Initialized" << std::endl;
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
                std::cout << control_time_ << "  right foot contact initialized" << std::endl;
                RF_contact_pos_holder = RF_global_contact_pos;
                RF_CP_est_holder_before = RF_CP_est_holder;
                RF_CP_est_holder = RF_CP_est;
            }
            else
            {
                std::cout << control_time_ << "  right foot contact disabled" << std::endl;
            }
        }
        if (contact_left != local_LF_contact)
        {
            left_change = true;
            if (local_LF_contact)
            {
                std::cout << control_time_ << "  left foot contact initialized" << std::endl;
                LF_contact_pos_holder = LF_global_contact_pos;
                LF_CP_est_holder_before = LF_CP_est_holder;
                LF_CP_est_holder = LF_CP_est;
            }
            else
            {
                std::cout << control_time_ << "  left foot contact disabled" << std::endl;
            }
        }

        if (dc_.stateEstimateSwitch)
        {

            LF_contact_pos_holder(2) = 0.0;
            RF_contact_pos_holder(2) = 0.0;
            dc_.stateEstimateSwitch = false;
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

        //Vector3d es_zmp;

        //es_zmp = dc.tocabi_.com_.pos - dc.tocabi_.com_.pos(2)/9.81*

        contact_right = local_RF_Contact;
        contact_left = local_LF_contact;

        rf_cp_m = RF_fixed_contact_pos - RF_contact_pos_holder;
        lf_cp_m = LF_fixed_contact_pos - LF_contact_pos_holder;

        double dr, dl;
        //dr =

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

        if (contact_right && contact_left)
        {
            mod_base_pos = rf_cp_m * rf_s_ratio + lf_cp_m * lf_s_ratio;
            //mod_base_pos(2) = mod_base_pos(2) + ((link_[Right_Foot].xpos(2) + link_[Right_Foot].contact_point(2)) * rf_s_ratio/ (rf_s_ratio + lf_s_ratio) + (link_[Left_Foot].xpos(2) + link_[Left_Foot].contact_point(2)) * lf_s_ratio / (rf_s_ratio + lf_s_ratio));
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

        //Pelvis Velocity Complementary filter
        //v = alpha *(pelv_imu_acc * dt + v_before) + (1-alpha)*mb_v

        Vector3d imu_acc_dat;
        imu_acc_dat = link_local_[Pelvis].rotm * rd_.imu_lin_acc;

        imu_acc_dat = imu_acc_dat - imu_init;

        double dt = 0.0005;
        double tau = 0.6;
        double alpha = tau / (tau + dt);

        pelv_v = alpha * (imu_acc_dat * dt + pelv_v_before) + (1 - alpha) * mod_base_vel;
        pelv_v_before = pelv_v;
        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;

        //std::cout<<"pelv_v es : "<<pelv_v_before.transpose()<<" imu acc : "<<imu_acc_dat.transpose()<<"  imu init : "<<imu_init.transpose() <<" imu lin acc : "<<imu_lin_acc.transpose() <<" mod base : "<<mod_base_vel.transpose()<<std::endl;

        pelv_x = alpha * (pelv_v * dt + imu_acc_dat * dt * dt * 0.5 + pelv_x_before) + (1 - alpha) * (-mod_base_pos);
        pelv_x_before = pelv_x;

        pelv_anga = (q_dot_virtual_.segment<3>(3) - rd_.imu_ang_vel_before) * 2000;
        rd_.imu_ang_vel_before = q_dot_virtual_.segment<3>(3);

        for (int i = 0; i < 3; i++)
        {
            q_virtual_(i) = -mod_base_pos(i);
            //q_dot_virtual_(i) = pelv_v(i);

            q_dot_virtual_(i) = mod_base_vel(i);

            q_ddot_virtual_(i) = imu_acc_dat(i);
            q_ddot_virtual_(i + 3) = pelv_anga(i);
        }

        // fr_msg.data[6] = pelv_v[0];
        // fr_msg.data[7] = pelv_v[1];
        // fr_msg.data[8] = pelv_v[2];
        // fr_msg.data[9] = imu_acc_dat[0];
        // fr_msg.data[10] = imu_acc_dat[1];
        // fr_msg.data[11] = imu_acc_dat[2];

        //acceleration calculation!
        //q_ddot_virtual_ = (q_dot_virtual_ - q_dot_virtual_before) / ((double)dc.ctime / 1000000.0);
        //q_dot_virtual_before = q_dot_virtual_;

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
        //quat_before = imu_quat;
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
    //RigidBodyDynamics::NonlinearEffects(model_local_,)
}

void StateManager::PublishData()
{
    timer_msg_.data = control_time_;

    timer_pub_.publish(timer_msg_);

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
        joint_state_msg_.effort[i] = rd_gl_.torque_desired[i];
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

    point_pub_msg_.polygon.points[4].x = tr_; //rpy
    point_pub_msg_.polygon.points[4].y = tp_; //rpy
    point_pub_msg_.polygon.points[4].z = ty_; //rpy

    point_pub_msg_.polygon.points[5].x = link_[Right_Hand].xpos(0);
    point_pub_msg_.polygon.points[5].y = link_[Right_Hand].xpos(1);
    point_pub_msg_.polygon.points[5].z = link_[Right_Hand].xpos(2);

    point_pub_msg_.polygon.points[6].x = link_[Left_Hand].xpos(0);
    point_pub_msg_.polygon.points[6].y = link_[Left_Hand].xpos(1);
    point_pub_msg_.polygon.points[6].z = link_[Left_Hand].xpos(2);

    point_pub_msg_.polygon.points[7].x = 0.0; //zmp
    point_pub_msg_.polygon.points[7].y = 0.0;
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

    //geometry_msgs::Point32 p_vel, p_vel_virtual_;

    point_pub_msg_.polygon.points[11].x = link_[Pelvis].v(0);
    point_pub_msg_.polygon.points[11].y = link_[Pelvis].v(1);
    point_pub_msg_.polygon.points[11].z = link_[Pelvis].v(2);

    point_pub_msg_.polygon.points[12].x = dc_.tc_shm_->vel_virtual[0];
    point_pub_msg_.polygon.points[12].y = dc_.tc_shm_->vel_virtual[1];
    point_pub_msg_.polygon.points[12].z = dc_.tc_shm_->vel_virtual[2];

    point_pub_.publish(point_pub_msg_);

    Eigen::Quaterniond q_head_(DyrosMath::rotateWithZ(-rd_.yaw) * link_[Head].rotm);

    head_pose_msg_.orientation.w = q_head_.w();
    head_pose_msg_.orientation.x = q_head_.x();
    head_pose_msg_.orientation.y = q_head_.y();
    head_pose_msg_.orientation.z = q_head_.z();

    head_pose_pub_.publish(head_pose_msg_);

    //head_pose_msg_.orientation.

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
        syspub_msg.data[1] = 3;
        syspub_msg.data[2] = dc_.tc_shm_->ft_state;
        syspub_msg.data[3] = 3;
    }
    syspub_msg.data[4] = rd_gl_.semode;
    if (rd_gl_.tc_run) //tc on warn error off
    {
        syspub_msg.data[5] = 0; //on
    }
    else
    {
        syspub_msg.data[5] = 3; //off
    }

    gui_state_pub_.publish(syspub_msg);
    //
    //memcpy(joint_state_before_, joint_state_, sizeof(int) * MODEL_DOF);
}

void StateManager::SimCommandCallback(const std_msgs::StringConstPtr &msg)
{

    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        //parameterInitialize();
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
        //dc.semode_init = true;
        mujoco_reset = true;
    }

    if (buf == "terminate")
    {
        dc_.tc_shm_->shutdown = true;
    }
}

void StateManager::GuiCommandCallback(const std_msgs::StringConstPtr &msg)
{
    std::cout << "Received msg from GUI : " << msg->data << std::endl;
    //Receiving Command from GUI!

    if (msg->data == "torqueon")
    {
        dc_.torqueOnSwitch = true;
    }
    else if (msg->data == "torqueoff")
    {
        dc_.torqueOffSwitch = true;
    }
    else if (msg->data == "E0")
    {
        std::cout << "Emergency Switch pressed! " << std::endl;
        dc_.emergencySwitch = true;
        dc_.tc_shm_->emergencyOff = true;
    }
    else if (msg->data == "E1")
    {
        std::cout << "Emergency Stop Activated !" << std::endl;
        dc_.E1Switch = true;
    }
    else if (msg->data == "E2")
    {
        std::cout << "Emergency Damping mode Active ! " << std::endl;
        dc_.E2Switch = true;
    }
    else if (msg->data == "gravity")
    {
        rd_gl_.tc_run = false;
        rd_gl_.pc_mode = false;
    }
    else if (msg->data == "inityaw")
    {
        dc_.inityawSwitch = true;
    }
    else if (msg->data == "ftcalib")
    {
        dc_.ftcalibSwtich = true;
    }
    else if (msg->data == "imureset")
    {
        dc_.imuResetSwtich = true;
    }
    else if (msg->data == "stateestimation")
    {
        if (rd_gl_.semode)
        {
            std::cout << "stateestimation off" << std::endl;
            rd_gl_.semode = false;
        }
        else
        {
            std::cout << "stateestimation on" << std::endl;
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
            std::cout << "safety checking disabled!" << std::endl;
            dc_.tc_shm_->safety_reset_lower_signal = true;
            dc_.tc_shm_->safety_reset_upper_signal = true;
        }
        else
        {

            std::cout << "safety checking enabled!" << std::endl;
        }
    }
    else if (msg->data == "ecatinit")
    {
        dc_.tc_shm_->upper_init_signal = true;
    }
    else if (msg->data == "ecatinitlower")
    {
        dc_.tc_shm_->low_init_signal = true;
    }
    else if (msg->data == "ecatinitwaist")
    {
        dc_.tc_shm_->waist_init_signal = true;
    }
    else if (msg->data == "simvirtualjoint")
    {
        dc_.useSimVirtual = !dc_.useSimVirtual;

        if (dc_.useSimVirtual)
        {
            rd_gl_.semode = false;
        }
    }
    else if (msg->data == "positioncontrol")
    {
        dc_.positionControlSwitch = true;
    }
    else if (msg->data == "forceload")
    {
        dc_.tc_shm_->force_load_saved_signal = true;
    }

    //Controlling GUI
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

    //std::cout<<str_;

    va_end(lst);
}
