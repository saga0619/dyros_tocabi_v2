#include "tocabi_controller/state_manager.h"
#include "fstream"
#include "algorithm"

using namespace std;
using namespace TOCABI;

StateManager::StateManager(DataContainer &dc_global) : dc_(dc_global), rd_gl_(dc_global.rd_)
{
    string urdf_path;

    ros::param::get("/tocabi_controller/urdf_path", urdf_path);
    rd_.InitModelData(urdf_path.c_str(), true, false);
    dc_.rd_holder_.InitModelData(urdf_path.c_str(), true, false);
    rd_gl_.InitModelData(urdf_path.c_str(), true, false);

    bool verbose = false;

    // rd_gl_.InitModelData(urdf_path.c_str(), true, false);

    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_local_, true, verbose);
    // RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_global_, true, verbose);

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

        total_mass_ = rd_.total_mass_;

        for (int i = 0; i < LINK_NUMBER; i++)
        {
            link_local_[i].Initialize(model_local_, link_id_[i]);
        }

        link_local_[Right_Foot].contact_point << 0.03, 0, -0.1585;
        link_local_[Right_Foot].sensor_point << 0.0, 0.0, -0.09;
        link_local_[Left_Foot].contact_point << 0.03, 0, -0.1585;
        link_local_[Left_Foot].sensor_point << 0.0, 0.0, -0.09;

        link_local_[Right_Hand].contact_point << 0, 0.0, -0.035;
        link_local_[Right_Hand].sensor_point << 0.0, 0.0, 0.0;
        link_local_[Left_Hand].contact_point << 0, 0.0, -0.035;
        link_local_[Left_Hand].sensor_point << 0.0, 0.0, 0.0;

        // memcpy(link_local_, link_, sizeof(LinkData) * LINK_NUMBER);
    }

    Vector3d foot_contact_point;
    foot_contact_point << 0.03, 0, -0.1585;
    Vector3d foot_contact_vector;
    foot_contact_vector << 0, 0, 1;

    Vector3d hand_contact_point;
    hand_contact_point << 0.0, 0.0, -0.035;

    rd_.AddContactConstraint(6, DWBC::CONTACT_6D, foot_contact_point, foot_contact_vector, 0.15, 0.075);
    rd_.AddContactConstraint(12, DWBC::CONTACT_6D, foot_contact_point, foot_contact_vector, 0.15, 0.075);
    rd_.AddContactConstraint(Left_Hand, DWBC::CONTACT_6D, hand_contact_point, foot_contact_vector, 0.04, 0.04);
    rd_.AddContactConstraint(Right_Hand, DWBC::CONTACT_6D, hand_contact_point, foot_contact_vector, 0.04, 0.04);

    // dc_.rd_holder_.AddContactConstraint(6, DWBC::CONTACT_6D, foot_contact_point, foot_contact_vector, 0.15, 0.075);
    // dc_.rd_holder_.AddContactConstraint(12, DWBC::CONTACT_6D, foot_contact_point, foot_contact_vector, 0.15, 0.075);
    // dc_.rd_holder_.AddContactConstraint(Left_Hand, DWBC::CONTACT_6D, hand_contact_point, foot_contact_vector, 0.04, 0.04);
    // dc_.rd_holder_.AddContactConstraint(Right_Hand, DWBC::CONTACT_6D, hand_contact_point, foot_contact_vector, 0.04, 0.04);

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

    com_status_pub_ = dc_.nh.advertise<std_msgs::Float32MultiArray>("/tocabi/comstates", 100);

    com_status_msg_.data.resize(17);

    point_pub_msg_.polygon.points.resize(21);
    syspub_msg.data.resize(8);
    elmo_status_msg_.data.resize(MODEL_DOF * 3);

    q_dot_virtual_local_.setZero();
    q_ddot_virtual_local_.setZero();
}

StateManager::~StateManager()
{
    cout << "StateManager Terminate" << endl;
}

void *StateManager::StateThread()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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

        auto dur_start_ = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_start).count();
        control_time_ = rcv_tcnt / 2000.0;

        // local kinematics update : 33.7 us // w/o march native 20 us
        UpdateKinematics_local(model_local_, link_local_, q_virtual_local_, q_dot_virtual_local_, q_ddot_virtual_local_);

        GetSensorData();

        auto d1 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t1).count();

        auto t2 = chrono::steady_clock::now();
        StateEstimate();

        auto d2 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t2).count();

        auto t3 = chrono::steady_clock::now();

        // global kinematics update : 127 us //w/o march native 125 us

        // std::cout << q_virtual_.transpose() << std::endl;
        rd_.UpdateKinematics(q_virtual_, q_dot_virtual_, q_ddot_virtual_);

        // UpdateKinematics(model_global_, link_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);

        // UpdateCMM(rd_, link_);

        auto d3 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t3).count();

        auto t4 = chrono::steady_clock::now();

        if (!rd_gl_.firstCalc)
        {
            rd_gl_.firstCalc = true;
        }

        StoreState(rd_gl_); // 6.2 us //w/o march native 8us

        // MeasureTime(dc_.stm_cnt, d1, d2);

        // rd_gl_.control_time_ = dur_start_ / 1000000.0;
        rd_gl_.control_time_us_ = dur_start_;
        dc_.tc_shm_->control_time_us_ = dur_start_;

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
            std::cout << " STATE : Turning off ... " << std::endl;
            dc_.torqueOffTime = rd_gl_.control_time_;
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
            if (rd_gl_.control_time_ <= dc_.torqueOnTime + rTime1)
            {
                torqueRatio = rat1 * DyrosMath::minmax_cut((rd_gl_.control_time_ - dc_.torqueOnTime) / rTime1, 0.0, 1.0);
            }
            if (rd_gl_.control_time_ > dc_.torqueOnTime + rTime1 && rd_gl_.control_time_ <= dc_.torqueOnTime + rTime1 + rTime2)
            {
                torqueRatio = rat1 + rat2 * DyrosMath::minmax_cut((rd_gl_.control_time_ - dc_.torqueOnTime - rTime1) / rTime2, 0.0, 1.0);
            }
            else if (rd_gl_.control_time_ > dc_.torqueOnTime + rTime1 + rTime2)
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

            if (rd_gl_.control_time_ <= dc_.torqueOffTime + rTime2)
            {
                torqueRatio = (1 - rat2 * DyrosMath::minmax_cut((rd_gl_.control_time_ - dc_.torqueOffTime) / rTime2, 0.0, 1.0));
            }
            if (rd_gl_.control_time_ > dc_.torqueOffTime + rTime2 && rd_gl_.control_time_ <= dc_.torqueOffTime + rTime2 + rTime1)
            {
                torqueRatio = (1 - rat2 - rat1 * DyrosMath::minmax_cut((rd_gl_.control_time_ - dc_.torqueOffTime - rTime2) / rTime1, 0.0, 1.0));
            }
            else if (rd_gl_.control_time_ > dc_.torqueOffTime + rTime2 + rTime1)
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
    static int cCount = 0;
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

    q_ = Map<VectorQf>(q_a_, MODEL_DOF).cast<double>();
    q_dot_ = Map<VectorQf>(q_dot_a_, MODEL_DOF).cast<double>();

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

    static Vector6d RF_FT_LPF = RF_FT;
    static Vector6d LF_FT_LPF = LF_FT;

    for (int i = 0; i < 6; i++)
    {
        LF_FT_LPF(i) = DyrosMath::lpf(LF_FT(i), LF_FT_LPF(i), 2000, 60);

        RF_FT_LPF(i) = DyrosMath::lpf(RF_FT(i), RF_FT_LPF(i), 2000, 60);
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

    adt2.setIdentity();
    adt2.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();
    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = foot_plate_mass * GRAVITY;

    LF_CF_FT = rotrf * adt * LF_FT_LPF - adt2 * Wrench_foot_plate;

    // dc.tocabi_.ee_[0].contact_force_ft = LF_CF_FT;

    // LF_CF_FT_local = rotrf.inverse() * LF_CF_FT;
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
    dc_.state_locker_.producer_lock();

    rd_.CopyKinematicsData(rd_dst);

    rd_dst.q_ = q_;
    rd_dst.q_dot_ = q_dot_;
    rd_dst.q_virtual_ = q_virtual_;
    rd_dst.q_dot_virtual_ = q_dot_virtual_;
    rd_dst.q_ddot_virtual_ = q_ddot_virtual_;
    rd_dst.torque_elmo_ = torque_elmo_;

    rd_dst.CMM = rd_.CMM;

    rd_dst.roll = rd_.roll;
    rd_dst.pitch = rd_.pitch;
    rd_dst.yaw = rd_.yaw;

    rd_dst.control_time_ = control_time_;

    rd_dst.tp_state_ = rd_.tp_state_;

    rd_dst.LF_FT = LF_FT;
    rd_dst.RF_FT = RF_FT;

    rd_dst.LF_CF_FT = LF_CF_FT;
    rd_dst.RF_CF_FT = RF_CF_FT;

    // std::cout << "Command to TocabiController" << std::endl;

    dc_.state_locker_.producer_ready();
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
    // A_inv_ = A_.inverse();
    A_inv_ = A_.llt().solve(Eigen::MatrixVVd::Identity());
    // sector 3 end : 39 us

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

    // RigidBodyDynamics::CalcCenterOfMass(model_l, )

    // RigidBodyDynamics::UpdateKinematicsCustom()

    link_p[COM_id].v = jacobian_com.cast<double>() * q_dot_virtual_f;
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

    // for (int i = 0; i < LINK_NUMBER; i++)
    // {
    //     x_com_ = link_p[i].xpos - link_p[COM_id].xpos;

    //     c_product_.setZero();

    //     c_product_.block(0, 0, 3, 1) = x_com_ * x_com_(0);
    //     c_product_.block(0, 1, 3, 1) = x_com_ * x_com_(1);
    //     c_product_.block(0, 2, 3, 1) = x_com_ * x_com_(2);

    //     i_com_ += link_p[i].rotm * link_p[i].inertia * link_[i].rotm.transpose() + link_p[i].mass * (Eigen::Matrix3d::Identity() * (x_com_(0) * x_com_(0) + x_com_(1) * x_com_(1) + x_com_(2) * x_com_(2)) - c_product_);
    // }

    // link_[COM_id].inertia = i_com_;

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

        rd_.link_[Right_Foot].GetPointPos(rd_.model_, q_virtual_, q_dot_virtual_, RF_contactpoint_internal_pos, RF_global_contact_pos);
        rd_.link_[Left_Foot].GetPointPos(rd_.model_, q_virtual_, q_dot_virtual_, LF_contactpoint_internal_pos, LF_global_contact_pos);

        // link_[Right_Foot].GetPointPos(model_global_, q_virtual_, q_dot_virtual_, RF_contactpoint_internal_pos, RF_global_contact_pos, RF_global_contact_vel);
        // link_[Left_Foot].GetPointPos(model_global_, q_virtual_, q_dot_virtual_, LF_contactpoint_internal_pos, LF_global_contact_pos, LF_global_contact_vel);

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
            local_LF_contact = rd_gl_.cc_[0].contact;
            local_RF_Contact = rd_gl_.cc_[1].contact;
        }

        bool left_change, right_change;
        left_change = false;
        right_change = false;
        if (contact_right != local_RF_Contact)
        {
            right_change = true;
            if (local_RF_Contact)
            {
                std::cout << control_time_ << " STATE : right foot contact initialized" << std::endl;
                RF_contact_pos_holder = RF_global_contact_pos;
                RF_CP_est_holder_before = RF_CP_est_holder;
                RF_CP_est_holder = RF_CP_est;
            }
            else
            {
                std::cout << control_time_ << " STATE : right foot contact disabled" << std::endl;
            }
        }
        if (contact_left != local_LF_contact)
        {
            left_change = true;
            if (local_LF_contact)
            {
                std::cout << control_time_ << " STATE : left foot contact initialized" << std::endl;
                LF_contact_pos_holder = LF_global_contact_pos;
                LF_CP_est_holder_before = LF_CP_est_holder;
                LF_CP_est_holder = LF_CP_est;
            }
            else
            {
                std::cout << control_time_ << " STATE : left foot contact disabled" << std::endl;
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

        for (int i = 0; i < 3; i++)
        {
            q_virtual_(i) = -mod_base_pos(i);
            // q_dot_virtual_(i) = pelv_v(i);

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

    point_pub_msg_.polygon.points[0].x = rd_.link_[COM_id].xpos(0);
    point_pub_msg_.polygon.points[0].y = rd_.link_[COM_id].xpos(1);
    point_pub_msg_.polygon.points[0].z = rd_.link_[COM_id].xpos(2);

    point_pub_msg_.polygon.points[1].x = rd_.link_[Right_Foot].xpos(0);
    point_pub_msg_.polygon.points[1].y = rd_.link_[Right_Foot].xpos(1);
    point_pub_msg_.polygon.points[1].z = rd_.link_[Right_Foot].xpos(2);

    point_pub_msg_.polygon.points[2].x = rd_.link_[Left_Foot].xpos(0);
    point_pub_msg_.polygon.points[2].y = rd_.link_[Left_Foot].xpos(1);
    point_pub_msg_.polygon.points[2].z = rd_.link_[Left_Foot].xpos(2);

    point_pub_msg_.polygon.points[3].x = rd_.link_[Pelvis].xpos(0);
    point_pub_msg_.polygon.points[3].y = rd_.link_[Pelvis].xpos(1);
    point_pub_msg_.polygon.points[3].z = rd_.link_[Pelvis].xpos(2);

    double tr_, tp_, ty_;

    DyrosMath::rot2Euler_tf2(rd_.link_[Pelvis].rotm, tr_, tp_, ty_);

    point_pub_msg_.polygon.points[4].x = tr_; // rpy
    point_pub_msg_.polygon.points[4].y = tp_; // rpy
    point_pub_msg_.polygon.points[4].z = ty_; // rpy

    point_pub_msg_.polygon.points[5].x = rd_.link_[Right_Hand].xpos(0);
    point_pub_msg_.polygon.points[5].y = rd_.link_[Right_Hand].xpos(1);
    point_pub_msg_.polygon.points[5].z = rd_.link_[Right_Hand].xpos(2);

    point_pub_msg_.polygon.points[6].x = rd_.link_[Left_Hand].xpos(0);
    point_pub_msg_.polygon.points[6].y = rd_.link_[Left_Hand].xpos(1);
    point_pub_msg_.polygon.points[6].z = rd_.link_[Left_Hand].xpos(2);

    point_pub_msg_.polygon.points[7].x = rd_gl_.zmp_global_(0);
    point_pub_msg_.polygon.points[7].y = rd_gl_.zmp_global_(1);
    point_pub_msg_.polygon.points[7].z = 0.0;

    DyrosMath::rot2Euler_tf2(rd_.link_[Left_Foot].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[8].x = tr_;
    point_pub_msg_.polygon.points[8].y = tp_;
    point_pub_msg_.polygon.points[8].z = ty_;

    DyrosMath::rot2Euler_tf2(rd_.link_[Right_Foot].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[9].x = tr_;
    point_pub_msg_.polygon.points[9].y = tp_;
    point_pub_msg_.polygon.points[9].z = ty_;

    DyrosMath::rot2Euler_tf2(rd_.link_[Upper_Body].rotm, tr_, tp_, ty_);
    point_pub_msg_.polygon.points[10].x = tr_;
    point_pub_msg_.polygon.points[10].y = tp_;
    point_pub_msg_.polygon.points[10].z = ty_;

    // geometry_msgs::Point32 p_vel, p_vel_virtual_;

    point_pub_msg_.polygon.points[11].x = rd_.link_[Pelvis].v(0);
    point_pub_msg_.polygon.points[11].y = rd_.link_[Pelvis].v(1);
    point_pub_msg_.polygon.points[11].z = rd_.link_[Pelvis].v(2);

    point_pub_msg_.polygon.points[12].x = dc_.tc_shm_->vel_virtual[0];
    point_pub_msg_.polygon.points[12].y = dc_.tc_shm_->vel_virtual[1];
    point_pub_msg_.polygon.points[12].z = dc_.tc_shm_->vel_virtual[2];

    double zx, zy;

    zx = -LF_CF_FT(4) / LF_CF_FT(2);
    zy = LF_CF_FT(3) / LF_CF_FT(2);

    zx = DyrosMath::minmax_cut(zx, -0.3, 0.3);
    zy = DyrosMath::minmax_cut(zy, -0.3, 0.3);

    point_pub_msg_.polygon.points[13].x = rd_gl_.cc_[0].xc_pos(0) + zx;
    point_pub_msg_.polygon.points[13].y = rd_gl_.cc_[0].xc_pos(1) + zy;
    point_pub_msg_.polygon.points[13].z = 0.0;

    // std::cout << LF_CF_FT.transpose() << rd_gl_.ee_[0].xpos_contact.transpose() << std::endl;

    zx = -RF_CF_FT(4) / RF_CF_FT(2);
    zy = RF_CF_FT(3) / RF_CF_FT(2);

    zx = DyrosMath::minmax_cut(zx, -0.3, 0.3);
    zy = DyrosMath::minmax_cut(zy, -0.3, 0.3);

    point_pub_msg_.polygon.points[14].x = rd_gl_.cc_[1].xc_pos(0) + zx;
    point_pub_msg_.polygon.points[14].y = rd_gl_.cc_[1].xc_pos(1) + zy;
    point_pub_msg_.polygon.points[14].z = 0.0;

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

    point_pub_msg_.polygon.points[19].x = rd_.link_[COM_id].v(1);
    point_pub_msg_.polygon.points[19].y = -link_local_[Left_Foot].v(1);
    point_pub_msg_.polygon.points[19].z = -link_local_[Right_Foot].v(1);

    // static double com_pos_before = 0;

    point_pub_msg_.polygon.points[20].x = pelvis_velocity_estimate_(1);
    point_pub_msg_.polygon.points[20].y = link_local_[Left_Foot].v(1);
    point_pub_msg_.polygon.points[20].z = link_local_[Right_Foot].v(1);

    // com_pos_before = link_[COM_id].xpos(1);

    point_pub_.publish(point_pub_msg_);

    Eigen::Quaterniond q_head_(DyrosMath::rotateWithZ(-rd_.yaw) * rd_.link_[Head].rotm);

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
    if (rd_gl_.tc_run) // tc on warn error off
    {
        syspub_msg.data[5] = 0; // on
    }
    else
    {
        syspub_msg.data[5] = 3; // off
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
    else if (msg->data == "imureset")
    {
        dc_.imuResetSwtich = true;
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
        }
        else
        {
            std::cout << "lowerbody activate" << std::endl;
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
            rd_gl_.semode = false;
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

    std::string torqueLogFile = "/home/dyros/tocabi_log/torque_elmo_log";
    std::string ecatStatusFile = "/home/dyros/tocabi_log/ecat_status_log";
    std::string torqueclogFile = "/home/dyros/tocabi_log/torque_command_log";
    std::string torqueActualLogFile = "/home/dyros/tocabi_log/torque_actual_log";
    std::string posLogFile = "/home/dyros/tocabi_log/pos_log";
    std::string velLogFile = "/home/dyros/tocabi_log/vel_log";
    std::string maskLogFile = "/home/dyros/tocabi_log/mask_log";

    ofstream torqueLog;
    ofstream torqueCommandLog;
    ofstream torqueActualLog;
    ofstream maskLog;
    ofstream ecatStatusLog;
    ofstream posLog;
    ofstream velLog;

    int log_count = 0;
    int pub_count = 0;
    int s_count = 0;

    timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    int record_seconds = 120;

    int record_tick = record_seconds * 2000;

    long current_time = rd_gl_.control_time_us_;

    while (!dc_.tc_shm_->shutdown)
    {
        pub_count++;
        if (pub_count % 33 == 0)
        {
            if (current_time < rd_gl_.control_time_us_)
            {
                PublishData();
                current_time = rd_gl_.control_time_us_;
            }
        }
        ros::spinOnce();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        ts.tv_nsec += 500000;

        if (ts.tv_nsec >= 1000000000)
        {
            ts.tv_nsec -= 1000000000;
            ts.tv_sec++;
        }

        if (activateLogger && (!startLogger))
        {
            if (dc_.tc_shm_->controlModeLower && dc_.tc_shm_->controlModeUpper)
            {
                startLogger = true;
            }
        }

        if (startLogger)
        {
            if (log_count % record_tick == 0)
            {
                std::string apd_;
                std::string cpd_;
                if (s_count % 2 == 0)
                {
                    apd_ = "0";
                    cpd_ = "1";
                    std::cout << "LOGGER : Open Log Files : 0" << std::endl;
                }
                else
                {
                    apd_ = "1";
                    cpd_ = "0";
                    std::cout << "LOGGER : Open Log Files : 1" << std::endl;
                }

                if (s_count > 0)
                {
                    torqueLog.close();
                    torqueCommandLog.close();
                    torqueActualLog.close();
                    maskLog.close();
                    ecatStatusLog.close();
                    posLog.close();
                    velLog.close();
                }

                torqueLog.open((torqueLogFile + apd_).c_str());
                torqueLog.fill(' ');
                torqueCommandLog.open((torqueclogFile + apd_).c_str());
                torqueActualLog.open((torqueActualLogFile + apd_).c_str());
                maskLog.open((maskLogFile + apd_).c_str());
                ecatStatusLog.open((ecatStatusFile + apd_).c_str());
                posLog.open((posLogFile + apd_).c_str());
                velLog.open((velLogFile + apd_).c_str());
                s_count++;
            }
            log_count++;

            torqueLog << (float)rd_gl_.control_time_us_ / 1000000.0 << " ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torqueLog << (int)dc_.tc_shm_->elmo_torque[i] << " ";
            }
            torqueLog << std::endl;

            torqueCommandLog << (float)rd_gl_.control_time_us_ / 1000000.0 << " ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torqueCommandLog << dc_.torque_command[i] << " ";
            }
            torqueCommandLog << std::endl;

            posLog << (float)rd_gl_.control_time_us_ / 1000000.0 << " ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                posLog << rd_gl_.q_[i] << " ";
            }
            posLog << std::endl;

            velLog << (float)rd_gl_.control_time_us_ / 1000000.0 << " ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                velLog << rd_gl_.q_dot_[i] << " ";
            }
            velLog << std::endl;

            torqueActualLog << (float)rd_gl_.control_time_us_ / 1000000.0 << " ";
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torqueActualLog << (int)dc_.tc_shm_->torqueActual[i] << " ";
            }
            torqueActualLog << std::endl;

            maskLog << (float)rd_gl_.control_time_us_ / 1000000.0 << " ";
            for (int i = 0; i < 10; i++)
            {
                maskLog << std::setfill(' ') << std::setw(6) << (int)dc_.tc_shm_->e1_m[i] << " ";
            }
            for (int i = 0; i < 10; i++)
            {
                maskLog << std::setfill(' ') << std::setw(6) << (int)dc_.tc_shm_->e2_m[i] << " ";
            }
            maskLog << std::endl;

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
    torqueCommandLog.close();
    torqueActualLog.close();
    maskLog.close();
    posLog.close();
    velLog.close();

    std::cout << "Logger : END!" << std::endl;
    return (void *)NULL;
}
