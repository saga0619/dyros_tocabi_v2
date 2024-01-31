#include "tocabi_controller/tocabi_controller.h"

using namespace std;

TocabiController::TocabiController(StateManager &stm_global) : dc_(stm_global.dc_), stm_(stm_global), rd_(stm_global.dc_.rd_)
#ifdef COMPILE_TOCABI_CC
                                                               ,
                                                               my_cc(*(new CustomController(rd_)))
#endif
#ifdef COMPILE_TOCABI_AVATAR
                                                               ,
                                                               ac_(*(new AvatarController(rd_)))
#endif
{
    // Tocabi Controller Initialize Component

    nh_controller_.setCallbackQueue(&queue_controller_);
    // sub_1 = nh_controller_.subscribe("/tocabi/avatar_test", 1, &AvatarController::avatar_callback, this);

    task_command_sub_ = nh_controller_.subscribe("/tocabi/taskcommand", 100, &TocabiController::TaskCommandCallback, this);
    task_command_que_sub_ = nh_controller_.subscribe("/tocabi/taskquecommand", 100, &TocabiController::TaskQueCommandCallback, this);
    position_command_sub_ = nh_controller_.subscribe("/tocabi/positioncommand", 100, &TocabiController::PositionCommandCallback, this);
    task_gain_sub_ = nh_controller_.subscribe("/tocabi/taskgaincommand", 100, &TocabiController::TaskGainCommandCallback, this);

    ros::param::get("/tocabi_controller/Kp", rd_.pos_kp_v);
    ros::param::get("/tocabi_controller/Kv", rd_.pos_kv_v);

    // DWBC::R

    string urdf_path;

    ros::param::get("/tocabi_controller/urdf_path", urdf_path);

    drd_.LoadModelData(urdf_path, true, false);

    drd_.AddContactConstraint("l_ankleroll_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    drd_.AddContactConstraint("r_ankleroll_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    drd_.AddContactConstraint("l_wrist2_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    drd_.AddContactConstraint("r_wrist2_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
}

TocabiController::~TocabiController()
{
    cout << "TocabiController Terminated" << endl;
}

// Thread1 : running
void *TocabiController::Thread1() // Thread1, running with 2Khz.
{
    // std::cout << "thread1_entered" << std::endl;

    volatile int rcv_time_ = 0;
    // cout << "shm_msgs:" << dc_.tc_shm_->t_cnt << endl;
    // cout << "entered" << endl;

    if (dc_.tc_shm_->shutdown)
    {
        cout << "what?" << endl;
    }

    // cout << "waiting first calc.." << endl;
    while (!rd_.firstCalc)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        if (dc_.tc_shm_->shutdown)
        {
            break;
        }
    }

    // std::cout << "thread1 Proceeding ... " << endl;

    WBC::SetContactInit(rd_);

    EnableThread2(false); // Set true for Thread2
    EnableThread3(false); // True for thread3 ...

    if (dc_.simMode)
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {
            rd_.link_[i].pos_p_gain << 400, 400, 400;
            rd_.link_[i].pos_d_gain << 40, 40, 40;
            rd_.link_[i].pos_a_gain << 1, 1, 1;

            rd_.link_[i].rot_p_gain << 400, 400, 400;
            rd_.link_[i].rot_d_gain << 40, 40, 40;
            rd_.link_[i].rot_a_gain << 1, 1, 1;
        }
    }
    else
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {
            rd_.link_[i].pos_p_gain << 80, 80, 80;
            rd_.link_[i].pos_d_gain << 10, 10, 10;
            rd_.link_[i].pos_a_gain << 1, 1, 1;

            rd_.link_[i].rot_p_gain << 200, 200, 200;
            rd_.link_[i].rot_d_gain << 16, 16, 16;
            rd_.link_[i].rot_a_gain << 1, 1, 1;
        }
        // std::cout << " CNTRL : Set with realrobot gain" << std::endl;
    }

    // std::cout<<"21"<<std::endl;

    // std::cout << "entering thread1 loop" << endl;
    auto t_start = std::chrono::steady_clock::now();

    signalThread1 = true;
    int thread1_count = 0;

    // get system time and make it to std::string
    // time format : yearmonthday_hour_min_sec
    // example : 231231_12_30_30
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string year = std::to_string(1900 + ltm->tm_year);
    std::string month = std::to_string(1 + ltm->tm_mon);
    std::string day = std::to_string(ltm->tm_mday);
    std::string hour = std::to_string(ltm->tm_hour);
    std::string min = std::to_string(ltm->tm_min);
    std::string sec = std::to_string(ltm->tm_sec);
    std::string system_time = year + month + day + "_" + hour + "_" + min + "_" + sec;
    // output_file = output_file + "_" + time + ".txt";

    bool run_controller = false;
    long prev_chrono_time = 0;
    double prev_control_time = 0;
    bool task_que_mode_ = false;
    bool retain_pre_target = false;
    while (!dc_.tc_shm_->shutdown)
    {
        mb();

        if (dc_.tc_shm_->shutdown)
            break;
        if (dc_.triggerThread1)
        {
            rd_.mtx.lock();
            dc_.triggerThread1 = false;
            rd_.mtx.unlock();
            thread1_count++;
            run_controller = true;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }

        if (run_controller)
        {
            run_controller = false;

            rcv_time_ = rd_.control_time_us_;

            auto t1 = std::chrono::steady_clock::now();

            queue_controller_.callAvailable(ros::WallDuration());

            //////////////////////////////////////////////////////////
            ////////////////////Start Tocabi Controll/////////////////
            //////////////////////////////////////////////////////////

            VectorQd torque_task_, torque_grav_, torque_contact_;
            torque_task_.setZero();
            torque_grav_.setZero();
            torque_contact_.setZero();

            static VectorQd zero_m = VectorQd::Zero();

            WBC::ContactCalcDefault(rd_);

            mb();

            if (rd_.control_time_ == prev_control_time)
            {
                std::cout << "same time error : " << rd_.control_time_ << std::endl;
            }

            double d_time = rd_.control_time_ - prev_control_time;

            prev_control_time = rd_.control_time_;

            long chrono_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_start).count();

            int chrono_dt = chrono_time - prev_chrono_time;
            prev_chrono_time = chrono_time;

            if (rd_.positionControlSwitch)
            {
                rd_.positionControlSwitch = false;

                rd_.q_desired = rd_.q_;
                rd_.positionHoldSwitch = true;
                rd_.pc_mode = true;

                std::cout << " CNTRL : Position Hold!" << rd_.control_time_ << std::endl;

                // for (int i = 0; i < MODEL_DOF; i++)
                // {
                //     std::cout << rd_.pos_kp_v[i] << "  " << rd_.pos_kv_v[i] << std::endl;
                // }
            }

            if (task_que_mode_)
            {
                if (rd_.control_time_ > (rd_.tc_time_ + rd_.tc_.time))
                {
                    rd_.task_que_signal_ = true;
                }
            }
            if (rd_.task_que_signal_)
            {
                rd_.task_que_signal_ = false;

                if (stm_.modechange_flag)
                {
                    std::cout << "force mode to 2 from "<< rd_.tc_q_.tque[0].mode  << std::endl;
                    if (rd_.tc_q_.tque[0].mode == 1)
                    {
                        rd_.tc_q_.tque[0].mode = 2;
                    }
                }
                GetTaskCommand(rd_.tc_q_.tque[0]);

                std::cout << " TASK QUE : " << rd_.tc_q_.tque.size() << std::endl;
                rd_.tc_q_.tque.erase(rd_.tc_q_.tque.begin());

                if (rd_.tc_q_.tque.size() == 0)
                {
                    std::cout << " TASK QUE END " << std::endl;
                    task_que_mode_ = false;
                }
                else
                {
                    task_que_mode_ = true;
                    std::cout << " TASK QUE CONTINUEING " << std::endl;
                }
            }

            if (rd_.tc_avatar_switch)
            {
                rd_.pc_mode = false;
                rd_.tc_.mode = 12;

                std::cout << " CNTRL : task signal received mode :" << rd_.tc_.mode << std::endl;

                stm_.StatusPub("%f task Control mode : %d", (float)rd_.control_time_, rd_.tc_.mode);

                rd_.tc_time_ = rd_.control_time_;
                rd_.tc_run = true;
                rd_.tc_init = true;

                rd_.tc_avatar_switch = false;
            }

            if (rd_.pc_mode)
            {
                if (rd_.positionHoldSwitch)
                {
                }
                else
                {
                    rd_.q_desired = DyrosMath::cubicVector(rd_.control_time_, rd_.pc_time_, rd_.pc_time_ + rd_.pc_traj_time_, rd_.pc_pos_init, rd_.pc_pos_des, rd_.pc_vel_init, zero_m);
                }

                for (int i = 0; i < MODEL_DOF; i++)
                {
                    rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (zero_m[i] - rd_.q_dot_[i]);
                }

                if (rd_.pc_gravity)
                {
                    WBC::SetContact(rd_, 1, 1);

                    rd_.torque_desired = rd_.torque_desired + WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
                }
            }
            else if (rd_.tc_run)
            {
                static ofstream task_log;
                static ofstream task1_log;
                static ofstream task2_log;
                // get user name of linux system
                //  std::string user_name = getenv("USER");

                std::string output_file = "/home/dyros/tocabi_log/output";
                output_file = output_file + "_" + system_time + ".txt";

                std::string output1_file = "/home/dyros/tocabi_log/original";
                output1_file = output1_file + "_" + system_time + ".txt";
                std::string output2_file = "/home/dyros/tocabi_log/reduced";
                output2_file = output2_file + "_" + system_time + ".txt";

                if (rd_.tc_.mode == 0)
                {
                    static double time_start_mode2 = 0.0;
                    double ang2rad = 0.0174533;
                    drd_.UpdateKinematics(rd_.q_virtual_, rd_.q_dot_virtual_, rd_.q_ddot_virtual_);
                    drd_.control_time_ = rd_.control_time_;

                    int drd_lh_id = drd_.getLinkID("l_wrist2_link");
                    int drd_rh_id = drd_.getLinkID("r_wrist2_link");
                    int drd_ub_id = drd_.getLinkID("upperbody_link");
                    int drd_pl_id = drd_.getLinkID("pelvis_link");
                    int drd_com_id = drd_.getLinkID("COM");

                    static bool init_qp;
                    if (rd_.tc_init)
                    {
                        if (rd_.tc_.customTaskGain)
                        {
                            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        }

                        init_qp = true;

                        if (rd_.tc_.solver == 0)
                        {
                            std::cout << "TASK MODE 0 : 2LEVEL TASK EXPERIMENT :::: ORIGINAL " << std::endl;
                        }
                        else if (rd_.tc_.solver == 1)
                        {
                            std::cout << "TASK MODE 0 : 2LEVEL TASK EXPERIMENT :::: REDUCED " << std::endl;
                        }
                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                        drd_.ClearTaskSpace();
                        // drd_.AddTaskSpace(DWBC::TASK_CUSTOM, 6);
                        drd_.AddTaskSpace(0, DWBC::TASK_LINK_POSITION, "COM", Vector3d::Zero());

                        if (rd_.tc_.maintain_lc)
                        {
                            std::cout << "maintain lc" << std::endl;
                            rd_.link_[COM_id].x_init = rd_.link_[Pelvis].x_desired;

                            rd_.link_[Pelvis].rot_init = rd_.link_[Pelvis].rot_desired;

                            rd_.link_[Upper_Body].rot_init = rd_.link_[Upper_Body].rot_desired;

                            rd_.link_[Left_Hand].x_init = rd_.link_[Left_Hand].x_desired;
                            rd_.link_[Left_Hand].rot_init = drd_.ts_[3].task_link_[0].rot_traj;

                            rd_.link_[Right_Hand].x_init = rd_.link_[Right_Hand].x_desired;
                            rd_.link_[Right_Hand].rot_init = drd_.ts_[3].task_link_[1].rot_traj;
                        }

                        rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                        rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
                        rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, rd_.tc_.pelv_pitch * ang2rad, rd_.link_[Pelvis].yaw_init);
                        rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                        Vector3d com_diff = rd_.link_[Pelvis].x_desired - rd_.link_[COM_id].x_init;

                        rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].x_init + com_diff;
                        rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init + com_diff;

                        drd_.ts_[0].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[0].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[COM_id].x_init, Vector3d::Zero(), rd_.link_[Pelvis].x_desired, Vector3d::Zero());
                        // drd_.ts_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(1, DWBC::TASK_LINK_ROTATION, "pelvis_link", Vector3d::Zero());
                        drd_.ts_[1].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[1].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(2, DWBC::TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());
                        drd_.ts_[2].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[2].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Upper_Body].rot_init, Vector3d::Zero(), rd_.link_[Upper_Body].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "l_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].x_init, Vector3d::Zero(), rd_.link_[Left_Hand].x_desired, Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].rot_init, Vector3d::Zero(), rd_.link_[Left_Hand].rot_init, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "r_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[1].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].x_init, Vector3d::Zero(), rd_.link_[Right_Hand].x_desired, Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].rot_init, Vector3d::Zero(), rd_.link_[Right_Hand].rot_init, Vector3d::Zero());
                    }

                    int d1, d2, d3, d4, d5;
                    std::chrono::time_point<std::chrono::steady_clock> t0, t1, t2, t3, t4, t5;

                    int ret1, ret2;

                    if (rd_.tc_.solver == 0)
                    {

                        t0 = std::chrono::steady_clock::now();

                        drd_.SetContact(1, 1);
                        drd_.CalcContactConstraint();
                        drd_.CalcGravCompensation();

                        t1 = std::chrono::steady_clock::now();

                        drd_.CalcTaskSpace();

                        t2 = std::chrono::steady_clock::now();

                        ret1 = drd_.CalcTaskControlTorque(true, init_qp, false);

                        t3 = std::chrono::steady_clock::now();

                        ret2 = drd_.CalcContactRedistribute(true, init_qp);

                        t4 = std::chrono::steady_clock::now();

                        t5 = std::chrono::steady_clock::now();
                    }
                    else if (rd_.tc_.solver == 1)
                    {
                        t0 = std::chrono::steady_clock::now();

                        drd_.SetContact(1, 1);
                        // drd_.CalcContactConstraint();
                        drd_.ReducedDynamicsCalculate();
                        t1 = std::chrono::steady_clock::now();

                        drd_.ReducedCalcContactConstraint();
                        drd_.ReducedCalcGravCompensation();

                        t2 = std::chrono::steady_clock::now();

                        drd_.ReducedCalcTaskSpace();

                        t3 = std::chrono::steady_clock::now();

                        ret1 = drd_.ReducedCalcTaskControlTorque(true, init_qp, false);

                        t4 = std::chrono::steady_clock::now();

                        ret2 = drd_.ReducedCalcContactRedistribute(true, init_qp);

                        t5 = std::chrono::steady_clock::now();
                    }

                    d1 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                    d2 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                    d3 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
                    d4 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
                    d5 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

                    rd_.torque_desired = drd_.torque_task_ + drd_.torque_grav_ + drd_.torque_contact_;

                    if (!ret1)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "task control error" << std::endl;
                    }
                    if (!ret2)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "contact control error" << std::endl;
                    }

                    init_qp = false;

                    Vector3d plv_rpy = DyrosMath::rot2Euler(drd_.link_[drd_pl_id].rotm);
                    Vector3d ub_rpy = DyrosMath::rot2Euler(drd_.link_[drd_ub_id].rotm);

                    Vector3d plv_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[1].task_link_[0].rot_traj);
                    Vector3d ub_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[2].task_link_[0].rot_traj);

                    // tf2::RotationMatrix plv_rot;

                    for (int i = 0; i < 3; i++)
                    {
                        if (plv_rpy(i) > 0.5 * M_PI)
                        {
                            plv_rpy(i) -= M_PI;
                        }
                        else if (plv_rpy(i) < -0.5 * M_PI)
                        {
                            plv_rpy(i) += M_PI;
                        }
                        if (ub_rpy(i) > 0.5 * M_PI)
                        {
                            ub_rpy(i) -= M_PI;
                        }
                        else if (ub_rpy(i) < -0.5 * M_PI)
                        {
                            ub_rpy(i) += M_PI;
                        }
                        if (plv_rpy_traj(i) > 0.5 * M_PI)
                        {
                            plv_rpy_traj(i) -= M_PI;
                        }
                        else if (plv_rpy_traj(i) < -0.5 * M_PI)
                        {
                            plv_rpy_traj(i) += M_PI;
                        }
                        if (ub_rpy_traj(i) > 0.5 * M_PI)
                        {
                            ub_rpy_traj(i) -= M_PI;
                        }
                        else if (ub_rpy_traj(i) < -0.5 * M_PI)
                        {
                            ub_rpy_traj(i) += M_PI;
                        }
                    }
                }
                if (rd_.tc_.mode == 1)
                {
                    static double time_start_mode1 = 0.0;
                    double ang2rad = 0.0174533;
                    drd_.UpdateKinematics(rd_.q_virtual_, rd_.q_dot_virtual_, rd_.q_ddot_virtual_);
                    drd_.control_time_ = rd_.control_time_;

                    int drd_lh_id = drd_.getLinkID("l_wrist2_link");
                    int drd_rh_id = drd_.getLinkID("r_wrist2_link");
                    int drd_ub_id = drd_.getLinkID("upperbody_link");
                    int drd_pl_id = drd_.getLinkID("pelvis_link");
                    int drd_com_id = drd_.getLinkID("COM");

                    static bool init_qp;
                    if (rd_.tc_init)
                    {

                        if (task1_log.is_open())
                        {
                            std::cout << "file already opened " << std::endl;
                        }
                        else
                        {
                            task1_log.open(output1_file.c_str(), fstream::out | fstream::app);
                            task1_log << "time d1 d2 d3 d4 d5 cm_tx cm_ty cm_tz cm_x cm_y cm_z pv_tr pv_tp pv_ty pv_r pv_p pv_y ub_tr ub_tp ub_ty ub_r ub_p ub_y lh_tx lh_ty lh_tz lh_x lh_y lh_z rh_tx rh_ty rh_tz rh_x rh_y rh_z" << std::endl;
                            // task_log << "time com_pos_x com_pos_y com_pos_z ft0 ft1 ft2 ft3 ft4 ft5 ft6 ft7 ft8 ft9 ft10 ft11" << std::endl;
                            if (task1_log.is_open())
                            {
                                std::cout << "open success " << std::endl;
                                time_start_mode1 = drd_.control_time_;
                            }
                        }

                        if (rd_.tc_.customTaskGain)
                        {
                            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        }

                        init_qp = true;

                        std::cout << "TASK MODE 1 : 2LEVEL TASK EXPERIMENT :::: ORIGINAL " << std::endl;

                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                        drd_.ClearTaskSpace();
                        // drd_.AddTaskSpace(DWBC::TASK_CUSTOM, 6);
                        drd_.AddTaskSpace(0, DWBC::TASK_LINK_POSITION, "COM", Vector3d::Zero());

                        if (rd_.tc_.maintain_lc)
                        {
                            std::cout << "Maintain lc" << std::endl;
                            rd_.link_[COM_id].x_init = rd_.link_[Pelvis].x_desired;

                            rd_.link_[Pelvis].rot_init = rd_.link_[Pelvis].rot_desired;

                            rd_.link_[Upper_Body].rot_init = rd_.link_[Upper_Body].rot_desired;

                            rd_.link_[Left_Hand].x_init = rd_.link_[Left_Hand].x_desired;
                            rd_.link_[Left_Hand].rot_init = drd_.ts_[3].task_link_[0].rot_traj;

                            rd_.link_[Right_Hand].x_init = rd_.link_[Right_Hand].x_desired;
                            rd_.link_[Right_Hand].rot_init = drd_.ts_[3].task_link_[1].rot_traj;
                        }

                        rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                        rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
                        rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, rd_.tc_.pelv_pitch * ang2rad, rd_.link_[Pelvis].yaw_init);
                        rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                        Vector3d com_diff = rd_.link_[Pelvis].x_desired - rd_.link_[COM_id].x_init;

                        rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].x_init + com_diff;
                        rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init + com_diff;

                        drd_.ts_[0].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[0].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[COM_id].x_init, Vector3d::Zero(), rd_.link_[Pelvis].x_desired, Vector3d::Zero());
                        // drd_.ts_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(1, DWBC::TASK_LINK_ROTATION, "pelvis_link", Vector3d::Zero());
                        drd_.ts_[1].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[1].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(2, DWBC::TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());
                        drd_.ts_[2].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[2].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Upper_Body].rot_init, Vector3d::Zero(), rd_.link_[Upper_Body].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "l_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].x_init, Vector3d::Zero(), rd_.link_[Left_Hand].x_desired, Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].rot_init, Vector3d::Zero(), rd_.link_[Left_Hand].rot_init, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "r_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[1].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].x_init, Vector3d::Zero(), rd_.link_[Right_Hand].x_desired, Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].rot_init, Vector3d::Zero(), rd_.link_[Right_Hand].rot_init, Vector3d::Zero());
                    }

                    int d1, d2, d3, d4, d5;
                    std::chrono::time_point<std::chrono::steady_clock> t0, t1, t2, t3, t4, t5;

                    int ret1, ret2;

                    // if (rd_.tc_.solver == 0)
                    {

                        t0 = std::chrono::steady_clock::now();

                        drd_.SetContact(1, 1);
                        drd_.CalcContactConstraint();
                        drd_.CalcGravCompensation();

                        t1 = std::chrono::steady_clock::now();

                        drd_.CalcTaskSpace();

                        t2 = std::chrono::steady_clock::now();

                        ret1 = drd_.CalcTaskControlTorque(true, init_qp, false);

                        t3 = std::chrono::steady_clock::now();

                        ret2 = drd_.CalcContactRedistribute(true, init_qp);

                        t4 = std::chrono::steady_clock::now();

                        t5 = std::chrono::steady_clock::now();
                    }
                    // else if (rd_.tc_.solver == 1)
                    // {
                    //     t0 = std::chrono::steady_clock::now();

                    //     drd_.SetContact(1, 1);
                    //     // drd_.CalcContactConstraint();
                    //     drd_.ReducedDynamicsCalculate();
                    //     t1 = std::chrono::steady_clock::now();

                    //     drd_.ReducedCalcContactConstraint();
                    //     drd_.ReducedCalcGravCompensation();

                    //     t2 = std::chrono::steady_clock::now();

                    //     drd_.ReducedCalcTaskSpace();

                    //     t3 = std::chrono::steady_clock::now();

                    //     ret1 = drd_.ReducedCalcTaskControlTorque(init_qp, true, false);

                    //     t4 = std::chrono::steady_clock::now();

                    //     ret2 = drd_.ReducedCalcContactRedistribute(init_qp);

                    //     t5 = std::chrono::steady_clock::now();
                    // }

                    d1 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                    d2 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                    d3 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
                    d4 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
                    d5 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

                    rd_.torque_desired = drd_.torque_task_ + drd_.torque_grav_ + drd_.torque_contact_;

                    if (!ret1)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "task control error" << std::endl;
                    }
                    if (!ret2)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "contact control error" << std::endl;
                    }

                    init_qp = false;

                    Vector3d plv_rpy = DyrosMath::rot2Euler(drd_.link_[drd_pl_id].rotm);
                    Vector3d ub_rpy = DyrosMath::rot2Euler(drd_.link_[drd_ub_id].rotm);

                    Vector3d plv_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[1].task_link_[0].rot_traj);
                    Vector3d ub_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[2].task_link_[0].rot_traj);

                    // tf2::RotationMatrix plv_rot;

                    for (int i = 0; i < 3; i++)
                    {
                        if (plv_rpy(i) > 0.5 * M_PI)
                        {
                            plv_rpy(i) -= M_PI;
                        }
                        else if (plv_rpy(i) < -0.5 * M_PI)
                        {
                            plv_rpy(i) += M_PI;
                        }
                        if (ub_rpy(i) > 0.5 * M_PI)
                        {
                            ub_rpy(i) -= M_PI;
                        }
                        else if (ub_rpy(i) < -0.5 * M_PI)
                        {
                            ub_rpy(i) += M_PI;
                        }
                        if (plv_rpy_traj(i) > 0.5 * M_PI)
                        {
                            plv_rpy_traj(i) -= M_PI;
                        }
                        else if (plv_rpy_traj(i) < -0.5 * M_PI)
                        {
                            plv_rpy_traj(i) += M_PI;
                        }
                        if (ub_rpy_traj(i) > 0.5 * M_PI)
                        {
                            ub_rpy_traj(i) -= M_PI;
                        }
                        else if (ub_rpy_traj(i) < -0.5 * M_PI)
                        {
                            ub_rpy_traj(i) += M_PI;
                        }
                    }

                    task1_log << fixed << setprecision(6) << drd_.control_time_ - time_start_mode1 << " "
                              << d1 << " " << d2 << " " << d3 << " " << d4 << " " << d5 << " "
                              << drd_.ts_[0].task_link_[0].pos_traj_(0) << " " << drd_.ts_[0].task_link_[0].pos_traj_(1) << " " << drd_.ts_[0].task_link_[0].pos_traj_(2) << " "
                              << drd_.link_[drd_com_id].xpos(0) << " " << drd_.link_[drd_com_id].xpos(1) << " " << drd_.link_[drd_com_id].xpos(2) << " "
                              << plv_rpy_traj(0) << " " << plv_rpy_traj(1) << " " << plv_rpy_traj(2) << " "
                              << plv_rpy(0) << " " << plv_rpy(1) << " " << plv_rpy(2) << " "
                              << ub_rpy_traj(0) << " " << ub_rpy_traj(1) << " " << ub_rpy_traj(2) << " "
                              << ub_rpy(0) << " " << ub_rpy(1) << " " << ub_rpy(2) << " "
                              << drd_.ts_[3].task_link_[0].pos_traj_(0) << " " << drd_.ts_[3].task_link_[0].pos_traj_(1) << " " << drd_.ts_[3].task_link_[0].pos_traj_(2) << " "
                              << drd_.link_[drd_lh_id].xpos(0) << " " << drd_.link_[drd_lh_id].xpos(1) << " " << drd_.link_[drd_lh_id].xpos(2) << " "
                              << drd_.ts_[3].task_link_[1].pos_traj_(0) << " " << drd_.ts_[3].task_link_[1].pos_traj_(1) << " " << drd_.ts_[3].task_link_[1].pos_traj_(2) << " "
                              << drd_.link_[drd_rh_id].xpos(0) << " " << drd_.link_[drd_rh_id].xpos(1) << " " << drd_.link_[drd_rh_id].xpos(2) << " "
                              << std::endl;
                }
                else if (rd_.tc_.mode == 2)
                {
                    static double time_start_mode2 = 0.0;
                    double ang2rad = 0.0174533;
                    drd_.UpdateKinematics(rd_.q_virtual_, rd_.q_dot_virtual_, rd_.q_ddot_virtual_);
                    drd_.control_time_ = rd_.control_time_;

                    int drd_lh_id = drd_.getLinkID("l_wrist2_link");
                    int drd_rh_id = drd_.getLinkID("r_wrist2_link");
                    int drd_ub_id = drd_.getLinkID("upperbody_link");
                    int drd_pl_id = drd_.getLinkID("pelvis_link");
                    int drd_com_id = drd_.getLinkID("COM");

                    static bool init_qp;
                    if (rd_.tc_init)
                    {

                        if (task2_log.is_open())
                        {
                            std::cout << "file already opened " << std::endl;
                        }
                        else
                        {
                            task2_log.open(output2_file.c_str(), fstream::out | fstream::app);
                            task2_log << "time d1 d2 d3 d4 d5 cm_tx cm_ty cm_tz cm_x cm_y cm_z pv_tr pv_tp pv_ty pv_r pv_p pv_y ub_tr ub_tp ub_ty ub_r ub_p ub_y lh_tx lh_ty lh_tz lh_x lh_y lh_z rh_tx rh_ty rh_tz rh_x rh_y rh_z" << std::endl;
                            // task_log << "time com_pos_x com_pos_y com_pos_z ft0 ft1 ft2 ft3 ft4 ft5 ft6 ft7 ft8 ft9 ft10 ft11" << std::endl;
                            if (task2_log.is_open())
                            {
                                std::cout << "open success " << std::endl;
                                time_start_mode2 = drd_.control_time_;
                            }
                        }

                        if (rd_.tc_.customTaskGain)
                        {
                            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        }

                        init_qp = true;

                        std::cout << "TASK MODE 2 : 2LEVEL TASK EXPERIMENT :::: REDUCED " << std::endl;

                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                        drd_.ClearTaskSpace();
                        // drd_.AddTaskSpace(DWBC::TASK_CUSTOM, 6);
                        drd_.AddTaskSpace(0, DWBC::TASK_LINK_POSITION, "COM", Vector3d::Zero());

                        if (rd_.tc_.maintain_lc)
                        {
                            std::cout << "maintain lc" << std::endl;
                            rd_.link_[COM_id].x_init = rd_.link_[Pelvis].x_desired;

                            rd_.link_[Pelvis].rot_init = rd_.link_[Pelvis].rot_desired;

                            rd_.link_[Upper_Body].rot_init = rd_.link_[Upper_Body].rot_desired;

                            rd_.link_[Left_Hand].x_init = rd_.link_[Left_Hand].x_desired;
                            rd_.link_[Left_Hand].rot_init = drd_.ts_[3].task_link_[0].rot_traj;

                            rd_.link_[Right_Hand].x_init = rd_.link_[Right_Hand].x_desired;
                            rd_.link_[Right_Hand].rot_init = drd_.ts_[3].task_link_[1].rot_traj;
                        }

                        rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                        rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
                        rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, rd_.tc_.pelv_pitch * ang2rad, rd_.link_[Pelvis].yaw_init);
                        rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                        Vector3d com_diff = rd_.link_[Pelvis].x_desired - rd_.link_[COM_id].x_init;

                        rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].x_init + com_diff;
                        rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init + com_diff;

                        drd_.ts_[0].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[0].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[COM_id].x_init, Vector3d::Zero(), rd_.link_[Pelvis].x_desired, Vector3d::Zero());
                        // drd_.ts_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(1, DWBC::TASK_LINK_ROTATION, "pelvis_link", Vector3d::Zero());
                        drd_.ts_[1].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[1].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(2, DWBC::TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());
                        drd_.ts_[2].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[2].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Upper_Body].rot_init, Vector3d::Zero(), rd_.link_[Upper_Body].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "l_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].x_init, Vector3d::Zero(), rd_.link_[Left_Hand].x_desired, Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].rot_init, Vector3d::Zero(), rd_.link_[Left_Hand].rot_init, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "r_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[1].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].x_init, Vector3d::Zero(), rd_.link_[Right_Hand].x_desired, Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].rot_init, Vector3d::Zero(), rd_.link_[Right_Hand].rot_init, Vector3d::Zero());
                    }

                    int d1, d2, d3, d4, d5;
                    std::chrono::time_point<std::chrono::steady_clock> t0, t1, t2, t3, t4, t5;

                    int ret1, ret2;

                    // if (rd_.tc_.solver == 0)
                    // {

                    //     t0 = std::chrono::steady_clock::now();

                    //     drd_.SetContact(1, 1);
                    //     drd_.CalcContactConstraint();
                    //     drd_.CalcGravCompensation();

                    //     t1 = std::chrono::steady_clock::now();

                    //     drd_.CalcTaskSpace();

                    //     t2 = std::chrono::steady_clock::now();

                    //     ret1 = drd_.CalcTaskControlTorque(init_qp, true, false);

                    //     t3 = std::chrono::steady_clock::now();

                    //     ret2 = drd_.CalcContactRedistribute(init_qp);

                    //     t4 = std::chrono::steady_clock::now();

                    //     t5 = std::chrono::steady_clock::now();
                    // }
                    // else if (rd_.tc_.solver == 1)
                    {
                        t0 = std::chrono::steady_clock::now();

                        drd_.SetContact(1, 1);
                        // drd_.CalcContactConstraint();
                        drd_.ReducedDynamicsCalculate();
                        t1 = std::chrono::steady_clock::now();

                        drd_.ReducedCalcContactConstraint();
                        drd_.ReducedCalcGravCompensation();

                        t2 = std::chrono::steady_clock::now();

                        drd_.ReducedCalcTaskSpace();

                        t3 = std::chrono::steady_clock::now();

                        ret1 = drd_.ReducedCalcTaskControlTorque(true, init_qp, false);

                        t4 = std::chrono::steady_clock::now();

                        ret2 = drd_.ReducedCalcContactRedistribute(true, init_qp);

                        t5 = std::chrono::steady_clock::now();
                    }

                    d1 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                    d2 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                    d3 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
                    d4 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
                    d5 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

                    rd_.torque_desired = drd_.torque_task_ + drd_.torque_grav_ + drd_.torque_contact_;

                    if (!ret1)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "task control error" << std::endl;
                    }
                    if (!ret2)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "contact control error" << std::endl;
                    }

                    init_qp = false;

                    Vector3d plv_rpy = DyrosMath::rot2Euler(drd_.link_[drd_pl_id].rotm);
                    Vector3d ub_rpy = DyrosMath::rot2Euler(drd_.link_[drd_ub_id].rotm);

                    Vector3d plv_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[1].task_link_[0].rot_traj);
                    Vector3d ub_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[2].task_link_[0].rot_traj);

                    // tf2::RotationMatrix plv_rot;

                    for (int i = 0; i < 3; i++)
                    {
                        if (plv_rpy(i) > 0.5 * M_PI)
                        {
                            plv_rpy(i) -= M_PI;
                        }
                        else if (plv_rpy(i) < -0.5 * M_PI)
                        {
                            plv_rpy(i) += M_PI;
                        }
                        if (ub_rpy(i) > 0.5 * M_PI)
                        {
                            ub_rpy(i) -= M_PI;
                        }
                        else if (ub_rpy(i) < -0.5 * M_PI)
                        {
                            ub_rpy(i) += M_PI;
                        }
                        if (plv_rpy_traj(i) > 0.5 * M_PI)
                        {
                            plv_rpy_traj(i) -= M_PI;
                        }
                        else if (plv_rpy_traj(i) < -0.5 * M_PI)
                        {
                            plv_rpy_traj(i) += M_PI;
                        }
                        if (ub_rpy_traj(i) > 0.5 * M_PI)
                        {
                            ub_rpy_traj(i) -= M_PI;
                        }
                        else if (ub_rpy_traj(i) < -0.5 * M_PI)
                        {
                            ub_rpy_traj(i) += M_PI;
                        }
                    }

                    task2_log << fixed << setprecision(6) << drd_.control_time_ - time_start_mode2 << " "
                              << d1 << " " << d2 << " " << d3 << " " << d4 << " " << d5 << " "
                              << drd_.ts_[0].task_link_[0].pos_traj_(0) << " " << drd_.ts_[0].task_link_[0].pos_traj_(1) << " " << drd_.ts_[0].task_link_[0].pos_traj_(2) << " "
                              << drd_.link_[drd_com_id].xpos(0) << " " << drd_.link_[drd_com_id].xpos(1) << " " << drd_.link_[drd_com_id].xpos(2) << " "
                              << plv_rpy_traj(0) << " " << plv_rpy_traj(1) << " " << plv_rpy_traj(2) << " "
                              << plv_rpy(0) << " " << plv_rpy(1) << " " << plv_rpy(2) << " "
                              << ub_rpy_traj(0) << " " << ub_rpy_traj(1) << " " << ub_rpy_traj(2) << " "
                              << ub_rpy(0) << " " << ub_rpy(1) << " " << ub_rpy(2) << " "
                              << drd_.ts_[3].task_link_[0].pos_traj_(0) << " " << drd_.ts_[3].task_link_[0].pos_traj_(1) << " " << drd_.ts_[3].task_link_[0].pos_traj_(2) << " "
                              << drd_.link_[drd_lh_id].xpos(0) << " " << drd_.link_[drd_lh_id].xpos(1) << " " << drd_.link_[drd_lh_id].xpos(2) << " "
                              << drd_.ts_[3].task_link_[1].pos_traj_(0) << " " << drd_.ts_[3].task_link_[1].pos_traj_(1) << " " << drd_.ts_[3].task_link_[1].pos_traj_(2) << " "
                              << drd_.link_[drd_rh_id].xpos(0) << " " << drd_.link_[drd_rh_id].xpos(1) << " " << drd_.link_[drd_rh_id].xpos(2) << " "
                              << std::endl;
                }
                else if (rd_.tc_.mode == 3)
                {
                    static double time_start_mode2 = 0.0;
                    double ang2rad = 0.0174533;
                    drd_.UpdateKinematics(rd_.q_virtual_, rd_.q_dot_virtual_, rd_.q_ddot_virtual_);
                    drd_.control_time_ = rd_.control_time_;

                    rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
                    rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, rd_.tc_.pelv_pitch * ang2rad, rd_.link_[Pelvis].yaw_init);
                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                    static bool init_qp;
                    if (rd_.tc_init)
                    {

                        if (task_log.is_open())
                        {
                            std::cout << "file already opened " << std::endl;
                        }
                        else
                        {
                            task_log.open(output_file.c_str(), fstream::out | fstream::app);
                            task_log << "time d1 d2 d3 d4 d5 cm_tx cm_ty cm_tz cm_x cm_y cm_z pv_tr pv_tp pv_ty pv_r pv_p pv_y ub_tr ub_tp ub_ty ub_r ub_p ub_y lh_tx lh_ty lh_tz lh_x lh_y lh_z rh_tx rh_ty rh_tz rh_x rh_y rh_z" << std::endl;
                            // task_log << "time com_pos_x com_pos_y com_pos_z ft0 ft1 ft2 ft3 ft4 ft5 ft6 ft7 ft8 ft9 ft10 ft11" << std::endl;
                            if (task_log.is_open())
                            {
                                std::cout << "open success " << std::endl;
                                time_start_mode2 = drd_.control_time_;
                            }
                        }

                        if (rd_.tc_.customTaskGain)
                        {
                            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        }

                        init_qp = true;

                        if (rd_.tc_.solver == 0)
                        {
                            std::cout << "TASK MODE 3 : 2LEVEL TASK EXPERIMENT :::: ORIGINAL " << std::endl;
                        }
                        else if (rd_.tc_.solver == 1)
                        {
                            std::cout << "TASK MODE 3 : 2LEVEL TASK EXPERIMENT :::: REDUCED " << std::endl;
                        }
                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                        drd_.ClearTaskSpace();
                        // drd_.AddTaskSpace(DWBC::TASK_CUSTOM, 6);
                        drd_.AddTaskSpace(0, DWBC::TASK_LINK_POSITION, "COM", Vector3d::Zero());
                        drd_.ts_[0].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[0].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[COM_id].x_init, Vector3d::Zero(), rd_.link_[Pelvis].x_desired, Vector3d::Zero());
                        // drd_.ts_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(1, DWBC::TASK_LINK_ROTATION, "pelvis_link", Vector3d::Zero());
                        drd_.ts_[1].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[1].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(2, DWBC::TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());
                        drd_.ts_[2].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[2].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Upper_Body].rot_init, Vector3d::Zero(), rd_.link_[Upper_Body].rot_desired, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "l_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].x_init, Vector3d::Zero(), rd_.link_[Left_Hand].x_init, Vector3d::Zero());
                        drd_.ts_[3].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].rot_init, Vector3d::Zero(), rd_.link_[Left_Hand].rot_init, Vector3d::Zero());

                        drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "r_wrist2_link", Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
                        drd_.ts_[3].task_link_[1].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].x_init, Vector3d::Zero(), rd_.link_[Right_Hand].x_init, Vector3d::Zero());
                        drd_.ts_[3].task_link_[1].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].rot_init, Vector3d::Zero(), rd_.link_[Right_Hand].rot_init, Vector3d::Zero());
                    }

                    int d1, d2, d3, d4, d5;
                    std::chrono::time_point<std::chrono::steady_clock> t0, t1, t2, t3, t4, t5;

                    int ret1, ret2;

                    if (rd_.tc_.solver == 0)
                    {

                        t0 = std::chrono::steady_clock::now();

                        drd_.SetContact(1, 1);
                        drd_.CalcContactConstraint();
                        drd_.CalcGravCompensation();

                        t1 = std::chrono::steady_clock::now();

                        drd_.CalcTaskSpace();

                        t2 = std::chrono::steady_clock::now();

                        ret1 = drd_.CalcTaskControlTorque(init_qp, true, false);

                        t3 = std::chrono::steady_clock::now();

                        ret2 = drd_.CalcContactRedistribute(init_qp);

                        t4 = std::chrono::steady_clock::now();

                        t5 = std::chrono::steady_clock::now();
                    }
                    else if (rd_.tc_.solver == 1)
                    {
                        t0 = std::chrono::steady_clock::now();

                        drd_.SetContact(1, 1);
                        // drd_.CalcContactConstraint();
                        drd_.ReducedDynamicsCalculate();
                        t1 = std::chrono::steady_clock::now();

                        drd_.ReducedCalcContactConstraint();
                        drd_.ReducedCalcGravCompensation();

                        t2 = std::chrono::steady_clock::now();

                        drd_.ReducedCalcTaskSpace();

                        t3 = std::chrono::steady_clock::now();

                        ret1 = drd_.ReducedCalcTaskControlTorque(init_qp, true, false);

                        t4 = std::chrono::steady_clock::now();

                        ret2 = drd_.ReducedCalcContactRedistribute(init_qp);

                        t5 = std::chrono::steady_clock::now();
                    }

                    d1 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                    d2 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                    d3 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
                    d4 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
                    d5 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

                    rd_.torque_desired = drd_.torque_task_ + drd_.torque_grav_ + drd_.torque_contact_;

                    if (!ret1)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "task control error" << std::endl;
                    }
                    if (!ret2)
                    {
                        rd_.positionControlSwitch = true;
                        std::cout << "contact control error" << std::endl;
                    }

                    init_qp = false;

                    int drd_lh_id = drd_.getLinkID("l_wrist2_link");
                    int drd_rh_id = drd_.getLinkID("r_wrist2_link");
                    int drd_ub_id = drd_.getLinkID("upperbody_link");
                    int drd_pl_id = drd_.getLinkID("pelvis_link");
                    int drd_com_id = drd_.getLinkID("COM");

                    task_log << drd_.control_time_ - time_start_mode2 << " "
                             << d1 << " " << d2 << " " << d3 << " " << d4 << " " << d5 << " "
                             << drd_.ts_[0].task_link_[0].pos_traj_(0) << " " << drd_.ts_[0].task_link_[0].pos_traj_(1) << " " << drd_.ts_[0].task_link_[0].pos_traj_(2) << " "
                             << drd_.link_[drd_com_id].xpos(0) << " " << drd_.link_[drd_com_id].xpos(1) << " " << drd_.link_[drd_com_id].xpos(2) << " "
                             << drd_.ts_[2].task_link_[0].rpy_traj(0) << " " << drd_.ts_[2].task_link_[0].rpy_traj(1) << " " << drd_.ts_[2].task_link_[0].rpy_traj(2) << " "
                             << drd_.link_[drd_pl_id].rpy(0) << " " << drd_.link_[drd_pl_id].rpy(1) << " " << drd_.link_[drd_pl_id].rpy(2) << " "
                             << drd_.ts_[3].task_link_[0].rpy_traj(0) << " " << drd_.ts_[3].task_link_[0].rpy_traj(1) << " " << drd_.ts_[3].task_link_[0].rpy_traj(2) << " "
                             << drd_.link_[drd_ub_id].rpy(0) << " " << drd_.link_[drd_ub_id].rpy(1) << " " << drd_.link_[drd_ub_id].rpy(2) << " "
                             << drd_.ts_[1].task_link_[0].pos_traj_(0) << " " << drd_.ts_[1].task_link_[0].pos_traj_(1) << " " << drd_.ts_[1].task_link_[0].pos_traj_(2) << " "
                             << drd_.link_[drd_lh_id].xpos(0) << " " << drd_.link_[drd_lh_id].xpos(1) << " " << drd_.link_[drd_lh_id].xpos(2) << " "
                             << drd_.ts_[1].task_link_[1].pos_traj_(0) << " " << drd_.ts_[1].task_link_[1].pos_traj_(1) << " " << drd_.ts_[1].task_link_[1].pos_traj_(2) << " "
                             << drd_.link_[drd_rh_id].xpos(0) << " " << drd_.link_[drd_rh_id].xpos(1) << " " << drd_.link_[drd_rh_id].xpos(2) << " "
                             << std::endl;
                }
                else if (rd_.tc_.mode == 5)
                {
                    static bool init_qp;
                    if (rd_.tc_init)
                    {
                        init_qp = true;

                        // if (task_log.is_open())
                        // {
                        //     std::cout << "file already opened " << std::endl;
                        // }
                        // else
                        // {
                        //     task_log.open(output_file.c_str(), fstream::out | fstream::app);
                        //     task_log << "time pel_pos_x pel_pos_y pel_pos_z pel_vel_x pel_vel_y pel_vel_z xtraj_x xtraj_y xtraj_z vtraj_x vtraj_y vtraj_z upper_r upper_p upper_y upper_tx upper_ty upper_tz rtraj_r rtraj_p rtraj_y ttraj_x ttraj_y ttraj_z" << std::endl;
                        //     if (task_log.is_open())
                        //     {
                        //         std::cout << "open success " << std::endl;
                        //     }
                        // }

                        std::cout << "mode 5 init" << std::endl;
                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    int task1_id = Pelvis;
                    int task2_id = Upper_Body;

                    // rd_.tc_.left_foot = 1;
                    // rd_.tc_.right_foot = 1;

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
                    double ang2rad = 0.0174533;

                    rd_.link_[task1_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;

                    rd_.link_[task1_id].x_desired(0) = rd_.link_[task1_id].xi_init(0);

                    rd_.link_[task1_id].x_desired(2) = rd_.tc_.height;

                    rd_.link_[task1_id].rot_desired = DyrosMath::Euler2rot(0, 0 * ang2rad, rd_.link_[Pelvis].yaw_init);

                    rd_.link_[task2_id].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                    if (rd_.tc_.customTaskGain)
                    {
                        rd_.link_[task1_id].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        rd_.link_[task2_id].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                    }

                    rd_.link_[task1_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[task1_id].xi_init, rd_.link_[task1_id].x_desired);
                    rd_.link_[task1_id].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[task2_id].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

                    TaskSpace ts_(6);

                    Eigen::MatrixXd Jtask = rd_.link_[task1_id].JacCOM();
                    Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[task1_id], true, true);
                    ts_.Update(Jtask, fstar);

                    WBC::CalcJKT(rd_, ts_);

                    // std::cout << "effective Of COM mass along Y axis : ";

                    rd_.link_[Pelvis].x_desired = rd_.link_[Pelvis].x_init;
                    rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, 0, rd_.link_[Pelvis].yaw_init);

                    TaskSpace ts_pelv(6);
                    ts_pelv.Update(rd_.link_[Pelvis].Jac(), WBC::GetFstar6d(rd_.link_[Pelvis], true, true));

                    WBC::CalcJKT(rd_, ts_pelv);

                    Eigen::Vector6d u_vector;
                    u_vector.setZero();
                    u_vector(1) = 1.0;

                    // std::cout << 1 / (u_vector.transpose() * ts_.Lambda_task_.inverse() * u_vector) << std::endl;

                    // std::cout << "effective Of Pelvis mass along Y axis :" << 1 / (u_vector.transpose() * ts_pelv.Lambda_task_.inverse() * u_vector) << std::endl;

                    WBC::CalcTaskNull(rd_, ts_);

                    static CQuadraticProgram task_qp_;
                    int hqp1_solve_result = WBC::TaskControlHQP(rd_, ts_, task_qp_, rd_.torque_grav, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), init_qp);

                    // WBC::CalcTaskNull(rd_, ts2_);
                    VectorQd torque_task_hqp_;
                    if (hqp1_solve_result)
                    {

                        Eigen::MatrixXd Jtask2 = rd_.link_[task2_id].Jac().bottomRows(3);
                        // std::cout << "1" << std::endl;

                        Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[task2_id]);
                        // std::cout << "2" << std::endl;

                        TaskSpace ts2_(3);
                        static CQuadraticProgram task_qp2_;
                        ts2_.Update(Jtask2, fstar2);
                        // std::cout << "3" << std::endl;

                        WBC::CalcJKT(rd_, ts2_);

                        // std::cout << "4" << std::endl;
                        if (WBC::TaskControlHQP(rd_, ts2_, task_qp2_, rd_.torque_grav + ts_.torque_h_, ts_.Null_task, init_qp))
                        {
                            // std::cout << "5" << std::endl;
                            torque_task_hqp_ = rd_.torque_grav + ts_.torque_h_ + ts_.Null_task * ts2_.torque_h_;
                        }
                        else
                        {
                            // std::cout << "6" << std::endl;
                            torque_task_hqp_ = rd_.torque_grav + ts_.torque_h_; // + ts_.Null_task * ts2_.torque_h_;
                        }
                        // std::cout << "7" << std::endl;
                    }
                    else
                    {
                        torque_task_hqp_ = rd_.torque_grav;

                        std::cout << "Solve Error, Disable Task Control" << std::endl;
                        rd_.positionControlSwitch = true;
                    }
                    // std::cout << "6" << std::endl;

                    // WBC::TaskControlHQP(rd_, task_qp_2, Jtask2, Jkt2, fstar2, lambda_task2, torque_task_hqp_ + rd_.torque_grav, ts_.Null_task, fstar_qp2, contact_qp2, init_qp);

                    // VectorQd torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts2_.torque_h_;
                    // VectorQd torque_Task2 = ts_.J_kt_ *ts_.Lambda_task_ * ts_.f_star_;

                    // WBC::CalcContactRedistributeHQP(rd_, contact_qp_, torque_Task2 + rd_.torque_grav, torque_contact_qp_, init_qp);

                    // rd_.torque_desired = torque_Task2 + rd_.torque_grav + torque_contact_qp_;

                    // rd_.torque_desired = torque_Task2 + rd_.torque_grav + rd_.NwJw * ts2_.contact_qp_;
                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, torque_task_hqp_);
                    // std::cout << "8" << std::endl;

                    // std::cout << "7" << std::endl;

                    // Vector12d cf_est = WBC::getContactForce(rd_, rd_.torque_desired);

                    // std::cout << "9" << std::endl;
                    // Vector3d zmp_got = WBC::GetZMPpos_from_ContactForce(rd_, cf_est);
                    // std::cout << "8" << std::endl;

                    // std::cout << "10" << std::endl;
                    // double ur, up, uy, utx, uty, utz;

                    // DyrosMath::rot2Euler_tf2(rd_.link_[Upper_Body].rotm, ur, up, uy);
                    // DyrosMath::rot2Euler_tf2(rd_.link_[Upper_Body].r_traj, utx, uty, utz);

                    // if (rd_.control_time_ > rd_.tc_time_ && rd_.control_time_ < rd_.tc_time_ + rd_.tc_.time + 0.5)
                    // {
                    //     task_log << rd_.control_time_ << " "
                    //              << rd_.link_[Pelvis].xipos(0) << " " << rd_.link_[Pelvis].xipos(1) << " " << rd_.link_[Pelvis].xipos(2) << " "
                    //              << rd_.link_[Pelvis].vi(0) << " " << rd_.link_[Pelvis].vi(1) << " " << rd_.link_[Pelvis].vi(2) << " "
                    //              << rd_.link_[Pelvis].x_traj(0) << " " << rd_.link_[Pelvis].x_traj(1) << " " << rd_.link_[Pelvis].x_traj(2) << " "
                    //              << rd_.link_[Pelvis].v_traj(0) << " " << rd_.link_[Pelvis].v_traj(1) << " " << rd_.link_[Pelvis].v_traj(2) << " "

                    //              << ur << " " << up << " " << uy << " "
                    //              << rd_.link_[Upper_Body].w(0) << " " << rd_.link_[Upper_Body].w(1) << " " << rd_.link_[Upper_Body].w(2) << " "

                    //              << utx << " " << uty << " " << utz << " "
                    //              << rd_.link_[Upper_Body].w_traj(0) << " " << rd_.link_[Upper_Body].w_traj(1) << " " << rd_.link_[Upper_Body].w_traj(2) << " "

                    //              //  << rd_.link_[Pelvis].xpos(0) << " " << rd_.link_[Pelvis].xpos(1) << " " << rd_.link_[Pelvis].xpos(2) << " "
                    //              //  << rd_.link_[Pelvis].v(0) << " " << rd_.link_[Pelvis].v(1) << " " << rd_.link_[Pelvis].v(2) << " "
                    //              //  << fstar(0) << " " << fstar(1) << " " << fstar(2) << " "
                    //              //  << out(0) << " " << out(1) << " " << out(2) << " "
                    //              //  << rd_.link_[COM_id].a_traj(0) << " " << rd_.link_[COM_id].a_traj(1) << " " << rd_.link_[COM_id].a_traj(2) << " "
                    //              //  << rd_.q_(0) << " " << rd_.q_(1) << " " << rd_.q_(2) << " " << rd_.q_(3) << " " << rd_.q_(4) << " " << rd_.q_(5) << " "
                    //              //  << rd_.q_dot_(0) << " " << rd_.q_dot_(1) << " " << rd_.q_dot_(2) << " " << rd_.q_dot_(3) << " " << rd_.q_dot_(4) << " " << rd_.q_dot_(5) << " "
                    //              //  << rd_.q_ext_(0) << " " << rd_.q_ext_(1) << " " << rd_.q_ext_(2) << " " << rd_.q_ext_(3) << " " << rd_.q_ext_(4) << " " << rd_.q_ext_(5) << " "
                    //              //  << rd_.zmp_global_(0) << " " << rd_.zmp_global_(1) << " "
                    //              //  << zmp_got(0) << " " << zmp_got(1) << " "
                    //              //  << rd_.q_ddot_virtual_(0) << " " << rd_.q_ddot_virtual_(1) << " " << rd_.q_ddot_virtual_(2) << " "
                    //              << std::endl;
                    // }

                    init_qp = false;
                }
                else if (rd_.tc_.mode == 6)
                {
                    static bool init_qp;
                    if (rd_.tc_init)
                    {
                        init_qp = true;

                        if (task_log.is_open())
                        {
                            std::cout << "file already opened " << std::endl;
                        }
                        else
                        {
                            task_log.open(output_file.c_str(), fstream::out | fstream::app);
                            task_log.precision(8);
                            task_log << "time rcv_time dtime chrono_Time dt_chr pel_pos_x pel_pos_y pel_pos_z pel_vel_x pel_vel_y pel_vel_z fstar_x fstar_y fstar_z force_x force_y force_z lf_x lf_y lf_z rf_x rf_y rf_z lf_dx lf_dy lf_dz rf_dx rf_dy rf_dz q0 q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 qe0 qe1 qe2 qe3 qe4 qe5 qe6 qe7 qe8 qe9 qe10 qe11" << std::endl;
                            if (task_log.is_open())
                            {
                                std::cout << "open success " << std::endl;
                            }
                        }

                        std::cout << "mode 6 init" << std::endl;
                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    // rd_.tc_.left_foot = 1;
                    // rd_.tc_.right_foot = 1;

                    int task1_id = Pelvis;
                    int task2_id = Upper_Body;

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
                    double ang2rad = 0.0174533;

                    rd_.link_[task1_id].x_desired = 0.5 * rd_.link_[Left_Foot].x_init + 0.5 * rd_.link_[Right_Foot].x_init;

                    rd_.link_[task1_id].x_desired(0) = rd_.link_[task1_id].xi_init(0);

                    rd_.link_[task1_id].x_desired(2) = rd_.tc_.height;

                    rd_.link_[task1_id].rot_desired = DyrosMath::Euler2rot(0, 0 * ang2rad, rd_.link_[Pelvis].yaw_init);

                    rd_.link_[task2_id].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                    if (rd_.tc_.customTaskGain)
                    {
                        rd_.link_[task1_id].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        rd_.link_[task2_id].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                    }

                    rd_.link_[task1_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[task1_id].xi_init, rd_.link_[task1_id].x_desired);
                    rd_.link_[task1_id].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[task2_id].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

                    TaskSpace ts_(6);

                    Eigen::MatrixXd Jtask = rd_.link_[task1_id].JacCOM();
                    Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[task1_id], true, true);

                    fstar(1) = rd_.tc_.ratio * sin(rd_.tc_.time * (rd_.control_time_ - rd_.tc_time_));

                    ts_.Update(Jtask, fstar);
                    WBC::CalcJKT(rd_, ts_);

                    rd_.task_force_ = ts_.Lambda_task_ * ts_.f_star_;

                    WBC::CalcTaskNull(rd_, ts_);

                    static CQuadraticProgram task_qp_;
                    int hqp1_solve_result = WBC::TaskControlHQP(rd_, ts_, task_qp_, rd_.torque_grav, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), init_qp);

                    // WBC::CalcTaskNull(rd_, ts2_);
                    VectorQd torque_task_hqp_;
                    if (hqp1_solve_result)
                    {

                        Eigen::MatrixXd Jtask2 = rd_.link_[task2_id].Jac().bottomRows(3);
                        // std::cout << "1" << std::endl;

                        Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[task2_id]);
                        // std::cout << "2" << std::endl;

                        TaskSpace ts2_(3);
                        static CQuadraticProgram task_qp2_;
                        ts2_.Update(Jtask2, fstar2);
                        // std::cout << "3" << std::endl;

                        WBC::CalcJKT(rd_, ts2_);

                        // std::cout << "4" << std::endl;
                        if (WBC::TaskControlHQP(rd_, ts2_, task_qp2_, rd_.torque_grav + ts_.torque_h_, ts_.Null_task, init_qp))
                        {
                            // std::cout << "5" << std::endl;
                            torque_task_hqp_ = rd_.torque_grav + ts_.torque_h_ + ts_.Null_task * ts2_.torque_h_;
                        }
                        else
                        {
                            torque_task_hqp_ = rd_.torque_grav;

                            std::cout << "Solve Error : Task2 ,Disable Task Control" << std::endl;
                            rd_.positionControlSwitch = true;
                            // std::cout << "6" << std::endl;
                            torque_task_hqp_ = rd_.torque_grav + ts_.torque_h_; // + ts_.Null_task * ts2_.torque_h_;
                        }
                        // std::cout << "7" << std::endl;
                    }
                    else
                    {
                        torque_task_hqp_ = rd_.torque_grav;

                        std::cout << "Solve Error : Task1 ,Disable Task Control" << std::endl;
                        rd_.positionControlSwitch = true;
                    }
                    // std::cout << "6" << std::endl;

                    // WBC::TaskControlHQP(rd_, task_qp_2, Jtask2, Jkt2, fstar2, lambda_task2, torque_task_hqp_ + rd_.torque_grav, ts_.Null_task, fstar_qp2, contact_qp2, init_qp);

                    // VectorQd torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts2_.torque_h_;
                    // VectorQd torque_Task2 = ts_.J_kt_ *ts_.Lambda_task_ * ts_.f_star_;

                    // WBC::CalcContactRedistributeHQP(rd_, contact_qp_, torque_Task2 + rd_.torque_grav, torque_contact_qp_, init_qp);

                    // rd_.torque_desired = torque_Task2 + rd_.torque_grav + torque_contact_qp_;

                    // rd_.torque_desired = torque_Task2 + rd_.torque_grav + rd_.NwJw * ts2_.contact_qp_;
                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, torque_task_hqp_);
                    // std::cout << "8" << std::endl;

                    // std::cout << "7" << std::endl;

                    // Vector12d cf_est = WBC::getContactForce(rd_, rd_.torque_desired);

                    // // std::cout << "9" << std::endl;
                    // Vector3d zmp_got = WBC::GetZMPpos_from_ContactForce(rd_, cf_est);
                    // // std::cout << "8" << std::endl;

                    // std::cout << "10" << std::endl;
                    // double ur, up, uy, utx, uty, utz;

                    // DyrosMath::rot2Euler_tf2(rd_.link_[Upper_Body].rotm, ur, up, uy);
                    // DyrosMath::rot2Euler_tf2(rd_.link_[Upper_Body].r_traj, utx, uty, utz);

                    double rcv_ctime = rcv_time_ / 1000000.0;
                    double chorno_db_time = chrono_time / 1000000.0;

                    // if (rd_.control_time_ > rd_.tc_time_ && rd_.control_time_ < rd_.tc_time_ + rd_.tc_.time + 0.5)
                    // {
                    task_log << rd_.control_time_ << " " << rcv_ctime << " " << d_time << " " << chorno_db_time << " " << chrono_dt << " "
                             << rd_.link_[task1_id].xipos(0) << " " << rd_.link_[task1_id].xipos(1) << " " << rd_.link_[task1_id].xipos(2) << " "
                             << rd_.link_[task1_id].vi(0) << " " << rd_.link_[task1_id].vi(1) << " " << rd_.link_[task1_id].vi(2) << " "
                             << ts_.f_star_(0) << " " << ts_.f_star_(1) << " " << ts_.f_star_(2) << " "
                             << rd_.task_force_(0) << " " << rd_.task_force_(1) << " " << rd_.task_force_(2) << " "
                             << rd_.link_[Left_Foot].xpos(0) << " " << rd_.link_[Left_Foot].xpos(1) << " " << rd_.link_[Left_Foot].xpos(2) << " "
                             << rd_.link_[Right_Foot].xpos(0) << " " << rd_.link_[Right_Foot].xpos(1) << " " << rd_.link_[Right_Foot].xpos(2) << " "
                             << rd_.link_[Left_Foot].v(0) << " " << rd_.link_[Left_Foot].v(1) << " " << rd_.link_[Left_Foot].v(2) << " "
                             << rd_.link_[Right_Foot].v(0) << " " << rd_.link_[Right_Foot].v(1) << " " << rd_.link_[Right_Foot].v(2) << " "

                             //  << ur << " " << up << " " << uy << " "
                             //  << rd_.link_[Upper_Body].w(0) << " " << rd_.link_[Upper_Body].w(1) << " " << rd_.link_[Upper_Body].w(2) << " "

                             //  << utx << " " << uty << " " << utz << " "
                             //  << rd_.link_[Upper_Body].w_traj(0) << " " << rd_.link_[Upper_Body].w_traj(1) << " " << rd_.link_[Upper_Body].w_traj(2) << " "

                             //  << rd_.link_[Pelvis].xpos(0) << " " << rd_.link_[Pelvis].xpos(1) << " " << rd_.link_[Pelvis].xpos(2) << " "
                             //  << rd_.link_[Pelvis].v(0) << " " << rd_.link_[Pelvis].v(1) << " " << rd_.link_[Pelvis].v(2) << " "
                             //  << fstar(0) << " " << fstar(1) << " " << fstar(2) << " "
                             //  << out(0) << " " << out(1) << " " << out(2) << " "
                             //  << rd_.link_[COM_id].a_traj(0) << " " << rd_.link_[COM_id].a_traj(1) << " " << rd_.link_[COM_id].a_traj(2) << " "
                             << rd_.q_(0) << " " << rd_.q_(1) << " " << rd_.q_(2) << " " << rd_.q_(3) << " " << rd_.q_(4) << " " << rd_.q_(5) << " "
                             << rd_.q_(6) << " " << rd_.q_(7) << " " << rd_.q_(8) << " " << rd_.q_(9) << " " << rd_.q_(10) << " " << rd_.q_(11) << " "
                             //  << rd_.q_dot_(0) << " " << rd_.q_dot_(1) << " " << rd_.q_dot_(2) << " " << rd_.q_dot_(3) << " " << rd_.q_dot_(4) << " " << rd_.q_dot_(5) << " "
                             << rd_.q_ext_(0) << " " << rd_.q_ext_(1) << " " << rd_.q_ext_(2) << " " << rd_.q_ext_(3) << " " << rd_.q_ext_(4) << " " << rd_.q_ext_(5) << " "
                             << rd_.q_ext_(6 + 0) << " " << rd_.q_ext_(6 + 1) << " " << rd_.q_ext_(6 + 2) << " " << rd_.q_ext_(6 + 3) << " " << rd_.q_ext_(6 + 4) << " " << rd_.q_ext_(6 + 5) << " "
                             //  << rd_.zmp_global_(0) << " " << rd_.zmp_global_(1) << " "
                             //  << zmp_got(0) << " " << zmp_got(1) << " "
                             //  << rd_.q_ddot_virtual_(0) << " " << rd_.q_ddot_virtual_(1) << " " << rd_.q_ddot_virtual_(2) << " "
                             << std::endl;
                    // }

                    init_qp = false;
                }

#ifdef COMPILE_TOCABI_AVATAR
                if ((rd_.tc_.mode > 9) && (rd_.tc_.mode < 15))
                {
                    RequestThread2();

                    num1 = num1 + 1;
                    try
                    {
                        ac_.computeSlow();
                    }
                    catch (const std::exception &e)
                    {
                        std::cout << "Error occured at AVATAR THREAD1" << std::endl;

                        std::cerr << e.what() << '\n';

                        rd_.positionControlSwitch = true;
                    }

                    num2 = num2 + 1;
                    if (rd_.tc_.mode == 11 || rd_.tc_.mode == 13)
                    {
                        static int thread3_count = 1;

                        if (thread3_count == 40)
                        {
                            thread3_count = 0;

                            RequestThread3();
                        }
                        thread3_count++;
                    }

                    // If necessary, use
                    // To Enable Thread2, you need to fix the 50th line. Change EnableThread2(false) to EnableThread2(true).
                    // If not, thread2 is disabled, so that you cannot use thread2
                    // RequestThread2() : call this function to trigger Thread2 at each tick.
                }
#endif
#ifdef COMPILE_TOCABI_CC
                if ((rd_.tc_.mode > 5) && (rd_.tc_.mode < 9)) // 6,7,8
                {
                    my_cc.computeSlow();
                }
#endif
            }
            else
            {
                drd_.UpdateKinematics(rd_.q_virtual_, rd_.q_dot_virtual_, rd_.q_ddot_virtual_);

                drd_.SetContact(1, 1);
                drd_.CalcContactConstraint();
                drd_.CalcGravCompensation();
                drd_.CalcContactRedistribute(false);

                // WBC::SetContact(rd_, 1, 1);
                rd_.torque_desired = drd_.torque_grav_ + drd_.torque_contact_;
            }

            // Send Data To thread2

            // Data2Thread2

            // std::cout << torque_task_.norm() << "\t" << torque_grav_.norm() << "\t" << torque_contact_.norm() << std::endl;

            static std::chrono::steady_clock::time_point t_c_ = std::chrono::steady_clock::now();

            // Available at simMode for now ...
            // if (dc_.simMode)
            //     WBC::CheckTorqueLimit(rd_, rd_.torque_desired);

            SendCommand(rd_.torque_desired);

            auto t_end = std::chrono::steady_clock::now();

            auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t1).count();            // 150us without march=native
            auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(t_end - rd_.tp_state_).count(); // 150us without march=native

            // zmp calculation
            rd_.zmp_global_ = WBC::GetZMPpos_fromFT(rd_);

            Eigen::VectorXd cf_from_torque;
            // cf_from_torque.resize(rd_.contact_index * 6);
            cf_from_torque = WBC::getContactForce(rd_, rd_.torque_desired);
            // std::cout << cf_from_torque.transpose() << std::endl;

            // std::cout << rd_.LF_CF_FT.transpose() << rd_.RF_CF_FT.transpose() << std::endl;
            // std::cout << "ZMP from cf : "<<WBC::GetZMPpos_from_ContactForce(rd_, cf_from_torque).transpose() <<"  zmp from ft : " <<rd_.zmp_global_.transpose()<<std::endl;
            // std::cout << "lf zmp cf : "<< (rd_.ee_[0].zmp - rd_.ee_[0].xpos_contact).segment(0,2).transpose() << " direct calc from cf : "<< -cf_from_torque(4)/cf_from_torque(2) << " " <<cf_from_torque(3)/cf_from_torque(2) <<" lf zmp from ft : "<<- rd_.LF_CF_FT(4) / rd_.LF_CF_FT(2) << rd_.LF_CF_FT(3) / rd_.LF_CF_FT(2)<<std::endl;

            static int d1_over_cnt = 0;

            if (d1 > 500)
            {
                d1_over_cnt++;
            }

            static int d2_total = 0;
            static double d1_total = 0;

            d2_total += d2;
            d1_total += d1;

            // if (d2 > 350)
            // {
            //     std::cout << rd_.control_time_ << "command duration over 350us , " << d2 << std::endl;
            // }

            dc_.tcm_cnt = thread1_count;
            if (thread1_count % 2000 == 0)
            {
                /*
                WBC::SetContact(rd_, 1, 1);

                WBC::SetContact(rd_, 1, 0);

                WBC::SetContact(rd_, 0, 1);*/

                // std::cout << rd_.control_time_ << "s : avg rcv2send : " << d2_total / thread1_count << " us, state : " << rd_.state_ctime_total_ / thread1_count << " controller : " << d1_total / thread1_count << " diff : " << (d2_total - rd_.state_ctime_total_ - d1_total) / thread1_count << std::endl;

                if (d1_over_cnt > 0)
                {
                    std::cout << cred << " DYNCS : Thread1 calculation time over 500us.. : " << d1_over_cnt << "times, stm cnt : " << dc_.tc_shm_->stloopCount << creset << std::endl;
                    d1_over_cnt = 0;
                }

                d1_total = 0;
                d2_total = 0;
                rd_.state_ctime_total_ = 0;
            }
            t_c_ = std::chrono::steady_clock::now();
        }
        // std::cout<<"21"<<std::endl;
    }

    // cout << "thread1 terminate" << endl;
    return (void *)NULL;
}

// Thread2 : running with request
void *TocabiController::Thread2()
{

    cout << "CNTRL2 : started with pid : " << getpid() << std::endl;

    while (true)
    {
        if (signalThread1 || dc_.tc_shm_->shutdown)
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    if (enableThread2)
    {
        // std::cout << "thread2_entered" << std::endl;
        while (!dc_.tc_shm_->shutdown)
        {
            if (triggerThread2)
            {
                triggerThread2 = false;
                /////////////////////////////////////////////
                /////////////Do something in Thread2 !!!!!!!

                if (rd_.tc_run)
                {
#ifdef COMPILE_TOCABI_AVATAR
                    if ((rd_.tc_.mode > 9) && (rd_.tc_.mode < 15))
                    {
                        try
                        {
                            num3 = num3 + 1;
                            ac_.computeFast();
                            num4 = num4 + 1;
                        }
                        catch (const std::exception &e)
                        {
                            std::cout << "Error occured at AVATAR THREAD2" << std::endl;

                            std::cerr << e.what() << '\n';

                            rd_.positionControlSwitch = true;
                        }
                    }
#endif
#ifdef COMPILE_TOCABI_CC
                    if ((rd_.tc_.mode > 5) && (rd_.tc_.mode < 9)) // 6,7,8
                    {
                        my_cc.computeFast();
                    }
#endif
                }
                /////////////////////////////////////////////
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    }
    else
    {
        std::cout << " CNTRL : thread2 disabled" << std::endl;
    }

    // std::cout << "thread2 terminate" << std::endl;
    return (void *)NULL;
}

// Thread3 : running with request
void *TocabiController::Thread3()
{
    while (true)
    {
        if (signalThread1 || dc_.tc_shm_->shutdown)
            break;

        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    if (enableThread3)
    {
        // std::cout << " CNTRL : Thread3_entered" << std::endl;

        while (!dc_.tc_shm_->shutdown)
        {
            if (triggerThread3)
            {
                triggerThread3 = false;
/////////////////////////////////////////////
/////////////Do something in Thread3 !!!!!!!
#ifdef COMPILE_TOCABI_AVATAR
                ac_.computeThread3();
#endif

                /////////////////////////////////////////////
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
        }
    }
    else
    {
        std::cout << " CNTRL : thread3 disabled" << std::endl;
    }

    std::cout << " CNTRL : thread3 terminate" << std::endl;
    return (void *)NULL;
}

void TocabiController::MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2)
{
    dc_.tc_shm_->t_cnt2 = currentCount;

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

    dc_.tc_shm_->lat_avg2 = lavg;
    dc_.tc_shm_->lat_max2 = lmax;
    dc_.tc_shm_->lat_min2 = lmin;
    dc_.tc_shm_->lat_dev2 = ldev;

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

    dc_.tc_shm_->send_avg2 = savg;
    dc_.tc_shm_->send_max2 = smax;
    dc_.tc_shm_->send_min2 = smin;
    dc_.tc_shm_->send_dev2 = sdev;
}

void TocabiController::SendCommand(Eigen::VectorQd torque_command)
{
    while (dc_.t_c_)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    dc_.t_c_ = true;
    dc_.control_command_count++;
    std::copy(torque_command.data(), torque_command.data() + MODEL_DOF, dc_.torque_command);
    dc_.t_c_ = false;
}

void TocabiController::EnableThread2(bool enable)
{
    enableThread2 = enable;
}

void TocabiController::EnableThread3(bool enable)
{
    enableThread3 = enable;
}

void TocabiController::RequestThread2()
{
    triggerThread2 = true;
}
void TocabiController::RequestThread3()
{
    triggerThread3 = true;
}

void TocabiController::GetTaskCommand(tocabi_msgs::TaskCommand &msg)
{
    if (msg.maintain_lc)
    {
        if (!rd_.tc_run)
        {
            msg.maintain_lc = false;
        }
    }

    rd_.pc_mode = false;
    rd_.tc_ = msg;
    rd_.tc_time_ = rd_.control_time_;
    rd_.tc_run = true;
    rd_.tc_init = true;
    rd_.link_[Right_Foot].SetInitialWithPosition();
    rd_.link_[Left_Foot].SetInitialWithPosition();
    rd_.link_[Right_Hand].SetInitialWithPosition();
    rd_.link_[Left_Hand].SetInitialWithPosition();
    rd_.link_[Pelvis].SetInitialWithPosition();
    rd_.link_[Upper_Body].SetInitialWithPosition();
    rd_.link_[COM_id].SetInitialWithPosition();
}

void TocabiController::PositionCommandCallback(const tocabi_msgs::positionCommandConstPtr &msg)
{
    static bool position_command = false;

    rd_.pc_traj_time_ = msg->traj_time;

    rd_.pc_time_ = rd_.control_time_;

    rd_.pc_pos_init = rd_.q_;
    rd_.pc_vel_init = rd_.q_dot_;

    if (position_command && msg->relative)
    {
        rd_.pc_pos_init = rd_.q_desired;
        std::cout << " CNTRL : pos init with prev des" << std::endl;
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        rd_.pc_pos_des(i) = msg->position[i];
    }
    rd_.pc_mode = true;
    rd_.pc_gravity = msg->gravity;
    rd_.positionHoldSwitch = false;

    stm_.StatusPub("%f Position Control", (float)rd_.control_time_);
    position_command = true;
    std::cout << " CNTRL : Position command received" << std::endl;
}

void TocabiController::TaskCommandCallback(const tocabi_msgs::TaskCommandConstPtr &msg)
{
    std::cout << " CNTRL : task signal received mode :" << rd_.tc_.mode << std::endl;
    stm_.StatusPub("%f task Control mode : %d", (float)rd_.control_time_, rd_.tc_.mode);

    tocabi_msgs::TaskCommand _msg = *msg;

    GetTaskCommand(_msg);

    // double pos_p = 400.0;
    // double pos_d = 40.0;
    // double pos_a = 1;
    // double rot_p = 400.0;
    // double rot_d = 40.0;
    // double rot_a = 1.0;

    // rd_.link_[Right_Foot].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    // rd_.link_[Left_Foot].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    // rd_.link_[Right_Hand].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    // rd_.link_[Left_Hand].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    // rd_.link_[Pelvis].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    // rd_.link_[Upper_Body].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    // rd_.link_[COM_id].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);

    // std::cout << " pelv yaw init : " << rd_.link_[Pelvis].yaw_init << std::endl;

    // std::cout << "upperbody rotation init : " << DyrosMath::rot2Euler_tf(rd_.link_[Upper_Body].rot_init).transpose() << std::endl;

    if (!rd_.semode)
    {

        if (dc_.useSimVirtual)
        {
            std::cout << " CNTRL : Task control on sim v" << std::endl;
        }
        else
        {

            std::cout << " CNTRL : State Estimate is not running. disable task command" << std::endl;
            rd_.tc_run = false;
        }
    }
}

void TocabiController::TaskGainCommandCallback(const tocabi_msgs::TaskGainCommandConstPtr &msg)
{
    int __id = 0;
    bool okcheck = false;
    if (msg->mode == 1)
    {
        __id = COM_id;
    }
    else if (msg->mode == 2)
    {
        __id = Pelvis;
    }
    else if (msg->mode == 3)
    {
        __id = Upper_Body;
    }
    else if (msg->mode == 5)
    {

        __id = Right_Foot;
        std::cout << "Gain set on RightFoot" << std::endl;
    }
    else if (msg->mode == 6)
    {
        __id = Left_Foot;
        std::cout << "Gain set on leftfoot" << std::endl;
    }
    else
    {
        okcheck = true;
    }

    if (okcheck)
    {
        std::cout << "Wrong Mode " << std::endl;
    }
    else
    {
        rd_.link_[__id].pos_p_gain[0] = msg->pgain[0];
        rd_.link_[__id].pos_p_gain[1] = msg->pgain[1];
        rd_.link_[__id].pos_p_gain[2] = msg->pgain[2];

        rd_.link_[__id].pos_d_gain[0] = msg->dgain[0];
        rd_.link_[__id].pos_d_gain[1] = msg->dgain[1];
        rd_.link_[__id].pos_d_gain[2] = msg->dgain[2];

        rd_.link_[__id].rot_p_gain[0] = msg->pgain[3];
        rd_.link_[__id].rot_p_gain[1] = msg->pgain[4];
        rd_.link_[__id].rot_p_gain[2] = msg->pgain[5];

        rd_.link_[__id].rot_d_gain[0] = msg->dgain[3];
        rd_.link_[__id].rot_d_gain[1] = msg->dgain[4];
        rd_.link_[__id].rot_d_gain[2] = msg->dgain[5];

        std::cout << "Gain Set " << rd_.link_[__id].id << "  POS p : " << rd_.link_[__id].pos_p_gain.transpose() << "   d :" << rd_.link_[__id].pos_d_gain.transpose() << std::endl;
        std::cout << "Gain Set " << rd_.link_[__id].id << "  ROT p : " << rd_.link_[__id].rot_p_gain.transpose() << "   d :" << rd_.link_[__id].rot_d_gain.transpose() << std::endl;
    }
}

void TocabiController::TaskQueCommandCallback(const tocabi_msgs::TaskCommandQueConstPtr &msg)
{
    rd_.tc_q_ = *msg;
    rd_.task_que_signal_ = true;
    static int mode = 1;

    std::cout << "TASK QUE RECEIVED " << std::endl;
    if (!rd_.semode)
    {

        if (dc_.useSimVirtual)
        {
            std::cout << " CNTRL : Task control on sim v" << std::endl;
        }
        else
        {

            std::cout << " CNTRL : State Estimate is not running. disable task command" << std::endl;
            rd_.tc_run = false;
        }
    }
}

void TocabiController::QueCustomController()
{
}

void TocabiController::WaitCustomControllerCommand()
{
}
/*
void TocabiController::SetCommand()
{

}*/
