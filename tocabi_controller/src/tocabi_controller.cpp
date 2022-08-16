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

    EnableThread2(true); // Set true for Thread2
    EnableThread3(true); // True for thread3 ...

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

    signalThread1 = true;
    int thread1_count = 0;
    while (!dc_.tc_shm_->shutdown)
    {
        if (dc_.triggerThread1)
        {
            dc_.triggerThread1 = false;
            thread1_count++;
            if (dc_.tc_shm_->shutdown)
                break;
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

                std::string output_file = "/home/dyros/tocabi_log/output";
                if (rd_.tc_.mode == 0)
                {

                    if (rd_.tc_init)
                    {

                        // if (task_log.is_open())
                        // {
                        //     std::cout << "file already opened " << std::endl;
                        // }
                        // else
                        // {
                        //     task_log.open(output_file.c_str(), fstream::out | fstream::app);
                        //     task_log << "time com_pos_x com_pos_y com_pos_z com_vel_x com_vel_y com_vel_z pel_pos_x pel_pos_y pel_pos_z pel_vel_x pel_vel_y pel_vel_z fstar_x fstar_y fstar_z lambda_x lambda_y lambda_z xtraj_x xtraj_y xtraj_z vtraj_x vtraj_y vtraj_z atraj_x atraj_y atraz_z q0 q1 q2 q3 q4 q5 qdot0 qdot1 qdot2 qdot3 qdot4 qdot5 qe0 qe1 qe2 qe3 qe4 qe5 zmp_x zmp_y zmpes_x zmpes_y imux imuy imuz" << std::endl;
                        //     // task_log << "time com_pos_x com_pos_y com_pos_z ft0 ft1 ft2 ft3 ft4 ft5 ft6 ft7 ft8 ft9 ft10 ft11" << std::endl;
                        //     if (task_log.is_open())
                        //     {
                        //         std::cout << "open success " << std::endl;
                        //     }
                        // }
                        std::cout << "mode 0 init" << std::endl;
                        rd_.tc_init = false;

                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);

                    rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac().block(0, 0, 6, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

                    rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;
                    double ang2rad = 0.0174533;
                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                    if (rd_.tc_.customTaskGain)
                    {
                        rd_.link_[COM_id].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, 200, 20, 1);
                        rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, 200, 20, 1);
                    }

                    rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    Eigen::VectorXd fstar;
                    fstar.setZero(9);
                    fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id], true);
                    fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                    // if (rd_.link_[COM_id].a_traj(1) != 0)
                    // {
                    //     if (rd_.link_[COM_id].v_traj(1) > 0)
                    //     {
                    //         fstar(1) += rd_.tc_.ang_p;
                    //     }
                    //     else if (rd_.link_[COM_id].v_traj(1) < 0)
                    //     {
                    //         fstar(1) -= rd_.tc_.ang_p;
                    //     }
                    // }

                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));

                    VectorXd out = rd_.lambda * fstar;

                    Vector12d cf_est = WBC::getContactForce(rd_, rd_.torque_desired);

                    Vector3d zmp_got = WBC::GetZMPpos_from_ContactForce(rd_, cf_est);

                    // task_log << rd_.control_time_ << " "
                    //          << rd_.link_[COM_id].xpos(0) << " " << rd_.link_[COM_id].xpos(1) << " " << rd_.link_[COM_id].xpos(2) << " "
                    //          //  << rd_.LF_CF_FT(0) << " " << rd_.LF_CF_FT(1) << " " << rd_.LF_CF_FT(2) << " "
                    //          //  << rd_.LF_CF_FT(3) << " " << rd_.LF_CF_FT(4) << " " << rd_.LF_CF_FT(5) << " "
                    //          //  << rd_.RF_CF_FT(0) << " " << rd_.RF_CF_FT(1) << " " << rd_.RF_CF_FT(2) << " "
                    //          //  << rd_.RF_CF_FT(3) << " " << rd_.RF_CF_FT(4) << " " << rd_.RF_CF_FT(5) << " ";
                    //          << rd_.link_[COM_id].v(0) << " " << rd_.link_[COM_id].v(1) << " " << rd_.link_[COM_id].v(2) << " "
                    //          << rd_.link_[Pelvis].xpos(0) << " " << rd_.link_[Pelvis].xpos(1) << " " << rd_.link_[Pelvis].xpos(2) << " "
                    //          << rd_.link_[Pelvis].v(0) << " " << rd_.link_[Pelvis].v(1) << " " << rd_.link_[Pelvis].v(2) << " "
                    //          << fstar(0) << " " << fstar(1) << " " << fstar(2) << " "
                    //          << out(0) << " " << out(1) << " " << out(2) << " "
                    //          << rd_.link_[COM_id].x_traj(0) << " " << rd_.link_[COM_id].x_traj(1) << " " << rd_.link_[COM_id].x_traj(2) << " "
                    //          << rd_.link_[COM_id].v_traj(0) << " " << rd_.link_[COM_id].v_traj(1) << " " << rd_.link_[COM_id].v_traj(2) << " "
                    //          << rd_.link_[COM_id].a_traj(0) << " " << rd_.link_[COM_id].a_traj(1) << " " << rd_.link_[COM_id].a_traj(2) << " "
                    //          << rd_.q_(0) << " " << rd_.q_(1) << " " << rd_.q_(2) << " " << rd_.q_(3) << " " << rd_.q_(4) << " " << rd_.q_(5) << " "
                    //          << rd_.q_dot_(0) << " " << rd_.q_dot_(1) << " " << rd_.q_dot_(2) << " " << rd_.q_dot_(3) << " " << rd_.q_dot_(4) << " " << rd_.q_dot_(5) << " "
                    //          << rd_.q_ext_(0) << " " << rd_.q_ext_(1) << " " << rd_.q_ext_(2) << " " << rd_.q_ext_(3) << " " << rd_.q_ext_(4) << " " << rd_.q_ext_(5) << " "
                    //          << rd_.zmp_global_(0) << " " << rd_.zmp_global_(1) << " "
                    //          << zmp_got(0) << " " << zmp_got(1) << " "
                    //          << rd_.q_ddot_virtual_(0) << " " << rd_.q_ddot_virtual_(1) << " " << rd_.q_ddot_virtual_(2) << " "
                    //          << std::endl;

                    // std::cout << rd_.link_[COM_id].xpos(1) << std::endl;

                    /*
                    auto ts = std::chrono::steady_clock::now();
                    WBC::GetJKT1(rd_, rd_.J_task);
                    auto ds = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - ts).count();

                    auto ts2 = std::chrono::steady_clock::now();
                    WBC::GetJKT2(rd_, rd_.J_task);
                    auto ds2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - ts2).count();

                    rd_.time_for_inverse += ds;
                    rd_.time_for_inverse_total += ds2;

                    rd_.count_for_inverse++;
                    rd_.count_for_inverse_total++;

                    if (rd_.count_for_inverse == 2000)
                    {
                        std::cout << "avg 1 : " << rd_.time_for_inverse / rd_.count_for_inverse << " 2 : " << rd_.time_for_inverse_total / rd_.count_for_inverse_total << std::endl;

                        rd_.time_for_inverse = 0;
                        rd_.time_for_inverse_total = 0;
                        rd_.count_for_inverse = 0;
                        rd_.count_for_inverse_total = 0;
                    }*/
                }
                else if (rd_.tc_.mode == 1)
                {

                    if (rd_.tc_init)
                    {

                        // if (task_log.is_open())
                        // {
                        //     std::cout << "file already opened " << std::endl;
                        // }
                        // else
                        // {
                        //     task_log.open(output_file.c_str(), fstream::out | fstream::app);
                        //     task_log << "time com_pos_x com_pos_y com_pos_z com_vel_x com_vel_y com_vel_z xtraj_x xtraj_y xtraj_z vtraj_x vtraj_y vtraj_z" << std::endl;
                        //     // task_log << "time com_pos_x com_pos_y com_pos_z ft0 ft1 ft2 ft3 ft4 ft5 ft6 ft7 ft8 ft9 ft10 ft11" << std::endl;
                        //     if (task_log.is_open())
                        //     {
                        //         std::cout << "open success " << std::endl;
                        //     }
                        // }
                        std::cout << "mode 0 init" << std::endl;
                        rd_.tc_init = false;

                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);

                    rd_.J_task.setZero(6, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac().block(0, 0, 3, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(3, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

                    rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

                    double ang2rad = 0.0174533;

                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                    Eigen::VectorXd fstar;

                    if (rd_.tc_.customTaskGain)
                    {
                        rd_.link_[COM_id].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                    }

                    rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    // rd_.link_[COM_id].SetTrajectoryCubic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    fstar.setZero(6);
                    fstar.segment(0, 3) = WBC::GetFstarPos(rd_.link_[COM_id], true);
                    fstar.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));

                    VectorXd out = rd_.lambda * fstar;

                    Vector12d cf_est = WBC::getContactForce(rd_, rd_.torque_desired);

                    Vector3d zmp_got = WBC::GetZMPpos_from_ContactForce(rd_, cf_est);

                    // task_log << rd_.control_time_ << " "
                    //          << rd_.link_[COM_id].xpos(0) << " " << rd_.link_[COM_id].xpos(1) << " " << rd_.link_[COM_id].xpos(2) << " "
                    //          //  << rd_.LF_CF_FT(0) << " " << rd_.LF_CF_FT(1) << " " << rd_.LF_CF_FT(2) << " "
                    //          //  << rd_.LF_CF_FT(3) << " " << rd_.LF_CF_FT(4) << " " << rd_.LF_CF_FT(5) << " "
                    //          //  << rd_.RF_CF_FT(0) << " " << rd_.RF_CF_FT(1) << " " << rd_.RF_CF_FT(2) << " "
                    //          //  << rd_.RF_CF_FT(3) << " " << rd_.RF_CF_FT(4) << " " << rd_.RF_CF_FT(5) << " ";
                    //          << rd_.link_[COM_id].v(0) << " " << rd_.link_[COM_id].v(1) << " " << rd_.link_[COM_id].v(2) << " "
                    //          //  << rd_.link_[Pelvis].xpos(0) << " " << rd_.link_[Pelvis].xpos(1) << " " << rd_.link_[Pelvis].xpos(2) << " "
                    //          //  << rd_.link_[Pelvis].v(0) << " " << rd_.link_[Pelvis].v(1) << " " << rd_.link_[Pelvis].v(2) << " "
                    //          //  << fstar(0) << " " << fstar(1) << " " << fstar(2) << " "
                    //          //  << out(0) << " " << out(1) << " " << out(2) << " "
                    //          << rd_.link_[COM_id].x_traj(0) << " " << rd_.link_[COM_id].x_traj(1) << " " << rd_.link_[COM_id].x_traj(2) << " "
                    //          << rd_.link_[COM_id].v_traj(0) << " " << rd_.link_[COM_id].v_traj(1) << " " << rd_.link_[COM_id].v_traj(2) << " "
                    //          //  << rd_.link_[COM_id].a_traj(0) << " " << rd_.link_[COM_id].a_traj(1) << " " << rd_.link_[COM_id].a_traj(2) << " "
                    //          //  << rd_.q_(0) << " " << rd_.q_(1) << " " << rd_.q_(2) << " " << rd_.q_(3) << " " << rd_.q_(4) << " " << rd_.q_(5) << " "
                    //          //  << rd_.q_dot_(0) << " " << rd_.q_dot_(1) << " " << rd_.q_dot_(2) << " " << rd_.q_dot_(3) << " " << rd_.q_dot_(4) << " " << rd_.q_dot_(5) << " "
                    //          //  << rd_.q_ext_(0) << " " << rd_.q_ext_(1) << " " << rd_.q_ext_(2) << " " << rd_.q_ext_(3) << " " << rd_.q_ext_(4) << " " << rd_.q_ext_(5) << " "
                    //          //  << rd_.zmp_global_(0) << " " << rd_.zmp_global_(1) << " "
                    //          //  << zmp_got(0) << " " << zmp_got(1) << " "
                    //          //  << rd_.q_ddot_virtual_(0) << " " << rd_.q_ddot_virtual_(1) << " " << rd_.q_ddot_virtual_(2) << " "
                    //          << std::endl;

                    // std::cout << rd_.link_[COM_id].xpos(1) << std::endl;

                    /*
                    auto ts = std::chrono::steady_clock::now();
                    WBC::GetJKT1(rd_, rd_.J_task);
                    auto ds = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - ts).count();

                    auto ts2 = std::chrono::steady_clock::now();
                    WBC::GetJKT2(rd_, rd_.J_task);
                    auto ds2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - ts2).count();

                    rd_.time_for_inverse += ds;
                    rd_.time_for_inverse_total += ds2;

                    rd_.count_for_inverse++;
                    rd_.count_for_inverse_total++;

                    if (rd_.count_for_inverse == 2000)
                    {
                        std::cout << "avg 1 : " << rd_.time_for_inverse / rd_.count_for_inverse << " 2 : " << rd_.time_for_inverse_total / rd_.count_for_inverse_total << std::endl;

                        rd_.time_for_inverse = 0;
                        rd_.time_for_inverse_total = 0;
                        rd_.count_for_inverse = 0;
                        rd_.count_for_inverse_total = 0;
                    }*/
                }
                else if (rd_.tc_.mode == 2)
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
                            task_log << "time pel_pos_x pel_pos_y pel_pos_z pel_vel_x pel_vel_y pel_vel_z xtraj_x xtraj_y xtraj_z vtraj_x vtraj_y vtraj_z upper_r upper_p upper_y upper_tx upper_ty upper_tz rtraj_r rtraj_p rtraj_y ttraj_x ttraj_y ttraj_z" << std::endl;
                            if (task_log.is_open())
                            {
                                std::cout << "open success " << std::endl;
                            }
                        }

                        std::cout << "mode 2 init" << std::endl;
                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
                    double ang2rad = 0.0174533;

                    rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;

                    rd_.link_[Pelvis].x_desired(0) += rd_.tc_.pelv_pitch;

                    rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;

                    rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, 0 * ang2rad, rd_.link_[Pelvis].yaw_init);


                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                    if (rd_.tc_.customTaskGain)
                    {
                        rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                    }

                    rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].xi_init, rd_.link_[Pelvis].x_desired);
                    rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

                    TaskSpace ts_(6);

                    Eigen::MatrixXd Jtask = rd_.link_[Pelvis].JacCOM();
                    Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[Pelvis], true, true);
                    // std::cout << "1" << std::endl;
                    ts_.Update(Jtask, fstar);
                    WBC::CalcJKT(rd_, ts_);
                    // std::cout << "2" << std::endl;

                    WBC::CalcTaskNull(rd_, ts_);
                    // std::cout << "3" << std::endl;

                    static CQuadraticProgram task_qp_;
                    int hqp1_solve_result = WBC::TaskControlHQP(rd_, ts_, task_qp_, rd_.torque_grav, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), init_qp);

                    // std::cout << "4" << std::endl;
                    // WBC::CalcTaskNull(rd_, ts2_);
                    VectorQd torque_task_hqp_;
                    if (hqp1_solve_result)
                    {
                        // std::cout << "5" << std::endl;

                        Eigen::MatrixXd Jtask2 = rd_.link_[Upper_Body].Jac().bottomRows(3);
                        Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[Upper_Body]);
                        TaskSpace ts2_(3);
                        static CQuadraticProgram task_qp2_;
                        ts2_.Update(Jtask2, fstar2);
                        WBC::CalcJKT(rd_, ts2_);
                        if (WBC::TaskControlHQP(rd_, ts2_, task_qp2_, rd_.torque_grav + ts_.torque_h_, ts_.Null_task, init_qp))
                        {
                            torque_task_hqp_ = rd_.torque_grav + ts_.torque_h_ + ts_.Null_task * ts2_.torque_h_;
                        }
                        else
                        {
                            torque_task_hqp_ = rd_.torque_grav + ts_.torque_h_; // + ts_.Null_task * ts2_.torque_h_;
                        }
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

                    // std::cout << "7" << std::endl;

                    VectorXd out = rd_.lambda * fstar;

                    Vector12d cf_est = WBC::getContactForce(rd_, rd_.torque_desired);

                    Vector3d zmp_got = WBC::GetZMPpos_from_ContactForce(rd_, cf_est);
                    // std::cout << "8" << std::endl;

                    double ur, up, uy, utx, uty, utz;

                    
                    DyrosMath::rot2Euler_tf2(rd_.link_[Upper_Body].rotm, ur, up, uy);
                    DyrosMath::rot2Euler_tf2(rd_.link_[Upper_Body].r_traj, utx, uty, utz);

                    
                    if(rd_.control_time_ > rd_.tc_time_ && rd_.control_time_ < rd_.tc_time_ + rd_.tc_.time+0.5)
                    {
                    task_log << rd_.control_time_ << " "
                             << rd_.link_[Pelvis].xipos(0) << " " << rd_.link_[Pelvis].xipos(1) << " " << rd_.link_[Pelvis].xipos(2) << " "
                             << rd_.link_[Pelvis].vi(0) << " " << rd_.link_[Pelvis].vi(1) << " " << rd_.link_[Pelvis].vi(2) << " "
                             << rd_.link_[Pelvis].x_traj(0) << " " << rd_.link_[Pelvis].x_traj(1) << " " << rd_.link_[Pelvis].x_traj(2) << " "
                             << rd_.link_[Pelvis].v_traj(0) << " " << rd_.link_[Pelvis].v_traj(1) << " " << rd_.link_[Pelvis].v_traj(2) << " "

                             <<ur << " " << up << " " << uy << " "
                             << rd_.link_[Upper_Body].w(0) <<  " "<< rd_.link_[Upper_Body].w(1) <<  " "<< rd_.link_[Upper_Body].w(2) <<  " "

                             <<utx << " " << uty << " " << utz << " "
                             << rd_.link_[Upper_Body].w_traj(0) <<  " "<< rd_.link_[Upper_Body].w_traj(1) <<  " "<< rd_.link_[Upper_Body].w_traj(2) <<  " "

                             //  << rd_.link_[Pelvis].xpos(0) << " " << rd_.link_[Pelvis].xpos(1) << " " << rd_.link_[Pelvis].xpos(2) << " "
                             //  << rd_.link_[Pelvis].v(0) << " " << rd_.link_[Pelvis].v(1) << " " << rd_.link_[Pelvis].v(2) << " "
                             //  << fstar(0) << " " << fstar(1) << " " << fstar(2) << " "
                             //  << out(0) << " " << out(1) << " " << out(2) << " "
                             //  << rd_.link_[COM_id].a_traj(0) << " " << rd_.link_[COM_id].a_traj(1) << " " << rd_.link_[COM_id].a_traj(2) << " "
                             //  << rd_.q_(0) << " " << rd_.q_(1) << " " << rd_.q_(2) << " " << rd_.q_(3) << " " << rd_.q_(4) << " " << rd_.q_(5) << " "
                             //  << rd_.q_dot_(0) << " " << rd_.q_dot_(1) << " " << rd_.q_dot_(2) << " " << rd_.q_dot_(3) << " " << rd_.q_dot_(4) << " " << rd_.q_dot_(5) << " "
                             //  << rd_.q_ext_(0) << " " << rd_.q_ext_(1) << " " << rd_.q_ext_(2) << " " << rd_.q_ext_(3) << " " << rd_.q_ext_(4) << " " << rd_.q_ext_(5) << " "
                             //  << rd_.zmp_global_(0) << " " << rd_.zmp_global_(1) << " "
                             //  << zmp_got(0) << " " << zmp_got(1) << " "
                             //  << rd_.q_ddot_virtual_(0) << " " << rd_.q_ddot_virtual_(1) << " " << rd_.q_ddot_virtual_(2) << " "
                             << std::endl;

                    }

                    init_qp = false;
                }
                else if (rd_.tc_.mode == 3)
                {

                    static bool init_qp;
                    if (rd_.tc_init)
                    {
                        init_qp = true;

                        std::cout << "mode 3 init" << std::endl;
                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
                    double ang2rad = 0.0174533;

                    rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;

                    rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, rd_.tc_.pelv_pitch * ang2rad, rd_.link_[Pelvis].yaw_init);
                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                    if (rd_.tc_.customTaskGain)
                    {
                        rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                    }

                    rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].xi_init, rd_.link_[Pelvis].x_desired);
                    rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

                    Eigen::MatrixXd Jtask = rd_.link_[Pelvis].JacCOM();
                    Eigen::MatrixXd lambda_task;
                    Eigen::MatrixXd Jkt = WBC::GetJKT1(rd_, Jtask, lambda_task);
                    Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[Pelvis], true, true);
                    Eigen::MatrixXd task_null_ = Eigen::MatrixXd::Identity(MODEL_DOF, MODEL_DOF);

                    static CQuadraticProgram task_qp_;
                    Eigen::VectorXd fstar_qp, contact_qp;

                    // WBC::TaskControlHQP(rd_, task_qp_, Jtask, Jkt, fstar, lambda_task, rd_.torque_grav, task_null_, fstar_qp, contact_qp, init_qp);

                    VectorQd torque_task_hqp_ = Jkt * lambda_task * (fstar);

                    // Eigen::VectorXd fstar_qp2, contact_qp2;

                    Eigen::MatrixXd Jtask2 = rd_.link_[Upper_Body].Jac().bottomRows(3);
                    Eigen::MatrixXd lambda_task2;
                    Eigen::MatrixXd Jkt2 = WBC::GetJKT1(rd_, Jtask2, lambda_task2);
                    Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[Upper_Body]);
                    Eigen::MatrixXd task_null_2 = (task_null_ - Jkt * lambda_task * Jtask * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF));

                    // static CQuadraticProgram task_qp_2;
                    // WBC::TaskControlHQP(rd_, task_qp_2, Jtask2, Jkt2, fstar2, lambda_task2, torque_task_hqp_ + rd_.torque_grav, task_null_2, fstar_qp2, contact_qp2, init_qp);

                    VectorQd torque_Task2 = torque_task_hqp_ + task_null_2 * (Jkt2 * lambda_task2 * (fstar2));

                    // static CQuadraticProgram contact_qp_;

                    // VectorXd torque_contact_qp_;
                    // WBC::CalcContactRedistributeHQP(rd_, contact_qp_, torque_Task2 + rd_.torque_grav, torque_contact_qp_, init_qp);

                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, torque_Task2 + rd_.torque_grav);

                    VectorXd out = rd_.lambda * fstar;

                    Vector12d cf_est = WBC::getContactForce(rd_, rd_.torque_desired);

                    Vector3d zmp_got = WBC::GetZMPpos_from_ContactForce(rd_, cf_est);

                    init_qp = false;
                }
                else if (rd_.tc_.mode == 4)
                {
                    double ang2rad = 0.0174533;

                    static bool init_qp;
                    if (rd_.tc_init)
                    {
                        init_qp = true;

                        std::cout << "mode 4 init" << std::endl;
                        rd_.tc_init = false;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
                    if (rd_.tc_.customTaskGain)
                    {
                        rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                        rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                    }

                    rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
                    rd_.link_[Pelvis].rot_desired = DyrosMath::rotateWithY(rd_.tc_.pelv_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.link_[Pelvis].yaw_init);

                    rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init;
                    rd_.link_[Right_Hand].x_desired(0) += rd_.tc_.r_x;
                    rd_.link_[Right_Hand].x_desired(1) += rd_.tc_.r_y;
                    rd_.link_[Right_Hand].x_desired(2) += rd_.tc_.r_z;
                    rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * ang2rad) * DyrosMath::Euler2rot(0, 1.5708, -1.5708).transpose();
                    // rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * ang2rad);

                    rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.yaw * ang2rad);

                    rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].xi_init, rd_.link_[Pelvis].x_desired);
                    rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
                    rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

                    TaskSpace ts_(6);
                    Eigen::MatrixXd Jtask = rd_.link_[Pelvis].JacCOM();
                    Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[Pelvis], true, true);

                    ts_.Update(Jtask, fstar);
                    WBC::CalcJKT(rd_, ts_);
                    WBC::CalcTaskNull(rd_, ts_);
                    static CQuadraticProgram task_qp_;
                    WBC::TaskControlHQP(rd_, ts_, task_qp_, rd_.torque_grav, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), init_qp);

                    VectorQd torque_Task2 = ts_.torque_h_ + rd_.torque_grav;

                    TaskSpace ts1_(6);
                    Eigen::MatrixXd Jtask1 = rd_.link_[Right_Hand].Jac();
                    Eigen::VectorXd fstar1 = WBC::GetFstar6d(rd_.link_[Right_Hand], true);

                    ts1_.Update(Jtask1, fstar1);
                    WBC::CalcJKT(rd_, ts1_);
                    WBC::CalcTaskNull(rd_, ts1_);
                    static CQuadraticProgram task_qp1_;
                    WBC::TaskControlHQP(rd_, ts1_, task_qp1_, torque_Task2, ts_.Null_task, init_qp);

                    torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + rd_.torque_grav;

                    TaskSpace ts2_(3);
                    Eigen::MatrixXd Jtask2 = rd_.link_[Upper_Body].Jac().bottomRows(3);
                    Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[Upper_Body]);
                    ts2_.Update(Jtask2, fstar2);
                    WBC::CalcJKT(rd_, ts2_);

                    static CQuadraticProgram task_qp2_;
                    WBC::TaskControlHQP(rd_, ts2_, task_qp2_, torque_Task2, ts_.Null_task * ts1_.Null_task, init_qp);
                    // WBC::TaskControlHQP(rd_, task_qp_2, Jtask2, Jkt2, fstar2, lambda_task2, torque_task_hqp_ + rd_.torque_grav, ts_.Null_task, fstar_qp2, contact_qp2, init_qp);

                    // VectorQd torque_Task2 = ts_.J_kt_ *ts_.Lambda_task_ * ts_.f_star_;

                    torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + ts_.Null_task * ts1_.Null_task * ts2_.torque_h_ + rd_.torque_grav;

                    VectorXd torque_contact_qp_;
                    // WBC::CalcContactRedistributeHQP(rd_, contact_qp_, torque_Task2 + rd_.torque_grav, torque_contact_qp_, init_qp);

                    // rd_.torque_desired = torque_Task2 + rd_.torque_grav + torque_contact_qp_;

                    // rd_.torque_desired = torque_Task2 + rd_.torque_grav + rd_.NwJw * ts2_.contact_qp_;
                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, torque_Task2);

                    VectorXd out = rd_.lambda * fstar;

                    Vector12d cf_est = WBC::getContactForce(rd_, rd_.torque_desired);

                    Vector3d zmp_got = WBC::GetZMPpos_from_ContactForce(rd_, cf_est);

                    init_qp = false;
                }

#ifdef COMPILE_TOCABI_AVATAR
                if ((rd_.tc_.mode > 9) && (rd_.tc_.mode < 15))
                {
                    RequestThread2();
                    ac_.computeSlow();

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
                // if ((rd_.tc_.mode > 9) && (rd_.tc_.mode < 15))
                // {
                //     RequestThread2();
                //     my_cc.computeSlow();
                // }
#endif
            }
            else
            {
                WBC::SetContact(rd_, 1, 1);
                rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
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

            // Eigen::VectorXd cf_from_torque;
            // cf_from_torque.resize(rd_.contact_index * 6);
            // cf_from_torque = WBC::getContactForce(rd_, rd_.torque_desired);
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

            // std::cout<<"21"<<std::endl;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    // cout << "thread1 terminate" << endl;
    return (void *)NULL;
}

// Thread2 : running with request
void *TocabiController::Thread2()
{
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
                        ac_.computeFast();
                    }
#endif
                    // #ifdef COMPILE_TOCABI_CC
                    //                     if (rd_.tc_.mode == 15)
                    //                     {
                    //                         my_cc.computeFast();
                    //                     }
                    // #endif
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
    rd_.pc_mode = false;
    rd_.tc_ = *msg;
    std::cout << " CNTRL : task signal received mode :" << rd_.tc_.mode << std::endl;
    stm_.StatusPub("%f task Control mode : %d", (float)rd_.control_time_, rd_.tc_.mode);
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
        std::cout << " CNTRL : State Estimate is not running. disable task command" << std::endl;
        rd_.tc_run = false;
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
    std::cout << "task que received ... but doing nothing .." << std::endl;
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
