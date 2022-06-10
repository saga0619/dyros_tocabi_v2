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

    EnableThread2(true);  // Set true for Thread2
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
        std::cout << "set with realrobot gain" << std::endl;
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
                if (rd_.tc_.mode == 0)
                {
                    if (rd_.tc_init)
                    {
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
                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    fstar.setZero(6);
                    fstar.segment(0, 3) = WBC::GetFstarPos(rd_.link_[COM_id], true);
                    fstar.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));

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
                        std::cout << "mode 1 init" << std::endl;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, 1, 1);

                    rd_.J_task.setZero(6, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac().block(0, 0, 3, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(3, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

                    rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll, rd_.tc_.pitch, rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

                    Eigen::VectorXd fstar;
                    rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    fstar.setZero(6);
                    fstar.segment(0, 3) = WBC::GetFstarPos(rd_.link_[COM_id]);
                    fstar.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                    // rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) );

                    VectorQd task_torque_qp = WBC::TaskControlTorqueQP(rd_, fstar, rd_.tc_init);

                    rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + task_torque_qp);

                    // std::cout << (task_torque_qp - task_torque_qp2).transpose() << std::endl;

                    rd_.tc_init = false;
                }
                else if (rd_.tc_.mode == 2)
                {
                    if (rd_.tc_init)
                    {
                        std::cout << "mode 2 init : contact " << std::endl;
                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);

                    rd_.J_task.setZero(6, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac().block(0, 0, 3, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(3, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

                    rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll, rd_.tc_.pitch, rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

                    Eigen::VectorXd fstar;
                    rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    fstar.setZero(6);
                    fstar.segment(0, 3) = WBC::GetFstarPos(rd_.link_[COM_id]);
                    fstar.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                    // rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) );

                    VectorQd task_torque_qp = WBC::TaskControlTorqueQP(rd_, fstar, rd_.tc_init);

                    VectorQd gravity_torque_qp = WBC::GravityCompensationTorque(rd_);

                    VectorQd contact_torque_qp = WBC::ContactTorqueQP(rd_, task_torque_qp + gravity_torque_qp);

                    rd_.torque_desired = task_torque_qp + gravity_torque_qp + contact_torque_qp;

                    // std::cout << (task_torque_qp - task_torque_qp2).transpose() << std::endl;

                    rd_.tc_init = false;
                }

#ifdef COMPILE_TOCABI_AVATAR
                if ((rd_.tc_.mode > 9) && (rd_.tc_.mode < 15))
                {
                    RequestThread2();

                    static int thread3_count = 0;

                    thread3_count++;
                    if (thread3_count == 50)
                    {
                        thread3_count = 0;

                        RequestThread3();
                    }

                    ac_.computeSlow();

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
            if (dc_.simMode)
                WBC::CheckTorqueLimit(rd_, rd_.torque_desired);

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
#ifdef COMPILE_TOCABI_CC
                    if (rd_.tc_.mode == 15)
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
        std::cout << "thread2 disabled" << std::endl;
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
        std::cout << "thread3_entered" << std::endl;

        while (!dc_.tc_shm_->shutdown)
        {
            if (triggerThread3)
            {
                triggerThread3 = false;
                /////////////////////////////////////////////
                /////////////Do something in Thread3 !!!!!!!

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
        std::cout << "thread3 disabled" << std::endl;
    }

    std::cout << "thread3 terminate" << std::endl;
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