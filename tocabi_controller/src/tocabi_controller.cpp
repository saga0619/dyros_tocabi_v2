#include "tocabi_controller/tocabi_controller.h"

using namespace std;

TocabiController::TocabiController(StateManager &stm_global) : dc_(stm_global.dc_), stm_(stm_global), rd_(*stm_global.dc_.rd_)
{
    //Tocabi Controller Initialize Component

    nh_controller_.setCallbackQueue(&queue_controller_);
    //sub_1 = nh_controller_.subscribe("/tocabi/avatar_test", 1, &AvatarController::avatar_callback, this);

    task_command_sub_ = nh_controller_.subscribe("/tocabi/taskcommand", 100, &TocabiController::TaskCommandCallback, this);
    task_command_que_sub_ = nh_controller_.subscribe("/tocabi/taskquecommand", 100, &TocabiController::TaskQueCommandCallback, this);
    position_command_sub_ = nh_controller_.subscribe("/tocabi/positioncommand", 100, &TocabiController::PositionCommandCallback, this);

    ros::param::get("/tocabi_controller/Kp", rd_.pos_kp_v);
    ros::param::get("/tocabi_controller/Kv", rd_.pos_kv_v);
}

TocabiController::~TocabiController()
{
    cout << "TocabiController Terminated" << endl;
}

// Thread1 : running
void *TocabiController::Thread1() //Thread1, running with 2Khz.
{
    std::cout << "thread1_entered" << std::endl;

    volatile int rcv_time_ = 0;
    //cout << "shm_msgs:" << dc_.tc_shm_->t_cnt << endl;
    //cout << "entered" << endl;

    if (dc_.tc_shm_->shutdown)
    {
        cout << "what?" << endl;
    }

    //cout << "waiting first calc.." << endl;
    while (!rd_.firstCalc)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        if (dc_.tc_shm_->shutdown)
        {
            break;
        }
    }

    std::cout << "thread1 Proceeding ... " << endl;

    WBC::SetContactInit(rd_);

    EnableThread2(true);  //Set true for Thread2
    EnableThread3(false); //True for thread3 ...

    //std::cout<<"21"<<std::endl;

    std::cout << "entering thread1 loop" << endl;

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
            rcv_time_ = rd_.us_from_start_;

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

            if (dc_.positionControlSwitch)
            {
                dc_.positionControlSwitch = false;
                rd_.q_desired = rd_.q_;
                rd_.positionHoldSwitch = true;
                rd_.pc_mode = true;

                std::cout << "position hold switch " << std::endl;

                for (int i = 0; i < MODEL_DOF; i++)
                {
                    std::cout << rd_.pos_kp_v[i] << "  " << rd_.pos_kv_v[i] << std::endl;
                }
            }

            if (rd_.pc_mode)
            {
                if (rd_.positionHoldSwitch)
                {
                }
                else
                {
                    rd_.q_desired = DyrosMath::cubicVector(rd_.control_time_, rd_.pc_time_, rd_.pc_time_ + rd_.pc_traj_time_, rd_.pc_pos_init, rd_.pc_pos_des, zero_m, zero_m);
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

                    WBC::SetContact(rd_, 1, 1);

                    rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac();
                    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

                    rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

                    rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll, rd_.tc_.pitch, rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

                    Eigen::VectorXd fstar;
                    rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    fstar.setZero(9);
                    fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id]);
                    fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

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


            }
            else
            {
                WBC::SetContact(rd_, 1, 1);
                rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            }

            //Send Data To thread2

            //Data2Thread2

            //std::cout << torque_task_.norm() << "\t" << torque_grav_.norm() << "\t" << torque_contact_.norm() << std::endl;

            static std::chrono::steady_clock::time_point t_c_ = std::chrono::steady_clock::now();

            SendCommand(rd_.torque_desired);

            auto t_end = std::chrono::steady_clock::now();

            auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t1).count();            //150us without march=native
            auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(t_end - rd_.tp_state_).count(); //150us without march=native

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

            if (thread1_count % 2000 == 0)
            {
                /*
                WBC::SetContact(rd_, 1, 1);

                WBC::SetContact(rd_, 1, 0);

                WBC::SetContact(rd_, 0, 1);*/

                // std::cout << rd_.control_time_ << "s : avg rcv2send : " << d2_total / thread1_count << " us, state : " << rd_.state_ctime_total_ / thread1_count << " controller : " << d1_total / thread1_count << " diff : " << (d2_total - rd_.state_ctime_total_ - d1_total) / thread1_count << std::endl;

                if (d1_over_cnt > 0)
                {
                    std::cout << cred << "Controller Thread1 calculation time over 500us.. : " << d1_over_cnt << "times" << creset << std::endl;
                    d1_over_cnt = 0;
                }

                d1_total = 0;
                d2_total = 0;
                rd_.state_ctime_total_ = 0;
                thread1_count = 0;
            }
            t_c_ = std::chrono::steady_clock::now();

            //std::cout<<"21"<<std::endl;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }

    cout << "thread1 terminate" << endl;
}

//Thread2 : running with request
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
        std::cout << "thread2_entered" << std::endl;
        while (!dc_.tc_shm_->shutdown)
        {
            if (triggerThread2)
            {
                triggerThread2 = false;
                /////////////////////////////////////////////
                /////////////Do something in Thread2 !!!!!!!

                if (rd_.tc_run)
                {


                }
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
        std::cout << "thread2 disabled" << std::endl;
    }

    std::cout << "thread2 terminate" << std::endl;
}

//Thread3 : running with request
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
    rd_.pc_traj_time_ = msg->traj_time;

    rd_.pc_time_ = rd_.control_time_;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        rd_.pc_pos_des(i) = msg->position[i];
    }
    rd_.pc_pos_init = rd_.q_;
    rd_.pc_mode = true;
    rd_.pc_gravity = msg->gravity;
    rd_.positionHoldSwitch = false;

    std::cout << "position command received" << std::endl;
}

void TocabiController::TaskCommandCallback(const tocabi_msgs::TaskCommandConstPtr &msg)
{
    std::cout << "task signal received mode :" << rd_.tc_.mode << std::endl;
    rd_.pc_mode = false;
    rd_.tc_ = *msg;
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

    double pos_p = 400.0;
    double pos_d = 40.0;
    double pos_a = 1;
    double rot_p = 400.0;
    double rot_d = 40.0;
    double rot_a = 1.0;

    rd_.link_[Right_Foot].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    rd_.link_[Left_Foot].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    rd_.link_[Right_Hand].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    rd_.link_[Left_Hand].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    rd_.link_[Pelvis].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    rd_.link_[Upper_Body].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);
    rd_.link_[COM_id].SetGain(pos_p, pos_d, pos_a, rot_p, rot_d, rot_a);

    //std::cout << " pelv yaw init : " << rd_.link_[Pelvis].yaw_init << std::endl;

    //std::cout << "upperbody rotation init : " << DyrosMath::rot2Euler_tf(rd_.link_[Upper_Body].rot_init).transpose() << std::endl;

    if (!rd_.semode)
    {
        std::cout << "State Estimate is not running. disable task command" << std::endl;
        rd_.tc_run = false;
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