#include "tocabi_controller/tocabi_controller.h"

using namespace std;

TocabiController::TocabiController(StateManager &stm_global) : dc_(stm_global.dc_), stm_(stm_global), rd_(stm_global.dc_.rd_)
{
    //Tocabi Controller Initialize Component
}

TocabiController::~TocabiController()
{
    cout << "TocabiController Terminated" << endl;
}

// Thread1 : running
void *TocabiController::Thread1()
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

    CustomController my_cc(rd_);

    //std::cout<<"21"<<std::endl;

    std::cout << "entering thread1 loop" << endl;

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

            //////////////////////////////////////////////////////////
            ////////////////////Start Tocabi Controll/////////////////
            //////////////////////////////////////////////////////////

            if (rd_.task_signal_ || rd_.task_que_signal_)
            {
                std::cout << "task signal received" << std::endl;
                rd_.tc_time_ = rd_.control_time_;
                rd_.tc_run = true;
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

                if (rd_.task_signal_)
                {
                    rd_.task_signal_ = false;
                }

                if (rd_.task_que_signal_)
                {
                    std::cout << "task que received ... but doing nothing .." << std::endl;
                    rd_.task_que_signal_ = false;
                }
            }

            VectorQd torque_task_, torque_grav_, torque_contact_;
            torque_task_.setZero();
            torque_grav_.setZero();
            torque_contact_.setZero();

            if (rd_.tc_run)
            {
                if (rd_.tc_.mode == 0)
                {
                    if (rd_.tc_init)
                    {

                        rd_.tc_init = false;

                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                    WBC::SetContact(rd_, 1, 1);
                    torque_grav_ = WBC::GravityCompensationTorque(rd_);

                    rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
                    rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac();
                    rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

                    rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                    rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

                    rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll) * DyrosMath::rotateWithY(rd_.tc_.pitch) * DyrosMath::rotateWithZ(rd_.tc_.yaw);

                    Eigen::VectorXd fstar;
                    rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                    fstar.setZero(9);

                    rd_.link_[COM_id].r_traj = Eigen::Matrix3d::Identity();
                    fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id]);
                    fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                    //std::cout << rd_.link_[COM_id].r_traj << "\t" << rd_.link_[COM_id].rotm << std::endl;

                    //Quaterniond q_pelv(rd_.link_[COM_id].rotm);

                    //q_pelv.

                    //std::cout << fstar.transpose() << std::endl;

                    torque_task_ = WBC::TaskControlTorque(rd_, fstar);

                    //qstd::cout << rd_.link_[COM_id].x_traj.transpose() <<"\t"<< fstar.transpose() <<std::endl;

                    //rd_.link_[COM_id].SetTrajectory()
                }
                else if (rd_.tc_.mode > 10)
                {
                    my_cc.computeSlow();
                }
            }
            else
            {
                WBC::SetContact(rd_, 1, 1);
                WBC::GravityCompensationTorque(rd_);
                torque_grav_ = rd_.torque_grav;
            }

            torque_contact_ = WBC::contact_force_redistribution_torque(rd_, torque_grav_ + torque_task_);

            auto d1 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t1).count(); //150us without march=native

            //Send Data To thread2

            //Data2Thread2

            rd_.torque_desired = torque_task_ + torque_grav_ + torque_contact_;

            //std::cout << torque_task_.norm() << "\t" << torque_grav_.norm() << "\t" << torque_contact_.norm() << std::endl;

            static std::chrono::steady_clock::time_point t_c_ = std::chrono::steady_clock::now();

            SendCommand(rd_.torque_desired);

            auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - dc_.rd_.rc_t_).count(); //150us without march=native

            static int d2_total = 0;

            d2_total += d2;

            // if (d2 > 350)
            // {
            //     std::cout << rd_.control_time_ << "command duration over 350us , " << d2 << std::endl;
            // }

            if (thread1_count % 2000 == 0)
            {
                std::cout << rd_.control_time_ << "s : avg rcv2send : " << d2_total / thread1_count << " us" << std::endl;
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

//Thread2 : running with 2khz matching
void *TocabiController::Thread2()
{
    std::cout << "thread2_entered" << std::endl;

    while (!dc_.tc_shm_->shutdown)
    {
    }

    std::cout << "thread2 terminate" << std::endl;
}

void *TocabiController::Thread3()
{
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
    const double maxTorque = 1000.0;
    const double rTime = 6.0;

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
            dc_.torqueOnTime = dc_.rd_.control_time_;
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
            dc_.torqueOffTime = dc_.rd_.control_time_;
            dc_.toruqeDecreaseSeq = true;
        }
        else
        {
            std::cout << "torque is already off" << std::endl;
        }
    }

    dc_.tc_shm_->commanding = true;

    if (dc_.torqueOn)
    {
        if (dc_.torqueRisingSeq)
        {
            dc_.tc_shm_->maxTorque = (int)(maxTorque * DyrosMath::minmax_cut((dc_.rd_.control_time_ - dc_.torqueOnTime) / rTime, 0, 1));

            if (dc_.rd_.control_time_ > dc_.torqueOnTime + rTime)
            {
                std::cout << "torque 100% ! " << std::endl;

                dc_.torqueRisingSeq = false;
            }
        }
        else if (dc_.toruqeDecreaseSeq)
        {

            dc_.tc_shm_->maxTorque = (int)(maxTorque * (1 - DyrosMath::minmax_cut((dc_.rd_.control_time_ - dc_.torqueOffTime) / rTime, 0, 1)));

            if (dc_.rd_.control_time_ > dc_.torqueOffTime + rTime)
            {
                dc_.toruqeDecreaseSeq = false;

                std::cout << "torque 0% .. torque Off " << std::endl;

                dc_.torqueOn = false;
            }
        }
        else
        {
            dc_.tc_shm_->maxTorque = (int)maxTorque;
        }
    }
    else
    {
        dc_.tc_shm_->maxTorque = 0;
    }

    //std::cout<<torque_command.transpose()<<std::endl;
    //memcpy(dc_.tc_shm_->torqueCommand, torque_command.data(), torque_command.size() * sizeof(float));

    if (dc_.E1Switch) //Emergency stop
    {
        if (dc_.E1Status)
        {
            dc_.E1Status = false;
        }
        else
        {
            rd_.q_desired = rd_.q_;
            rd_.q_dot_desired.setZero();
            dc_.E1Status = true;
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

            //Damping mode = true!
        }

        dc_.E2Switch = false;
    }
    if (dc_.emergencySwitch)
    {
        dc_.emergencyStatus = true;
    }

    memset(dc_.tc_shm_->commandMode, 1, sizeof(dc_.tc_shm_->commandMode));
    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc_.tc_shm_->torqueCommand[i] = torque_command[i];
    }

    if (dc_.E1Status)
    {
        memset(dc_.tc_shm_->commandMode, 1, sizeof(dc_.tc_shm_->commandMode));
        for (int i = 0; i < MODEL_DOF; i++)
        {
            dc_.tc_shm_->torqueCommand[i] = dc_.Kps[i] * (dc_.rd_.q_desired(i) - dc_.rd_.q_(i)) + dc_.Kvs[i] * (dc_.rd_.q_dot_desired(i) - dc_.rd_.q_dot_(i));
        }
    }

    if (dc_.E2Status)
    {
        memset(dc_.tc_shm_->commandMode, 1, sizeof(dc_.tc_shm_->commandMode));
        for (int i = 0; i < MODEL_DOF; i++)
            dc_.tc_shm_->torqueCommand[i] = - 4.0 * dc_.Kvs[i] * dc_.rd_.q_dot_(i);
    }

    if (dc_.emergencyStatus)
    {
        memset(dc_.tc_shm_->commandMode, 1, sizeof(dc_.tc_shm_->commandMode));
        for (int i = 0; i < MODEL_DOF; i++)
            dc_.tc_shm_->torqueCommand[i] = 0.0;
    }
    //std::cout<<dc_.tc_shm_->torqueCommand[0]<<"\t"<<dc_.tc_shm_->torqueCommand[1]<<"\t"<<dc_.tc_shm_->torqueCommand[2]<<"\t"<<dc_.tc_shm_->torqueCommand[3]<<"\t"<<dc_.tc_shm_->torqueCommand[4]<<"\t"<<dc_.tc_shm_->torqueCommand[5]<<"\t"<<std::endl;

    dc_.tc_shm_->commandCount++;
    dc_.tc_shm_->commanding = false;
}

void TocabiController::GetTaskCommand(tocabi_msgs::TaskCommand &msg)
{
}

/*
void TocabiController::SetCommand()
{

}*/