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

    //std::cout<<"21"<<std::endl;

    std::cout << "entering thread1 loop" << endl;

    int thread1_count = 0;
    while (!dc_.tc_shm_->shutdown)
    {
        thread1_count++;
        while (rcv_time_ >= rd_.us_from_start_ && (!dc_.tc_shm_->shutdown))
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        if (dc_.tc_shm_->shutdown)
            break;
        rcv_time_ = rd_.us_from_start_;

        auto t1 = std::chrono::steady_clock::now();

        //Start Tocabi Controll

        if (rd_.task_signal_)
        {
            std::cout << "task signal received" << std::endl;
            rd_.task_signal_ = false;
        }
        if (rd_.task_que_signal_)
        {
            std::cout << "task que signal received" << std::endl;
            rd_.task_signal_ = false;
        }

        WBC::SetContact(rd_, 1, 1);
        WBC::gravity_compensation_torque(rd_);

        Vector12d fc_redis;
        fc_redis.setZero();
        double fc_ratio = 0.9;
        rd_.torque_desired = rd_.torque_grav;

        WBC::contact_force_redistribution_torque(rd_, rd_.torque_desired, fc_redis, fc_ratio);

        auto d1 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t1).count(); //150us without march=native

        //Send Data To thread2

        //Data2Thread2
        static std::chrono::steady_clock::time_point t_c_ = std::chrono::steady_clock::now();
        SendCommand(rd_.torque_grav + rd_.torque_contact);

        auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - dc_.rd_.rc_t_).count(); //150us without march=native

        static int d2_total = 0;

        d2_total += d2;

        if (d2 > 350)
        {
            std::cout << rd_.control_time_ << "command duration over 350us , " << d2 << std::endl;
        }

        if (thread1_count % 2000 == 0)
        {
            std::cout << rd_.control_time_ << "s : avg rcv2send : " << d2_total / thread1_count << " us" << std::endl;
        }
        t_c_ = std::chrono::steady_clock::now();

        //std::cout<<"21"<<std::endl;
    }

    cout << "thread1 terminate" << endl;
}

void *TocabiController::Thread2()
{
    std::cout << "thread2_entered" << std::endl;

    while (!dc_.tc_shm_->shutdown)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
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

    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc_.tc_shm_->commandMode[i] = 1;
        dc_.tc_shm_->torqueCommand[i] = torque_command[i];
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