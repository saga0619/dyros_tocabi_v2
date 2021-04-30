#include "tocabi_controller/tocabi_controller.h"

using namespace std;

TocabiController::TocabiController(StateManager &stm_global) : dc_(stm_global.dc_), stm_(stm_global), rd_(stm_global.dc_.rd_)
{
    cout << "TocabiController Initialized" << endl;
}

TocabiController::~TocabiController()
{
    cout << "TocabiController Terminated" << endl;
}

void *TocabiController::thread1()
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

    int64_t total1, total2;
    int64_t total_dev1, total_dev2;
    float lmax, lmin, ldev, lavg, lat;
    float smax, smin, sdev, savg, sat;

    total1 = 0;
    total2 = 0;
    total_dev1 = 0;
    total_dev2 = 0;

    ldev = 0.0;
    lavg = 0.0;
    lat = 0;

    lmax = 0.0;
    lmin = 10000.00;
    smax = 0.0;
    smin = 100000.0;

    sdev = 0;
    savg = 0;
    sat = 0;

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

        SendCommand(rd_.torque_grav + rd_.torque_contact);

        //std::cout<<"21"<<std::endl;

        sat = d1;
        total2 += sat;
        savg = total2 / thread1_count;
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
        sdev = total_dev2 / thread1_count;

        dc_.tc_shm_->send_avg2 = savg;
        dc_.tc_shm_->send_max2 = smax;
        dc_.tc_shm_->send_min2 = smin;
        dc_.tc_shm_->send_dev2 = sdev;
    }

    cout << "thread1 terminate" << endl;
}

void *TocabiController::thread2()
{
    std::cout << "thread2_entered" << std::endl;

    while (!dc_.tc_shm_->shutdown)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "thread2 terminate" << std::endl;
}

void *TocabiController::thread3()
{
}

void TocabiController::SendCommand(Eigen::VectorQd torque_command)
{
    dc_.tc_shm_->commanding = true;

    //std::cout<<torque_command.transpose()<<std::endl;
    //memcpy(dc_.tc_shm_->torqueCommand, torque_command.data(), torque_command.size() * sizeof(float));

    for (int i = 0; i < MODEL_DOF; i++)
    {
        dc_.tc_shm_->torqueCommand[i] = torque_command[i];
    }

    //std::cout<<dc_.tc_shm_->torqueCommand[0]<<"\t"<<dc_.tc_shm_->torqueCommand[1]<<"\t"<<dc_.tc_shm_->torqueCommand[2]<<"\t"<<dc_.tc_shm_->torqueCommand[3]<<"\t"<<dc_.tc_shm_->torqueCommand[4]<<"\t"<<dc_.tc_shm_->torqueCommand[5]<<"\t"<<std::endl;
    dc_.tc_shm_->commanding = false;
}

void TocabiController::GetTaskCommand(tocabi_msgs::TaskCommand &msg)
{

}

/*
void TocabiController::SetCommand()
{

}*/