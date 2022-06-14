#include "tocabi_controller/tocabi_controller.h"

#include "qp.h"

using namespace std;
using namespace TOCABI;

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
void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{
    Eigen::Matrix6x12d W;
    W.setZero();

    W.block(0, 0, 6, 6).setIdentity();
    W.block(0, 6, 6, 6).setIdentity();
    W.block(3, 0, 3, 3) = DyrosMath::skm(P1);
    W.block(3, 6, 3, 3) = DyrosMath::skm(P2);

    ResultantForce = W * F12; // F1F2;

    double eta_lb = 1.0 - eta_cust;
    double eta_ub = eta_cust;
    // printf("1 lb %f ub %f\n",eta_lb,eta_ub);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // boundary of eta Mx, A*eta + B < 0
    double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
    double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
    double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
    double a = A * A;
    double b = 2.0 * A * B;
    double c = B * B - C * C;
    double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
        {
            eta_ub = sol_eta1;
        }

        if (sol_eta2 > eta_lb)
        {
            eta_lb = sol_eta2;
        }
    }
    else // sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
        {
            eta_ub = sol_eta2;
        }

        if (sol_eta1 > eta_lb)
        {
            eta_lb = sol_eta1;
        }
    }

    // printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // boundary of eta My, A*eta + B < 0
    A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
    B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
    C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
            eta_ub = sol_eta1;

        if (sol_eta2 > eta_lb)
            eta_lb = sol_eta2;
    }
    else // sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
            eta_ub = sol_eta2;

        if (sol_eta1 > eta_lb)
            eta_lb = sol_eta1;
    }

    // printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
    A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
    B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
    C = staticFrictionCoeff * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
    {
        if (sol_eta1 < eta_ub)
            eta_ub = sol_eta1;
        if (sol_eta2 > eta_lb)
            eta_lb = sol_eta2;
    }
    else // sol_eta2 ÀÌ upper boundary
    {
        if (sol_eta2 < eta_ub)
            eta_ub = sol_eta2;
        if (sol_eta1 > eta_lb)
            eta_lb = sol_eta1;
    }
    // printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

    double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

    eta = eta_s;
    if (eta_s > eta_ub)
        eta = eta_ub;
    else if (eta_s < eta_lb)
        eta = eta_lb;

    if ((eta > eta_cust) || (eta < 1.0 - eta_cust))
        eta = 0.5;

    ForceRedistribution(0) = eta * ResultantForce(0);
    ForceRedistribution(1) = eta * ResultantForce(1);
    ForceRedistribution(2) = eta * ResultantForce(2);
    ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
    ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
    ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
    ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
    ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
    ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
    ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
    ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
    ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
    // ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
    // ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
    // ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

VectorQd ContactForceRedistributionTorque(RobotData &Robot, VectorQd command_torque, double eta)
{
    int contact_dof_ = Robot.J_C.rows();

    VectorQd torque_redis;

    if (contact_dof_ == 12)
    {
        Vector12d ContactForce_ = Robot.J_C_INV_T.rightCols(MODEL_DOF) * command_torque - Robot.P_C;

        Vector3d P1_ = Robot.cc_[0].xc_pos - Robot.link_[COM_id].xpos;
        Vector3d P2_ = Robot.cc_[1].xc_pos - Robot.link_[COM_id].xpos;

        Matrix3d Rotyaw = DyrosMath::rotateWithZ(-Robot.yaw);

        Eigen::Matrix<double, 12, 12> force_rot_yaw;
        force_rot_yaw.setZero();
        for (int i = 0; i < 4; i++)
        {
            force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
        }

        Vector6d ResultantForce_;
        ResultantForce_.setZero();

        Vector12d ResultRedistribution_;
        ResultRedistribution_.setZero();

        Vector12d F12 = force_rot_yaw * ContactForce_;

        double eta_cust = 0.99;
        double foot_length = 0.26;
        double foot_width = 0.1;

        ForceRedistributionTwoContactMod2(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, Rotyaw * P1_, Rotyaw * P2_, F12, ResultantForce_, ResultRedistribution_, eta);
        Robot.fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

        Vector12d desired_force;
        desired_force.setZero();

        desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + Robot.fc_redist_.segment(6, 6);
        torque_redis = Robot.V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).bottomRows(6) * Robot.V2.transpose()).inverse() * desired_force.segment(6, 6);
    }
    else
    {
        torque_redis.setZero();
    }

    return torque_redis;
}

VectorQd contact_redis_test(RobotData &rd_, VectorQd Control_Torque)
{
    int contact_index = 2;
    rd_.contact_index = 2;
    Eigen::MatrixXd crot_matrix;
    Eigen::MatrixXd RotW;

    crot_matrix.setZero(rd_.contact_index * 6, rd_.contact_index * 6);

    RotW.setIdentity(rd_.contact_index * 6, rd_.contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {
        Vector3d cv = rd_.cc_[i].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.cc_[i].xc_pos);
        Matrix3d cm = DyrosMath::rotateWithX(-atan(cv(1) / cv(2))) * DyrosMath::rotateWithY(atan(cv(0) / sqrt(cv(1) * cv(1) + cv(2) * cv(2))));
        cm.setIdentity();

        crot_matrix.block(i * 6, i * 6, 3, 3) = crot_matrix.block(i * 6 + 3, i * 6 + 3, 3, 3) = cm.transpose() * rd_.cc_[i].rotm.transpose();
        RotW(i * 6 + 2, i * 6 + 2) = 0;
    }

    // RotM_.setZero();

    // RotM_.block(0, 0, 3, 3) = crot_l.transpose() * rd_.link_[Left_Foot].rotm.transpose();
    // RotM_.block(3, 3, 3, 3) = crot_l.transpose() * rd_.link_[Left_Foot].rotm.transpose();

    // RotM_.block(6, 6, 3, 3) = crot_r.transpose() * rd_.link_[Right_Foot].rotm.transpose();
    // RotM_.block(9, 9, 3, 3) = crot_r.transpose() * rd_.link_[Right_Foot].rotm.transpose();

    // Vector12d COM_Rel_ContactForce;

    // COM_Rel_ContactForce = RotM_ * Fc;

    static CQuadraticProgram qp_torque_contact_;
    static int contact_dof, const_num;

    const_num = 4;

    qp_torque_contact_.InitializeProblemSize(rd_.contact_index * 6 - 6, rd_.contact_index * const_num);

    static MatrixXd H;
    static VectorXd g;
    static MatrixXd A_t;

    // std::cout<<"1"<<std::endl;

    MatrixXd NwJw = rd_.V2.transpose() * (rd_.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * rd_.V2.transpose()).inverse();

    // std::cout<<"2"<<std::endl;
    A_t = RotW * crot_matrix * rd_.J_C_INV_T.rightCols(MODEL_DOF) * NwJw;

    // std::cout<<"3"<<std::endl;
    H = A_t.transpose() * A_t;

    // std::cout<<"4"<<std::endl;
    g = (RotW * crot_matrix * (rd_.J_C_INV_T.rightCols(MODEL_DOF) * Control_Torque - rd_.P_C)).transpose() * A_t;

    // std::cout<<"5"<<std::endl;
    qp_torque_contact_.UpdateMinProblem(H, g);

    // Constraint

    MatrixXd A_const_a;
    A_const_a.setZero(const_num * rd_.contact_index, 6 * rd_.contact_index);

    MatrixXd A__mat;

    MatrixXd A_rot;

    A_rot.setZero(rd_.contact_index * 6, rd_.contact_index * 6);

    for (int i = 0; i < rd_.contact_index; i++)
    {
        // std::cout<<"1"<<std::endl;
        A_rot.block(i * 6, i * 6, 3, 3) = rd_.cc_[i].rotm.transpose(); // rd_.ee_[i].rotm.transpose();

        // std::cout<<"1"<<std::endl;
        A_rot.block(i * 6 + 3, i * 6 + 3, 3, 3) = rd_.cc_[i].rotm.transpose();

        // std::cout<<"1"<<std::endl;
        A_const_a(i * const_num + 0, i * 6 + 2) = rd_.cc_[i].contact_plane_y_;
        A_const_a(i * const_num + 0, i * 6 + 3) = 1;

        A_const_a(i * const_num + 1, i * 6 + 2) = rd_.cc_[i].contact_plane_y_;
        A_const_a(i * const_num + 1, i * 6 + 3) = -1;

        A_const_a(i * const_num + 2, i * 6 + 2) = rd_.cc_[i].contact_plane_x_;
        A_const_a(i * const_num + 2, i * 6 + 4) = 1;

        A_const_a(i * const_num + 3, i * 6 + 2) = rd_.cc_[i].contact_plane_x_;
        A_const_a(i * const_num + 3, i * 6 + 4) = -1;
    }

    // std::cout<<"0"<<std::endl;
    Eigen::VectorXd bA = A_const_a * A_rot * (rd_.P_C - rd_.J_C_INV_T.rightCols(MODEL_DOF) * Control_Torque);

    // std::cout<<"1"<<std::endl;

    Eigen::VectorXd lbA;
    lbA.setConstant(rd_.contact_index * const_num, -1000);

    // std::cout<<"2"<<std::endl;

    qp_torque_contact_.UpdateSubjectToAx(A_const_a * A_rot * rd_.J_C_INV_T.rightCols(MODEL_DOF) * NwJw, lbA - bA, bA);

    // std::cout << "3" << std::endl;

    // qp_torque_contact_.UpdateSubjectToAx()

    // std::cout<<"6"<<std::endl;
    Eigen::VectorXd qp_result;

    qp_torque_contact_.SolveQPoases(100, qp_result);

    return NwJw * qp_result;
}

// Thread1 : running
void *TocabiController::Thread1() // Thread1, running with 2Khz.
{

    volatile int rcv_time_ = 0;

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

    std::cout << "thread1 Proceeding ... " << endl;

    // WBC::SetContactInit(rd_);

    EnableThread2(true);  // Set true for Thread2
    EnableThread3(false); // True for thread3 ...

    signalThread1 = true;
    int thread1_count = 0;

    while (!dc_.tc_shm_->shutdown)
    {

        dc_.state_locker_.consumer_wait();

        rd_.CopyKinematicsData(dc_.rd_holder_);
        // std::cout << "Command Received" << std::endl;

        // dc_.rd_holder_.CopyKinematicsData(rd_);

        dc_.state_locker_.consumer_done();

        if (dc_.stateEstimateSwitch)
        {
            std::cout << "start tc" << std::endl;
        }
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

        // WBC::ContactCalcDefault(rd_);

        if (rd_.positionControlSwitch)
        {
            rd_.positionControlSwitch = false;

            rd_.q_desired = rd_.q_;
            rd_.positionHoldSwitch = true;
            rd_.pc_mode = true;

            std::cout << " CNTRL : Position Hold!" << rd_.control_time_ << std::endl;
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

                // WBC::SetContact(rd_, 1, 1);

                // rd_.torque_desired = rd_.torque_desired + WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            }
        }
        else if (rd_.tc_run)
        {
            if (rd_.tc_.mode == 0)
            {

                static ofstream task_log;

                if (rd_.tc_init)
                {

                    std::string output_file = "/home/dyros/tocabi_log/output";

                    task_log.open(output_file.c_str(), fstream::out | fstream::app);
                    task_log << "time com_pos_x com_pos_y com_pos_z com_vel_x com_vel_y com_vel_z pel_pos_x pel_pos_y pel_pos_z pel_vel_x pel_vel_y pel_vel_z fstar_x fstar_y fstar_z lambda_x lambda_y lambda_z xtraj_x xtraj_y xtraj_z vtraj_x vtraj_y vtraj_z" << std::endl;
                    std::cout << "mode 0 init" << std::endl;
                    rd_.tc_init = false;

                    // rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                }

                // WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);

                // rd_.J_task.setZero(6, MODEL_DOF_VIRTUAL);
                // rd_.J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac().block(0, 0, 3, MODEL_DOF_VIRTUAL);
                // rd_.J_task.block(3, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

                // rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
                // rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

                // double ang2rad = 0.0174533;

                // rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

                // Eigen::VectorXd fstar;

                // if (rd_.tc_.customTaskGain)
                // {
                //     rd_.link_[COM_id].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                //     rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                // }

                // rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
                // rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

                // fstar.setZero(6);
                // fstar.segment(0, 3) = WBC::GetFstarPos(rd_.link_[COM_id], true);
                // fstar.segment(3, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                // rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));

                // VectorXd out = rd_.lambda * fstar;

                // task_log << rd_.control_time_ << " "
                //          << rd_.link_[COM_id].xpos(0) << " " << rd_.link_[COM_id].xpos(1) << " " << rd_.link_[COM_id].xpos(2) << " "
                //          << rd_.link_[COM_id].v(0) << " " << rd_.link_[COM_id].v(1) << " " << rd_.link_[COM_id].v(2) << " "
                //          << rd_.link_[Pelvis].xpos(0) << " "<< rd_.link_[Pelvis].xpos(1) << " "<< rd_.link_[Pelvis].xpos(2) << " "
                //          << rd_.link_[Pelvis].v(0) << " "<< rd_.link_[Pelvis].v(1) << " "<< rd_.link_[Pelvis].v(2) << " "
                //          << fstar(0) << " "<< fstar(1) << " "<< fstar(2) << " "
                //          << out(0) << " "<< out(1) << " "<< out(2) << " "
                //          << rd_.link_[COM_id].x_traj(0) << " "<< rd_.link_[COM_id].x_traj(1) << " "<< rd_.link_[COM_id].x_traj(2) << " "
                //          << rd_.link_[COM_id].v_traj(0) << " "<< rd_.link_[COM_id].v_traj(1) << " "<< rd_.link_[COM_id].v_traj(2) << " "
                //          << std::endl;

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
            dc_.rd_holder_.SetContact(true, true);

            dc_.rd_holder_.CalcGravCompensation();

            dc_.rd_holder_.CalcContactRedistribute(dc_.rd_holder_.torque_grav_);
            rd_.torque_desired = dc_.rd_holder_.torque_grav_ + dc_.rd_holder_.torque_contact_;

            std::cout << "torque grav " << std::endl;
            std::cout << dc_.rd_holder_.torque_grav_.transpose() << std::endl;

            std::cout << "contact redis torque" << std::endl;
            std::cout << dc_.rd_holder_.torque_contact_.transpose() << std::endl;

            std::cout << " contact force after : " << std::endl;
            std::cout << dc_.rd_holder_.getContactForce(dc_.rd_holder_.torque_grav_ + dc_.rd_holder_.torque_contact_).transpose() << std::endl;

            VectorQd torque_redis2 = contact_redis_test(dc_.rd_holder_, dc_.rd_holder_.torque_grav_);

            std::cout << " contact redis mode 2 :" << std::endl;
            std::cout << torque_redis2.transpose() << std::endl;

            std::cout << " contact force after 2: " << std::endl;
            std::cout << dc_.rd_holder_.getContactForce(dc_.rd_holder_.torque_grav_ + torque_redis2).transpose() << std::endl;

            VectorQd torque_redis3 = ContactForceRedistributionTorque(dc_.rd_holder_, dc_.rd_holder_.torque_grav_, 0.9);

            std::cout << " contact redis mode 3 :" << std::endl;
            std::cout << torque_redis3.transpose() << std::endl;

            std::cout << " contact force after 3: " << std::endl;
            std::cout << dc_.rd_holder_.getContactForce(dc_.rd_holder_.torque_grav_ + torque_redis3).transpose() << std::endl;

            // rd_.SetContact(true, true);
            // rd_.CalcGravCompensation();
            // rd_.CalcContactRedistribute(rd_.CalcGravCompensation());
            // rd_.torque_desired = rd_.torque_grav_ + rd_.torque_contact_;
            // WBC::SetContact(rd_, 1, 1);
            // rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
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
        // rd_.zmp_global_ = WBC::GetZMPpos_fromFT(rd_);

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

    // rd_.link_[Right_Foot].SetInitialWithPosition();
    // rd_.link_[Left_Foot].SetInitialWithPosition();
    // rd_.link_[Right_Hand].SetInitialWithPosition();
    // rd_.link_[Left_Hand].SetInitialWithPosition();
    // rd_.link_[Pelvis].SetInitialWithPosition();
    // rd_.link_[Upper_Body].SetInitialWithPosition();
    // rd_.link_[COM_id].SetInitialWithPosition();

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
        // rd_.link_[__id].pos_p_gain[0] = msg->pgain[0];
        // rd_.link_[__id].pos_p_gain[1] = msg->pgain[1];
        // rd_.link_[__id].pos_p_gain[2] = msg->pgain[2];

        // rd_.link_[__id].pos_d_gain[0] = msg->dgain[0];
        // rd_.link_[__id].pos_d_gain[1] = msg->dgain[1];
        // rd_.link_[__id].pos_d_gain[2] = msg->dgain[2];

        // rd_.link_[__id].rot_p_gain[0] = msg->pgain[3];
        // rd_.link_[__id].rot_p_gain[1] = msg->pgain[4];
        // rd_.link_[__id].rot_p_gain[2] = msg->pgain[5];

        // rd_.link_[__id].rot_d_gain[0] = msg->dgain[3];
        // rd_.link_[__id].rot_d_gain[1] = msg->dgain[4];
        // rd_.link_[__id].rot_d_gain[2] = msg->dgain[5];

        // std::cout << "Gain Set " << rd_.link_[__id].id << "  POS p : " << rd_.link_[__id].pos_p_gain.transpose() << "   d :" << rd_.link_[__id].pos_d_gain.transpose() << std::endl;
        // std::cout << "Gain Set " << rd_.link_[__id].id << "  ROT p : " << rd_.link_[__id].rot_p_gain.transpose() << "   d :" << rd_.link_[__id].rot_d_gain.transpose() << std::endl;
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