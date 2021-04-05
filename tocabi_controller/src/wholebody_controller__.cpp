#include "tocabi_controller/wholebody_controller.h"
#include <Eigen/QR>
#include "ros/ros.h"
#include <vector>
#include <future>
#include <numeric>

//Left Foot is first! LEFT = 0, RIGHT = 1 !
// #include "cvxgen/solver.h"

// Vars vars;
// Params params;
// Workspace work;(DataContainer &dc, RobotData &kd_);
// Settings settings;
void WholebodyController::init(RobotData &Robot)
{
    Robot.contact_calc = false;
    Robot.task_force_control = false;
    Robot.task_force_control_feedback = false;
    Robot.zmp_control = false;
    Robot.mpc_init = false;
    Robot.task_force_control = false;
    Robot.task_force_control_feedback = false;
    Robot.zmp_control = false;
    Robot.zmp_feedback_control = false;

    Robot.Grav_ref.setZero(3);
    Robot.Grav_ref(2) = -9.81;

    Robot.contact_transition_time = 3.0;

    for (int i = 0; i < MODEL_DOF; i++)
    {
        Robot.torque_limit(i) = 1000 / NM2CNT_d[i];
        torque_limit(i) = 1000 / NM2CNT_d[i];
    }

    Robot.ee_[0].contact_transition_mode = -1;
    Robot.ee_[1].contact_transition_mode = -1;
    Robot.ee_[2].contact_transition_mode = -1;
    Robot.ee_[3].contact_transition_mode = -1;

    Robot.J_g.setZero(MODEL_DOF, MODEL_DOF + 6);
    Robot.J_g.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();

}

void WholebodyController::CalcAMatrix(RobotData &Robot, MatrixXd &A_matrix)
{
    A_matrix.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(Robot.model_, Robot.q_virtual_, A_matrix, true);
}

void WholebodyController::update(RobotData &Robot)
{
    if (Robot.data_print_switch)
    {
        Robot.data_print_switch = false;

        data_out1 = std::ofstream(print_file_name.c_str());
        if (data_out1.is_open())
        {
            data_out1 << fixed;
            data_out1.width(12);
            data_out1.precision(6);
            std::cout << "Start Logging wbc 1 Data .... to " << print_file_name << std::endl;
            data_out1 << "Start Data Print" << std::endl;
        }
        else
        {
            std::cout << "Failed to open tocabi_data.txt" << std::endl;
        }

        data_out2 = std::ofstream(print_file_name2.c_str());
        if (data_out2.is_open())
        {
            data_out2 << fixed;
            data_out2.width(12);
            data_out2.precision(6);
            std::cout << "Start Logging wbc 2 Data .... to " << print_file_name2 << std::endl;
            data_out2 << "Start Data Print" << std::endl;
        }
        else
        {
            std::cout << "Failed to open tocabi_data.txt" << std::endl;
        }
    }
}

void WholebodyController::set_robot_init(RobotData &Robot)
{
    Robot.ee_[0].cp_ = Robot.link_[Left_Foot].xpos_contact;
    Robot.ee_[1].cp_ = Robot.link_[Right_Foot].xpos_contact;
    Robot.ee_[2].cp_ = Robot.link_[Left_Hand].xpos_contact;
    Robot.ee_[3].cp_ = Robot.link_[Right_Hand].xpos_contact;

    Robot.ee_[0].xpos = Robot.link_[Left_Foot].xpos;
    Robot.ee_[1].xpos = Robot.link_[Right_Foot].xpos;
    Robot.ee_[2].xpos = Robot.link_[Left_Hand].xpos;
    Robot.ee_[3].xpos = Robot.link_[Right_Hand].xpos;

    Robot.ee_[0].rotm = Robot.link_[Left_Foot].Rotm;
    Robot.ee_[1].rotm = Robot.link_[Right_Foot].Rotm;
    Robot.ee_[2].rotm = Robot.link_[Left_Hand].Rotm;
    Robot.ee_[3].rotm = Robot.link_[Right_Hand].Rotm;

    Robot.ee_[0].sensor_xpos = Robot.link_[Left_Foot].xpos_sensor;
    Robot.ee_[1].sensor_xpos = Robot.link_[Right_Foot].xpos_sensor;
    Robot.ee_[2].sensor_xpos = Robot.link_[Left_Hand].xpos_sensor;
    Robot.ee_[3].sensor_xpos = Robot.link_[Right_Hand].xpos_sensor;

    Robot.ee_[0].cs_x_length = 0.07;
    Robot.ee_[0].cs_y_length = 0.025;
    Robot.ee_[1].cs_x_length = 0.07;
    Robot.ee_[1].cs_y_length = 0.025;

    Robot.ee_[2].cs_x_length = 0.013;
    Robot.ee_[2].cs_y_length = 0.013;
    Robot.ee_[3].cs_x_length = 0.013;
    Robot.ee_[3].cs_y_length = 0.013;

    Robot.ee_[0].minimum_press_force = 40;
    Robot.ee_[1].minimum_press_force = 40;
    Robot.ee_[2].minimum_press_force = 40;
    Robot.ee_[3].minimum_press_force = 40;

    Robot.friction_ratio = 0.20;

    Robot.ee_[0].friction_ratio = Robot.friction_ratio;
    Robot.ee_[1].friction_ratio = Robot.friction_ratio;

    Robot.ee_[2].friction_ratio = Robot.friction_ratio;
    Robot.ee_[3].friction_ratio = Robot.friction_ratio;

    Robot.ee_[0].friction_ratio_z = Robot.friction_ratio;
    Robot.ee_[1].friction_ratio_z = Robot.friction_ratio;
    Robot.ee_[2].friction_ratio_z = Robot.friction_ratio;
    Robot.ee_[3].friction_ratio_z = Robot.friction_ratio;

    Robot.Lambda_c = (Robot.J_C * Robot.A_matrix_inverse * (Robot.J_C.transpose())).inverse();
    Robot.J_C_INV_T = Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse;
    Robot.N_C.setZero(MODEL_DOF + 6, MODEL_DOF + 6);
    Robot.I37.setIdentity(MODEL_DOF + 6, MODEL_DOF + 6);
    Robot.N_C = Robot.I37 - Robot.J_C.transpose() * Robot.J_C_INV_T;
    Robot.Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Robot.Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Robot.Slc_k_T = Robot.Slc_k.transpose();
    Robot.W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; //2 types for w matrix

    Robot.W_inv = DyrosMath::pinv_QR(Robot.W, Robot.qr_V2);
}
void WholebodyController::calc_winv(RobotData &Robot)
{
    Robot.W = Robot.A_matrix_inverse.bottomRows(MODEL_DOF) * Robot.N_C.rightCols(MODEL_DOF); //2 types for w matrix
    Robot.winv_ret = std::async(std::launch::async, &DyrosMath::pinv_QR_pair, std::ref(Robot.W));
}

void WholebodyController::get_winv(RobotData &Robot)
{
    if (Robot.winv_ret.valid())
    {
        //std::cout<<"valid, returning ... ";
        std::pair<MatrixXd, MatrixXd> ans;
        ans = Robot.winv_ret.get();
        Robot.W_inv = ans.first;
        Robot.qr_V2 = ans.second;
    }
    else
    {
        std::cout << "Not valid";
    }
}

void WholebodyController::set_robot_init_multithread(RobotData &Robot)
{
    Robot.Lambda_c = (Robot.J_C * Robot.A_matrix_inverse * (Robot.J_C.transpose())).inverse();
    Robot.J_C_INV_T = Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse;
    Robot.N_C = MatrixVVd::Identity() - Robot.J_C.transpose() * Robot.J_C_INV_T;

    calc_winv(Robot);

    std::chrono::steady_clock::time_point t[2];
    int tcnt = 0;
    //t[tcnt++] = std::chrono::steady_clock::now();

    Robot.ee_[0].cp_ = Robot.link_[Left_Foot].xpos_contact;
    Robot.ee_[1].cp_ = Robot.link_[Right_Foot].xpos_contact;
    Robot.ee_[2].cp_ = Robot.link_[Left_Hand].xpos_contact;
    Robot.ee_[3].cp_ = Robot.link_[Right_Hand].xpos_contact;

    Robot.ee_[0].xpos = Robot.link_[Left_Foot].xpos;
    Robot.ee_[1].xpos = Robot.link_[Right_Foot].xpos;
    Robot.ee_[2].xpos = Robot.link_[Left_Hand].xpos;
    Robot.ee_[3].xpos = Robot.link_[Right_Hand].xpos;

    Robot.ee_[0].rotm = Robot.link_[Left_Foot].Rotm;
    Robot.ee_[1].rotm = Robot.link_[Right_Foot].Rotm;
    Robot.ee_[2].rotm = Robot.link_[Left_Hand].Rotm;
    Robot.ee_[3].rotm = Robot.link_[Right_Hand].Rotm;

    Robot.ee_[0].sensor_xpos = Robot.link_[Left_Foot].xpos_sensor;
    Robot.ee_[1].sensor_xpos = Robot.link_[Right_Foot].xpos_sensor;
    Robot.ee_[2].sensor_xpos = Robot.link_[Left_Hand].xpos_sensor;
    Robot.ee_[3].sensor_xpos = Robot.link_[Right_Hand].xpos_sensor;

    Robot.ee_[0].cs_x_length = 0.10;
    Robot.ee_[0].cs_y_length = 0.024;
    Robot.ee_[1].cs_x_length = 0.10;
    Robot.ee_[1].cs_y_length = 0.024;
    Robot.ee_[2].cs_x_length = 0.013;
    Robot.ee_[2].cs_y_length = 0.013;
    Robot.ee_[3].cs_x_length = 0.013;
    Robot.ee_[3].cs_y_length = 0.013;

    Robot.ee_[0].minimum_press_force = 40;
    Robot.ee_[1].minimum_press_force = 40;
    Robot.ee_[2].minimum_press_force = 40;
    Robot.ee_[3].minimum_press_force = 40;

    Robot.friction_ratio = 0.18;

    Robot.ee_[0].friction_ratio = Robot.friction_ratio;
    Robot.ee_[1].friction_ratio = Robot.friction_ratio;
    Robot.ee_[2].friction_ratio = Robot.friction_ratio;
    Robot.ee_[3].friction_ratio = Robot.friction_ratio;

    Robot.ee_[0].friction_ratio_z = Robot.friction_ratio;
    Robot.ee_[1].friction_ratio_z = Robot.friction_ratio;
    Robot.ee_[2].friction_ratio_z = Robot.friction_ratio;
    Robot.ee_[3].friction_ratio_z = Robot.friction_ratio;
    //t[tcnt++] = std::chrono::steady_clock::now();

    //t[tcnt++] = std::chrono::steady_clock::now();
    //t[tcnt++] = std::chrono::steady_clock::now();
    Robot.I37.setIdentity(MODEL_DOF + 6, MODEL_DOF + 6);
    //t[tcnt++] = std::chrono::steady_clock::now();
    Robot.Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Robot.Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Robot.Slc_k_T = Robot.Slc_k.transpose();
    //t[tcnt++] = std::chrono::steady_clock::now();
    //Robot.W = Robot.A_matrix_inverse.bottomRows(MODEL_DOF) * Robot.N_C.rightCols(MODEL_DOF); // * Robot.Slc_k_T; //2 types for w matrix
    //t[tcnt++] = std::chrono::steady_clock::now();
    //Robot.W_inv = DyrosMath::pinv_QR(Robot.W, Robot.qr_V2);
    get_winv(Robot);
}

void WholebodyController::set_contact_multithread(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand, bool right_hand)
{
    Robot.ee_[0].contact = left_foot;
    Robot.ee_[1].contact = right_foot;
    Robot.ee_[2].contact = left_hand;
    Robot.ee_[3].contact = right_hand;

    Robot.contact_index = 0;
    if (left_foot)
    {
        Robot.contact_part[Robot.contact_index] = Left_Foot;
        Robot.ee_idx[Robot.contact_index] = 0;
        Robot.contact_index++;
    }
    if (right_foot)
    {
        Robot.contact_part[Robot.contact_index] = Right_Foot;
        Robot.ee_idx[Robot.contact_index] = 1;
        Robot.contact_index++;
    }
    if (left_hand)
    {
        Robot.contact_part[Robot.contact_index] = Left_Hand;
        Robot.ee_idx[Robot.contact_index] = 2;
        Robot.contact_index++;
    }
    if (right_hand)
    {
        Robot.contact_part[Robot.contact_index] = Right_Hand;
        Robot.ee_idx[Robot.contact_index] = 3;
        Robot.contact_index++;
    }
    //contact_set(contact_index, contact_part);

    Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);

    Robot.link_[Left_Foot].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Left_Foot].contact_point);
    Robot.link_[Right_Foot].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Right_Foot].contact_point);
    Robot.link_[Left_Hand].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Left_Hand].contact_point);
    Robot.link_[Right_Hand].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Right_Hand].contact_point);

    for (int i = 0; i < Robot.contact_index; i++)
    {
        Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.link_[Robot.contact_part[i]].Jac_Contact;
    }

    set_robot_init_multithread(Robot);

    Robot.contact_force_predict.setZero();
    Robot.contact_calc = true;
}

void WholebodyController::set_contact(RobotData &Robot)
{

    Robot.contact_index = 0;
    if (Robot.ee_[0].contact)
    {
        Robot.contact_part[Robot.contact_index] = Left_Foot;
        Robot.ee_idx[Robot.contact_index] = 0;
        Robot.contact_index++;
    }
    if (Robot.ee_[1].contact)
    {
        Robot.contact_part[Robot.contact_index] = Right_Foot;
        Robot.ee_idx[Robot.contact_index] = 1;
        Robot.contact_index++;
    }
    /* if (right_hand)
    {
        contact_part[contact_index] = Right_Hand;
        contact_index++;
    }
    if (left_hand)
    {
        contact_part[contact_index] = Left_Hand;
        contact_index++;
    }*/
    //contact_set(contact_index, contact_part);

    Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);

    Robot.link_[Left_Foot].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Left_Foot].contact_point);
    Robot.link_[Right_Foot].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Right_Foot].contact_point);
    Robot.link_[Left_Hand].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Left_Hand].contact_point);
    Robot.link_[Right_Hand].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Right_Hand].contact_point);

    for (int i = 0; i < Robot.contact_index; i++)
    {
        Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.link_[Robot.contact_part[i]].Jac_Contact;
    }
    set_robot_init(Robot);

    Robot.contact_force_predict.setZero();
    Robot.contact_calc = true;
}

void WholebodyController::set_contact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand, bool right_hand)
{
    Robot.ee_[0].contact = left_foot;
    Robot.ee_[1].contact = right_foot;
    Robot.ee_[2].contact = left_hand;
    Robot.ee_[3].contact = right_hand;

    Robot.contact_index = 0;
    if (left_foot)
    {
        Robot.contact_part[Robot.contact_index] = Left_Foot;
        Robot.ee_idx[Robot.contact_index] = 0;
        Robot.contact_index++;
    }
    if (right_foot)
    {
        Robot.contact_part[Robot.contact_index] = Right_Foot;
        Robot.ee_idx[Robot.contact_index] = 1;
        Robot.contact_index++;
    }
    if (left_hand)
    {
        Robot.contact_part[Robot.contact_index] = Left_Hand;
        Robot.ee_idx[Robot.contact_index] = 2;
        Robot.contact_index++;
    }
    if (right_hand)
    {
        Robot.contact_part[Robot.contact_index] = Right_Hand;
        Robot.ee_idx[Robot.contact_index] = 3;
        Robot.contact_index++;
    }

    Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);

    Robot.link_[Left_Foot].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Left_Foot].contact_point);
    Robot.link_[Right_Foot].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Right_Foot].contact_point);
    Robot.link_[Left_Hand].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Left_Hand].contact_point);
    Robot.link_[Right_Hand].Set_Contact(Robot.q_virtual_, Robot.q_dot_virtual_, Robot.link_[Right_Hand].contact_point);

    for (int i = 0; i < Robot.contact_index; i++)
    {
        Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.link_[Robot.contact_part[i]].Jac_Contact;
    }

    set_robot_init(Robot);

    Robot.contact_force_predict.setZero();
    Robot.contact_calc = true;

    // std::cout << "setcontact : ";
    // for (int i = 0; i < tcnt; i++)
    // {
    //     std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t[i + 1] - t[i]).count() << "\t";
    // }
    // std::cout << "total : " << std::chrono::duration_cast<std::chrono::microseconds>(t[tcnt] - t[0]).count() << std::endl;
}

Matrix2d matpower(Matrix2d mat, int i)
{
    Matrix2d m;
    m.setIdentity();
    if (i == 0)
    {
        return m;
    }
    else
    {
        for (int j = 0; j < i; j++)
        {
            m = m * mat;
        }
    }
    return m;
}

Vector2d WholebodyController::getcpref(RobotData &Robot, double task_time, double future_time)
{
    double time_segment = 1.0;
    double step_length = 0.1;
    double w_ = sqrt(9.81 / 0.081);
    double b_ = exp(w_ * (time_segment));

    Vector2d CP_ref[9];
    CP_ref[0] << -0.04, 0;
    CP_ref[1] << -0.04, 0.096256;
    CP_ref[2] << 0.06, -0.096256;
    CP_ref[3] << 0.16, 0.096256;
    CP_ref[4] << 0.26, -0.096256;
    CP_ref[5] << 0.36, 0.096256;
    CP_ref[6] << 0.46, -0.096256;
    CP_ref[7] << 0.46, 0.0;
    CP_ref[8] << 0.46, 0.0;

    Vector2d ZMP_ref[8];
    for (int i = 0; i < 8; i++)
    {
        ZMP_ref[i] = 1 / (1 - b_) * CP_ref[i + 1] - b_ / (1 - b_) * CP_ref[i];
    }
    //non gui mode?

    //(int)time
    double left_time = 1 - task_time + (int)task_time;

    double t_see;

    t_see = task_time + future_time;
    Vector2d CP_t;
    CP_t = exp(w_ * (t_see - (int)t_see)) * CP_ref[(int)t_see] + (1.0 - exp(w_ * (t_see - (int)t_see))) * ZMP_ref[(int)t_see];

    if ((task_time - (int)task_time) + future_time < 1)
    {
        double b = exp(w_ * left_time);
        Vector2d zmp_ = 1 / (1 - b) * CP_ref[(int)task_time + 1] - b / (1 - b) * Robot.com_.CP;

        CP_t = exp(w_ * future_time) * Robot.com_.CP + (1.0 - exp(w_ * future_time)) * zmp_;
    }

    return CP_t;
}

// Vector2d WholebodyController::getcptraj(double time, Vector2d zmp) //task_time
// {
//   double time_segment = 1.0;
//   double step_length = 0.1;

//   int n_sample = 30;
//   double t_sample = 0.005; //milliseconds

//   double task_time = control_time_ - tc_.taskcommand_.command_time;

//   double w_ = sqrt(9.81 / 0.81);

//   Matrix2d A, B;
//   A << exp(w_ * t_sample), 0, 0, exp(w_ * t_sample);
//   B << 1 - exp(w_ * t_sample), 0, 0, 1 - exp(w_ * t_sample);

//   MatrixXd F_xi, F_p, F_p_temp;
//   F_xi.setZero(n_sample * 2, 2);
//   F_p.setZero(n_sample * 2, n_sample * 2);
//   F_p_temp.setZero(n_sample * 2, 2);

//   for (int i = 0; i < n_sample; i++)
//   {
//     F_xi.block(i * 2, 0, 2, 2) = matpower(A, i + 1);
//     for (int j = i; j < n_sample; j++)
//     {
//       F_p.block(j * 2, i * 2, 2, 2) = matpower(A, j - i) * B;
//     }
//   }

//   MatrixXd THETA;
//   THETA.setIdentity(n_sample * 2, n_sample * 2);

//   Matrix2d I2;
//   I2.setIdentity();

//   for (int i = 0; i < n_sample - 1; i++)
//   {
//     THETA.block(i * 2 + 2, i * 2, 2, 2) = -I2;
//   }

//   MatrixXd e1;
//   e1.setZero(2 * n_sample, 2);
//   e1.block(0, 0, 2, 2) = I2;

//   double q_par = 1.0;
//   double r_par = 0.4;

//   MatrixXd H, I_nsample;
//   I_nsample.setIdentity(2 * n_sample, 2 * n_sample);
//   MatrixXd Q, R;
//   Q = q_par * I_nsample;
//   R = r_par * I_nsample;

//   H = THETA.transpose() * R * THETA + F_p.transpose() * Q * F_p;

//   VectorXd g;
//   VectorXd cp_ref_t;
//   cp_ref_t.setZero(n_sample * 2);
//   for (int i = 0; i < n_sample; i++)
//   {
//     cp_ref_t.segment(i * 2, 2) = getcpref(time, i * t_sample);
//   }
//   std::cout << " pk-1" << std::endl;
//   std::cout << p_k_1 << std::endl;
//   std::cout << " cp_ref_t : " << std::endl;
//   std::cout << cp_ref_t << std::endl;
//   std::cout << " com_CP : " << com_.CP << std::endl;
//   std::cout << " g1 : " << std::endl
//             << F_p.transpose() * Q * (F_xi * com_.CP.segment(0, 2) - cp_ref_t) << std::endl;
//   std::cout << " g2 : " << std::endl
//             << THETA.transpose() * R * e1 * p_k_1 << std::endl;

//   g = F_p.transpose() * Q * (F_xi * com_.CP.segment(0, 2) - cp_ref_t) - THETA.transpose() * R * e1 * p_k_1;

//   VectorXd r(n_sample * 2);
//   Vector2d sf[8];
//   sf[0] << -0.04, 0;
//   sf[1] << -0.04, 0.1024;
//   sf[2] << 0.06, -0.1024;
//   sf[3] << 0.16, 0.1024;
//   sf[4] << 0.26, -0.1024;
//   sf[5] << 0.36, 0.1024;
//   sf[6] << 0.46, -0.1024;
//   sf[7] << 0.46, 0.0;

//   for (int i = 0; i < n_sample; i++)
//   {
//     r.segment(i * 2, 2) = sf[(int)(time + t_sample * i)];
//   }

//   VectorXd lb, ub, f;
//   f.setZero(n_sample * 2);

//   for (int i = 0; i < n_sample * 2; i++)
//   {
//     f(i) = 0.1;
//   }
//   lb = lb.setZero(n_sample * 2);
//   lb = r - f;
//   ub = ub.setZero(n_sample * 2);
//   ub = r + f;

//   Vector2d res_2;
//   res_2.setZero();
//   VectorXd res;
//   res.setZero(n_sample * 2);
//   if (mpc_init)
//   {
//     QP_mpc.InitializeProblemSize(2 * n_sample, 1);
//     QP_mpc.UpdateMinProblem(H, g);
//     QP_mpc.PrintMinProb();
//     QP_mpc.PrintSubjectTox();
//     QP_mpc.SolveQPoases(1000);
//     set_defaults();
//     setup_indexing();
//     settings.verbose = 0;

//     for (int i = 0; i < 2 * n_sample; i++)
//       for (int j = 0; j < 2 * n_sample; j++)
//         params.H[i + j * 2 * n_sample] = H(i, j);

//     for (int i = 0; i < 2 * n_sample; i++)
//     {
//       params.g[i] = g(i);
//       params.r[i] = r(i);
//     }

//     params.f[0] = 0.025;
//     int num_inters = solve();

//     std::cout << "ANSWER IS " << vars.P[0] << vars.P[1] << std::endl;

//     res_2(0) = vars.P[0];
//     res_2(1) = vars.P[1];

//     p_k_1 = res_2;
//   }

//   if (mpc_init == false)
//   {
//     p_k_1 = zmp;
//   }
//   mpc_init = true;

//   return res_2;
// }

VectorQd WholebodyController::task_control_torque(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_, int mode)
{
    if (mode == 0)
    {
        return task_control_torque_with_gravity(Robot, J_task, f_star_);
    }
    else if (mode == 1)
    {
        return task_control_torque_QP(Robot, J_task, f_star_);
    }
    else if (mode == 2)
    {
        Robot.qp2nd = false;
        return task_control_torque_QP2(Robot, J_task, f_star_);
    }
    else if (mode == 3)
    {
        Robot.qp2nd = true;
        return task_control_torque_QP2(Robot, J_task, f_star_);
    }
    else if (mode == 4)
    {
        Robot.qp2nd = true;
        return task_control_torque_QP3(Robot, J_task, f_star_);
    }
}

VectorQd WholebodyController::task_control_torque_QP(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    //QP1 : grav compensation performed in case of error from qp
    VectorXd f_star_qp_;
    VectorQd task_torque;
    //VectorQd gravity_torque = gravity_compensation_torque(Robot, dc.fixedgravity);
    double friction_ratio = 0.3;
    //qptest
    double foot_x_length = 0.12;
    double foot_y_length = 0.04;

    Robot.task_dof = J_task.rows();

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    MatrixXd J_com = Robot.link_[COM_id].Jac_COM_p;
    MatrixXd lambda_com_inv = J_com * Robot.A_matrix_inverse * Robot.N_C * J_com.transpose();
    MatrixXd J_com_inv_T = lambda_com_inv.inverse() * J_com * Robot.A_matrix_inverse * Robot.N_C;

    Vector3d fstar_com;
    fstar_com = lambda_com_inv * J_com_inv_T * Robot.J_task_T * Robot.lambda * f_star_;

    double zmp_com_e_x, zmp_com_e_y;
    zmp_com_e_x = Robot.com_.pos(0) - Robot.com_.pos(2) * fstar_com(0) / 9.81;
    zmp_com_e_y = Robot.com_.pos(1) - Robot.com_.pos(2) * fstar_com(1) / 9.81;

    double dist_l, dist_r;
    dist_l = sqrt((Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y));
    dist_r = sqrt((Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y));

    double ratio_r, ratio_l;
    ratio_r = dist_l / (dist_l + dist_r);
    ratio_l = dist_r / (dist_l + dist_r);

    static int task_dof, contact_dof;
    int constraint_per_contact = 12;
    bool qpt_info = false;

    if ((task_dof != Robot.task_dof) || (contact_dof != 6 * Robot.contact_index))
    {
        task_dof = Robot.task_dof;
        contact_dof = 6 * Robot.contact_index;
        std::cout << "############################" << std::endl
                  << "QP initialize ! " << std::endl
                  << "Task Dof    = " << Robot.task_dof << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        std::cout << std::endl
                  << "############################" << std::endl;

        qpt_info = true;
    }

    int variable_size = MODEL_DOF + contact_dof + task_dof;
    int constraint_size = task_dof + contact_dof + constraint_per_contact * Robot.contact_index;

    //QP initialize!
    QP_torque.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::
    W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = W;                              // + 0.01 * MatrixXd::Identity(MODEL_DOF,MODEL_DOF);
    g.segment(0, MODEL_DOF) = -Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;

    //fstar regulation ::
    H.block(MODEL_DOF + contact_dof, MODEL_DOF + contact_dof, task_dof, task_dof) = 100 * MatrixXd::Identity(task_dof, task_dof);
    g.segment(MODEL_DOF + contact_dof, task_dof) = -100 * f_star_;

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Fsl(6 * i + 0, 6 * i + 0) = 0.0001;
        Fsl(6 * i + 1, 6 * i + 1) = 0.0001;
        //Fsl(6 * i + 2, 6 * i + 2) = 1E-6;
        Fsl(6 * i + 3, 6 * i + 3) = 0.001;
        Fsl(6 * i + 4, 6 * i + 4) = 0.001;
        Fsl(6 * i + 5, 6 * i + 5) = 0.00001;
    }

    double rr = DyrosMath::minmax_cut(ratio_r / ratio_l * 10, 1, 10);
    double rl = DyrosMath::minmax_cut(ratio_l / ratio_r * 10, 1, 10);
    //std::cout << "left : " << rr << "\t right : " << rl << std::endl;

    if (Robot.qp2nd)
    {
        Fsl(0, 0) = 0.0001 * rr;
        Fsl(1, 1) = 0.0001 * rr;

        Fsl(3, 3) = 0.001 * rr;
        Fsl(4, 4) = 0.001 * rr;

        Fsl(6, 6) = 0.0001 * rl;
        Fsl(7, 7) = 0.0001 * rl;

        Fsl(9, 9) = 0.001 * rl;
        Fsl(10, 10) = 0.001 * rl;
    }

    /*

    Vector3d P_right = Robot.link_[Right_Foot].xpos - Robot.com_.pos;
    Vector3d P_left = Robot.link_[Left_Foot].xpos - Robot.com_.pos;

    double pr = sqrt(P_right(0) * P_right(0) + P_right(1) * P_right(1));
    double pl = sqrt(P_left(0) * P_left(0) + P_left(1) * P_left(1));

    double prr = pr / (pl + pr);
    double pll = pl / (pl + pr);

    prr = DyrosMath::minmax_cut(prr, 0.1, 0.9);
    pll = DyrosMath::minmax_cut(pll, 0.1, 0.9);

    bool hr_v = false;
    if (hr_v)
    {
        Fsl(3, 3) = 0.001 * pll;
        Fsl(4, 4) = 0.001 * pll;

        Fsl(9, 9) = 0.001 * prr;
        Fsl(10, 10) = 0.001 * prr;
    }
    else
    {
        Fsl(3, 3) = 0.001;
        Fsl(4, 4) = 0.001;

        Fsl(9, 9) = 0.001;
        Fsl(10, 10) = 0.001;
    } */

    H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl.transpose() * Fsl;

    //Rigid Body Dynamcis Equality Constraint
    A.block(0, 0, task_dof, MODEL_DOF) = Robot.J_task_inv_T * Robot.Slc_k_T;
    A.block(0, MODEL_DOF + contact_dof, task_dof, task_dof) = -Robot.lambda;
    lbA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;
    ubA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;

    //Contact Force Equality constraint
    A.block(task_dof, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(task_dof, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //std::cout << "calc done!" << std::endl;
    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 3 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 4 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }
    }

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(task_dof + contact_dof + i) = 0.0;
        ubA(task_dof + contact_dof + i) = 1000.0;
    }

    //std::cout << "calc done!" << std::endl;
    //Torque bound setting
    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -300;
        ub(i) = 300;
    }
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -1000;
        ub(MODEL_DOF + i) = 1000;
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        ub(MODEL_DOF + 6 * i + 2) = -20.0;
        ub(MODEL_DOF + 6 * i + 5) = 0.05;
        lb(MODEL_DOF + 6 * i + 5) = -0.05;
    }

    for (int i = 0; i < task_dof; i++)
    {
        lb(MODEL_DOF + contact_dof + i) = -1000;
        ub(MODEL_DOF + contact_dof + i) = 1000;
    }

    QP_torque.EnableEqualityCondition(0.0001);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    VectorXd qpres;
    if (QP_torque.SolveQPoases(100, qpres) == 0)
    {
        std::cout << "qp error" << std::endl;
        task_torque = gravity_compensation_torque(Robot);
    }
    else
    {
        task_torque = qpres.segment(0, MODEL_DOF);
    }

    VectorXd fc = qpres.segment(MODEL_DOF, contact_dof);

    qpt_info = false;
    if (qpt_info)
    {
        //std::cout << "calc done!! first solution! " << toc - tic << std::endl;
        //std::cout << " torque result : " << std::endl;
        //std::cout << task_torque << std::endl;
        std::cout << "fc result " << std::endl;
        std::cout << qpres.segment(MODEL_DOF, contact_dof) << std::endl;
        //std::cout << "fstar result : " << std::endl;
        //std::cout << qpres.segment(MODEL_DOF + contact_dof, task_dof) << std::endl;
        //std::cout << "fstar desired : " << std::endl;
        //std::cout << f_star_ << std::endl;
    }

    qpt_info = false;
    if (qpt_info)
    {
        //std::cout << "calc done! second solution! " << toc - tic << std::endl;
        //std::cout << " torque result : " << std::endl;
        //std::cout << task_torque << std::endl;
        std::cout << "fc result " << std::endl;
        std::cout << qpres.segment(MODEL_DOF, contact_dof) << std::endl;
        //std::cout << "fstar result : " << std::endl;
        //std::cout << qpres.segment(MODEL_DOF + contact_dof, task_dof) << std::endl;
        //std::cout << "fstar desired : " << std::endl;
        //std::cout << f_star_ << std::endl;
        std::cout << "##########################" << std::endl;
    }

    MatrixXd W_fr;

    if (Robot.ee_[0].contact && Robot.ee_[1].contact)
    {
        W_fr.setZero(6, 12);
        W_fr.block(0, 0, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 3, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 0, 3, 3) = Robot.link_[Left_Foot].Rotm * DyrosMath::skm(Robot.link_[Left_Foot].xpos_contact - Robot.com_.pos);

        W_fr.block(0, 6, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 9, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 6, 3, 3) = Robot.link_[Right_Foot].Rotm * DyrosMath::skm(Robot.link_[Right_Foot].xpos_contact - Robot.com_.pos);
    }
    else if (Robot.ee_[0].contact)
    {
        W_fr.setZero(6, 6);
        W_fr.block(0, 0, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 3, 3, 3) = Robot.link_[Left_Foot].Rotm;
        W_fr.block(3, 0, 3, 3) = Robot.link_[Left_Foot].Rotm * DyrosMath::skm(Robot.link_[Left_Foot].xpos_contact - Robot.com_.pos);
    }
    else if (Robot.ee_[1].contact)
    {
        W_fr.setZero(6, 6);
        W_fr.block(0, 0, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 3, 3, 3) = Robot.link_[Right_Foot].Rotm;
        W_fr.block(3, 0, 3, 3) = Robot.link_[Right_Foot].Rotm * DyrosMath::skm(Robot.link_[Right_Foot].xpos_contact - Robot.com_.pos);
    }

    VectorXd fr = W_fr * fc;

    Vector3d r_zmp = GetZMPpos(Robot, fc);
    qpt_info = false;
    if (qpt_info)
    {
        std::cout << "##########################" << std::endl;
        std::cout << "contact info ! " << std::endl
                  //<< "fc result : " << std::endl
                  //<< fc << std::endl
                  //<< "fc local : " << std::endl
                  //<< Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(0, 3) << std::endl
                  //<< Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(3, 3) << std::endl
                  //<< Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(6, 3) << std::endl
                  //<< Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(9, 3) << std::endl
                  //<< "qp fz : " << fc(2) + fc(8) << std::endl
                  //<< " fstar*lambda = " << (Robot.lambda * qpres.segment(MODEL_DOF + contact_dof, task_dof))(2) << std::endl
                  //<< " qp + lambda f* : " << (Robot.lambda * qpres.segment(MODEL_DOF + contact_dof, task_dof))(2) + fc(2) + fc(8) << std::endl
                  //<< " fstar desired : " << std::endl
                  //<< f_star_ << std::endl
                  //<< "fstar qp : " << std::endl
                  //<< qpres.segment(MODEL_DOF + contact_dof, task_dof) << std::endl
                  << "ft fz : " << Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8) << std::endl
                  << "resultant fz : " << std::endl
                  << fr << std::endl
                  << "resultant zmp x : " << r_zmp(0) << "\t y : " << r_zmp(1) << std::endl
                  << "estimate zmp x : " << zmp_com_e_x << "\t y : " << zmp_com_e_y << std::endl
                  << "com pos x : " << Robot.com_.pos(0) << "\t y : " << Robot.com_.pos(1) << std::endl
                  << "zmp result : " << std::endl
                  << "Left foot  x :" << fc(4) / fc(2) << "\t y :" << fc(3) / fc(2) << std::endl
                  << "Right foot x :" << fc(10) / fc(8) << "\t y :" << fc(9) / fc(8) << std::endl

                  << "fstar com induced : " << std::endl
                  << fstar_com << std::endl
                  << "fstar from torque : " << std::endl
                  << J_com * Robot.A_matrix_inverse * Robot.N_C * (Robot.Slc_k_T * task_torque - Robot.G) << std::endl;
    }

    fc.segment(0, 3) = Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(0, 3);
    fc.segment(3, 3) = Robot.link_[Left_Foot].Rotm.transpose() * fc.segment(3, 3);
    fc.segment(6, 3) = Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(6, 3);
    fc.segment(9, 3) = Robot.link_[Right_Foot].Rotm.transpose() * fc.segment(9, 3);

    double ft_zmp_l = Robot.ContactForce_FT(3) / Robot.ContactForce_FT(2);
    double ft_zmp_r = Robot.ContactForce_FT(9) / Robot.ContactForce_FT(8);

    if (abs(ft_zmp_l) > 0.04)
    {
        //std::cout << "lf zmp over limit : " << ft_zmp_l << std::endl;
    }
    if (abs(ft_zmp_r) > 0.04)
    {
        //std::cout << "rf zmp over limit : " << ft_zmp_r << std::endl;
    }

    Robot.ContactForce = fc;

    qpt_info = false;
    if (qpt_info)
    {
        std::cout << "##########################" << std::endl
                  << "fc result : " << std::endl
                  << fc << std::endl
                  << "zmp result : " << std::endl
                  << "Left foot  x :" << fc(4) / fc(2) << "\t y :" << fc(3) / fc(2) << std::endl
                  << "Right foot x :" << fc(10) / fc(8) << "\t y :" << fc(9) / fc(8) << std::endl
                  << "L/(L+R) : " << fc(2) / (fc(2) + fc(8)) << std::endl
                  //<< "pl/(pl+pr) : " << pr / (pl + pr) << std::endl

                  << "zmp from ft : " << std::endl
                  << "Left foot  x :" << Robot.ContactForce_FT(4) / Robot.ContactForce_FT(2) << "\t y :" << Robot.ContactForce_FT(3) / Robot.ContactForce_FT(2) << std::endl
                  << "Right foot x :" << Robot.ContactForce_FT(10) / Robot.ContactForce_FT(8) << "\t y :" << Robot.ContactForce_FT(9) / Robot.ContactForce_FT(8) << std::endl

                  << "com acc induce : " << std::endl;
        MatrixXd Jcom = Robot.link_[COM_id].Jac_COM_p;
        MatrixXd QQ = Jcom * Robot.A_matrix_inverse * Jcom.transpose() * (Jcom * Robot.A_matrix_inverse * Robot.N_C * Jcom.transpose()).inverse() * Jcom * Robot.A_matrix_inverse * Robot.N_C;

        std::cout << QQ * Robot.Slc_k_T * task_torque << std::endl;
        std::cout << "com acc - grav induce " << std::endl
                  << QQ * (Robot.Slc_k_T * task_torque - Robot.G) << std::endl;
    }
    return task_torque;
}

VectorQd WholebodyController::task_control_torque_QP2(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    VectorQd task_torque;
    VectorXd f_star_qp_;

    Robot.task_dof = J_task.rows();

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_inv = (DyrosMath::pinv_glsSVD(J_task.transpose())).transpose();
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    MatrixXd J_com = Robot.link_[COM_id].Jac_COM_p;
    MatrixXd lambda_com_inv = J_com * Robot.A_matrix_inverse * Robot.N_C * J_com.transpose();
    MatrixXd J_com_inv_T = lambda_com_inv.inverse() * J_com * Robot.A_matrix_inverse * Robot.N_C;

    Vector3d fstar_com;
    fstar_com = lambda_com_inv * J_com_inv_T * Robot.J_task_T * Robot.lambda * f_star_;

    double zmp_com_e_x, zmp_com_e_y;
    zmp_com_e_x = Robot.com_.pos(0) - Robot.com_.pos(2) * Robot.link_[COM_id].a_traj(0) / 9.81;
    zmp_com_e_y = Robot.com_.pos(1) - Robot.com_.pos(2) * Robot.link_[COM_id].a_traj(1) / 9.81;

    double dist_l, dist_r;
    dist_l = sqrt((Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y));
    dist_r = sqrt((Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y));

    double ratio_r, ratio_l;
    ratio_r = dist_l / (dist_l + dist_r);
    ratio_l = dist_r / (dist_l + dist_r);

    static int task_dof, contact_dof;
    int constraint_per_contact = 10;
    bool qpt_info = false;

    if ((task_dof != Robot.task_dof) || (contact_dof != 6 * Robot.contact_index))
    {
        task_dof = Robot.task_dof;
        contact_dof = 6 * Robot.contact_index;
        std::cout << "############################" << std::endl
                  << "QP initialize ! " << std::endl
                  << "Task Dof    = " << Robot.task_dof << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        std::cout << std::endl
                  << "############################" << std::endl;

        qpt_info = true;
    }

    int variable_size = MODEL_DOF + contact_dof + task_dof;
    int constraint_size = task_dof + contact_dof + constraint_per_contact * Robot.contact_index;

    //QP initialize!
    QP_torque.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::

    MatrixXd N_task;
    N_task.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    N_task = MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) - Robot.J_task_inv * Robot.J_task;

    double ea_weight = 1.0;
    //W = Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * N_task.transpose() * Robot.A_matrix * N_task * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    //g.segment(0, MODEL_DOF) = -ea_weight * Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * N_task.transpose() * Robot.A_matrix * N_task * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    g.segment(0, MODEL_DOF) = -ea_weight * Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = ea_weight * W; // + 0.01 * MatrixXd::Identity(MODEL_DOF,MODEL_DOF);

    //fstar regulation ::
    double fstar_weight = 50.0;
    H.block(MODEL_DOF + contact_dof, MODEL_DOF + contact_dof, task_dof, task_dof) = fstar_weight * MatrixXd::Identity(task_dof, task_dof);
    g.segment(MODEL_DOF + contact_dof, task_dof) = -fstar_weight * f_star_;

    if (Robot.showdata)
    {
        Robot.showdata = false;
    }

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Fsl(6 * i + 0, 6 * i + 0) = 0.003;
        Fsl(6 * i + 1, 6 * i + 1) = 0.003;
        Fsl(6 * i + 2, 6 * i + 2) = 0.001;
        Fsl(6 * i + 3, 6 * i + 3) = 0.01;
        Fsl(6 * i + 4, 6 * i + 4) = 0.01;
        Fsl(6 * i + 5, 6 * i + 5) = 0.01;
    }

    double rr = DyrosMath::minmax_cut(ratio_r / ratio_l * 10, 1, 10);
    double rl = DyrosMath::minmax_cut(ratio_l / ratio_r * 10, 1, 10);

    //rr = ratio_r;
    //rl = ratio_l;
    if (Robot.ee_[0].contact && Robot.ee_[1].contact)
    {
        if (Robot.qp2nd)
        {
            //std::cout << "left : " << rr << "\t right : " << rl << std::endl;
            Fsl(0, 0) = 0.003 * rr;
            Fsl(1, 1) = 0.003 * rr;

            Fsl(3, 3) = 0.01 * rr;
            Fsl(4, 4) = 0.01 * rr;

            Fsl(6, 6) = 0.003 * rl;
            Fsl(7, 7) = 0.003 * rl;

            Fsl(9, 9) = 0.01 * rl;
            Fsl(10, 10) = 0.01 * rl;
        }
    }

    H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl.transpose() * Fsl;

    //Rigid Body Dynamcis Equality Constraint
    A.block(0, 0, task_dof, MODEL_DOF) = Robot.J_task_inv_T * Robot.Slc_k_T;
    A.block(0, MODEL_DOF + contact_dof, task_dof, task_dof) = -Robot.lambda;
    lbA.segment(0, task_dof) = Robot.J_task_inv_T * Robot.G;
    ubA.segment(0, task_dof) = Robot.J_task_inv_T * Robot.G;

    //Contact Force Equality constraint
    A.block(task_dof, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(task_dof, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //std::cout << "calc done!" << std::endl;
    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 5 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 5 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;

        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }
    }

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(task_dof + contact_dof + i) = 0.0;
        ubA(task_dof + contact_dof + i) = 1000.0;
    }

    //std::cout << "calc done!" << std::endl;
    //Torque bound setting
    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -300;
        ub(i) = 300;
    }
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -1000;
        ub(MODEL_DOF + i) = 1000;
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        ub(MODEL_DOF + 6 * i + 2) = -5;
        ub(MODEL_DOF + 6 * i + 5) = 1000;
        lb(MODEL_DOF + 6 * i + 5) = -1000;
    }

    for (int i = 0; i < task_dof; i++)
    {
        lb(MODEL_DOF + contact_dof + i) = -1000;
        ub(MODEL_DOF + contact_dof + i) = 1000;
    }

    //std::cout << "calc done!" << std::endl;
    QP_torque.EnableEqualityCondition(0.0001);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    VectorXd qpres;
    //if()
    if (QP_torque.SolveQPoases(200, qpres))
    {
        task_torque = qpres.segment(0, MODEL_DOF);
        //std::cout << "task_torque" << task_torque.transpose() << std::endl;
    }
    else
    {
        Robot.task_control_switch = false;
        Robot.contact_redistribution_mode = 0;
        task_torque = gravity_compensation_torque(Robot);
    }

    return task_torque; // + gravity_torque;
}

VectorQd WholebodyController::task_control_torque_QP3(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    //Without fstar regulation.

    //Desired ZMP!
    //ZMP is the most important thing of walking.
    //If desired ZMP exists...
    VectorQd task_torque;
    VectorXd f_star_qp_;

    //VectorQd gravity_torque = gravity_compensation_torque(Robot, dc.fixedgravity);

    Robot.task_dof = J_task.rows();

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_inv = (DyrosMath::pinv_glsSVD(J_task.transpose())).transpose();
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    MatrixXd J_com = Robot.link_[COM_id].Jac_COM_p;
    MatrixXd lambda_com_inv = J_com * Robot.A_matrix_inverse * Robot.N_C * J_com.transpose();
    MatrixXd J_com_inv_T = lambda_com_inv.inverse() * J_com * Robot.A_matrix_inverse * Robot.N_C;

    Vector3d fstar_com;
    fstar_com = lambda_com_inv * J_com_inv_T * Robot.J_task_T * Robot.lambda * f_star_;

    double zmp_com_e_x, zmp_com_e_y;
    zmp_com_e_x = Robot.com_.pos(0) - Robot.com_.pos(2) * Robot.link_[COM_id].a_traj(0) / 9.81;
    zmp_com_e_y = Robot.com_.pos(1) - Robot.com_.pos(2) * Robot.link_[COM_id].a_traj(1) / 9.81;

    double dist_l, dist_r;
    dist_l = sqrt((Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y));
    dist_r = sqrt((Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y));

    double ratio_r, ratio_l;
    ratio_r = dist_l / (dist_l + dist_r);
    ratio_l = dist_r / (dist_l + dist_r);

    static int task_dof, contact_dof;
    static bool qp_init_;
    int constraint_per_contact = 10;
    bool qpt_info = false;

    static int variable_size;
    static int constraint_size;

    static bool contact_before[4];

    if (Robot.init_qp || ((task_dof != Robot.task_dof) || (contact_dof != 6 * Robot.contact_index)))
    {
        task_dof = Robot.task_dof;
        contact_dof = 6 * Robot.contact_index;
        for (int i = 0; i < 4; i++)
        {
            contact_before[i] = Robot.ee_[i].contact;
        }
        std::cout << "############################" << std::endl
                  << "QP3 initialize ! " << std::endl
                  << "Task Dof    = " << Robot.task_dof << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        qp_init_ = true;
        std::cout << std::endl
                  << "############################" << std::endl;
        variable_size = MODEL_DOF + contact_dof;
        constraint_size = task_dof + contact_dof + constraint_per_contact * Robot.contact_index; // + 2;
        QP_torque.InitializeProblemSize(variable_size, constraint_size);
        Robot.init_qp = false;
    }

    //QP initialize!

    MatrixXd H, A, W, R, MR;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    R.setZero(contact_dof, contact_dof);
    MR.setZero(contact_dof, contact_dof);

    Matrix3d RE[4];

    Vector3d VXE[4], VYE[4], VZE[4];

    VZE[0] = (Robot.link_[Left_Foot - 5].xpos - Robot.ee_[0].cp_);
    VZE[1] = (Robot.link_[Right_Foot - 5].xpos - Robot.ee_[1].cp_);
    VZE[2] = (Robot.link_[Left_Hand - 7].xpos - Robot.ee_[2].cp_);
    VZE[3] = (Robot.link_[Right_Hand - 7].xpos - Robot.ee_[3].cp_);

    //std::cout << VZE[0].transpose() << std::endl;
    for (int i = 0; i < 4; i++)
    {
        VZE[i].normalize();
        VXE[i] << 1, 0, -VZE[i](0) / VZE[i](2);
        VXE[i].normalize();
        VYE[i] = (VXE[i].cross(VZE[i]));
        VYE[i].normalize();

        RE[i] << VXE[i], -VYE[i], VZE[i];
    }
    //std::cout << VZE[0].transpose() << std::endl;

    //std::cout << RE[0] << std::endl;

    //RE[0].block(0, 1, 3, 1) = (Robot.link_)

    for (int i = 0; i < Robot.contact_index; i++)
    {
        //R.block(6 * i, 6 * i, 3, 3) = RE[Robot.ee_idx[i]].transpose();
        //R.block(6 * i + 3, 6 + 3 * i, 3, 3) = RE[Robot.ee_idx[i]].transpose();

        R.block(6 * i, 6 * i, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        R.block(6 * i + 3, 6 * i + 3, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
    }

    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);
    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::

    double ea_weight = 5.0;
    //W = Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * N_task.transpose() * Robot.A_matrix * N_task * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    //g.segment(0, MODEL_DOF) = -ea_weight * Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * N_task.transpose() * Robot.A_matrix * N_task * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    g.segment(0, MODEL_DOF) = -ea_weight * Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = ea_weight * W; // + 0.01 * MatrixXd::Identity(MODEL_DOF,MODEL_DOF);

    if (Robot.showdata)
    {
        Robot.showdata = false;
    }

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);

    for (int i = 0; i < Robot.contact_index; i++)
    {
        //Fsl(6 * i + 0, 6 * i + 0) = 0.0;
        //Fsl(6 * i + 1, 6 * i + 1) = 0.0;
        //Fsl(6 * i + 2, 6 * i + 2) = 0.0;
        //Fsl(6 * i + 3, 6 * i + 3) = 0.0001;
        //Fsl(6 * i + 4, 6 * i + 4) = 0.0001;
        //Fsl(6 * i + 5, 6 * i + 5) = 0.0001;
    }

    double rr = DyrosMath::minmax_cut(ratio_r / ratio_l * 10, 1, 10);
    double rl = DyrosMath::minmax_cut(ratio_l / ratio_r * 10, 1, 10);

    double ratioFoots[4] = {rr, rl, 1, 1};

    //H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = R * Fsl.transpose() * Fsl * R.transpose();

    H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl.transpose() * Fsl;

    //Rigid Body Dynamcis Equality Constraint
    Robot.Slc_k.setZero(MODEL_DOF, MODEL_DOF + 6);
    Robot.Slc_k.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();
    Robot.Slc_k_T = Robot.Slc_k.transpose();

    A.block(0, 0, task_dof, MODEL_DOF) = Robot.J_task_inv_T * Robot.Slc_k_T;
    //A.block(0, MODEL_DOF + contact_dof, task_dof, task_dof) = -Robot.lambda;
    lbA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;
    ubA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;
    //lbA.segment(0, task_dof) = Robot.J_task_inv_T * Robot.G;
    //ubA.segment(0, task_dof) = Robot.J_task_inv_T * Robot.G;

    //Contact Force Equality constraint
    A.block(task_dof, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(task_dof, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //std::cout << "calc done!" << std::endl;
    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        //R(6 * i, 6 * i, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        //R(6 * i + 3, 6 * i + 3, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();

        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 5 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 5 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;

        /*
        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        } */
    }
    A.block(task_dof + contact_dof, MODEL_DOF, constraint_per_contact * Robot.contact_index, contact_dof) =
        A.block(task_dof + contact_dof, MODEL_DOF, constraint_per_contact * Robot.contact_index, contact_dof) * R;

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(task_dof + contact_dof + i) = 0.0;
        ubA(task_dof + contact_dof + i) = 100000.0;
    }

    /*
    if (Robot.contact_index > 2)
    {
        for (int i = 0; i < Robot.contact_index; i++)
        {
            A(task_dof + contact_dof + constraint_per_contact * Robot.contact_index, MODEL_DOF + 2 + 6 * i) = Robot.ee_[Robot.ee_idx[i]].cp_[0] - Robot.com_.pos[0];
            A(task_dof + contact_dof + constraint_per_contact * Robot.contact_index + 1, MODEL_DOF + 2 + 6 * i) = Robot.ee_[Robot.ee_idx[i]].cp_[1] - Robot.com_.pos[1];
        }
        lbA(task_dof + contact_dof + constraint_per_contact * Robot.contact_index) = 0.0;
        ubA(task_dof + contact_dof + constraint_per_contact * Robot.contact_index) = 0.0;

        lbA(task_dof + contact_dof + constraint_per_contact * Robot.contact_index + 1) = 0.0;
        ubA(task_dof + contact_dof + constraint_per_contact * Robot.contact_index + 1) = 0.0;
    }*/
    //std::cout << "calc done!" << std::endl;
    //Torque bound setting

    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -1500 / NM2CNT_d[i];
        ub(i) = 1500 / NM2CNT_d[i];
    }
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -10000;
        ub(MODEL_DOF + i) = 10000;
    }

    double zforce = 0;
    double zforce_des = 10;
    double trans_time = Robot.contact_transition_time;
    for (int i = 0; i < Robot.contact_index; i++)
    {

        if (Robot.ee_[Robot.ee_idx[i]].contact_transition_mode == 1)
        {
            lb(MODEL_DOF + 6 * i + 2) = -1000 * (Robot.control_time_ - Robot.ee_[Robot.ee_idx[i]].contact_time) / trans_time;
            ub(MODEL_DOF + 6 * i + 2) = -zforce_des * (Robot.control_time_ - Robot.ee_[Robot.ee_idx[i]].contact_time) / trans_time;
            //std::cout << lb(MODEL_DOF + 6 * i + 2) << "\t" << ub(MODEL_DOF + 6 * i + 2) << std::endl;
        }
        else if (Robot.ee_[Robot.ee_idx[i]].contact_transition_mode == 0)
        {
            lb(MODEL_DOF + 6 * i + 2) = -1000 * (Robot.ee_[Robot.ee_idx[i]].contact_time + trans_time - Robot.control_time_) / trans_time;
            ub(MODEL_DOF + 6 * i + 2) = -zforce_des * (Robot.ee_[Robot.ee_idx[i]].contact_time + trans_time - Robot.control_time_) / trans_time;
            //std::cout << lb(MODEL_DOF + 6 * i + 2) << "\t" << ub(MODEL_DOF + 6 * i + 2) << std::endl;
        }
        else
        {
            lb(MODEL_DOF + 6 * i + 2) = -1000;
            ub(MODEL_DOF + 6 * i + 2) = -zforce_des;
        }

        ub(MODEL_DOF + 6 * i + 5) = 10000;
        lb(MODEL_DOF + 6 * i + 5) = -10000;
    }

    //std::cout << "calc done!" << std::endl;
    //QP_torque.EnableEqualityCondition(0.0001);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    //std::cout << "Loop" << std::endl;
    VectorXd qpres;
    int solve_result;
    solve_result = QP_torque.SolveQPoases(200, qpres);

    /*
    int setup_result;
    int solve_result;
    bool qp_hots = false;
    if (qp_init_)
    {
        setup_result = QP_torque3_.setup(H, g, A, lbA, ubA, lb, ub);
        solve_result = QP_torque3_.solve(qpres);

        qp_init_ = false;
        qp_hots = true;
        //std::cout << "qpset" << std::endl;

        //std::cout << solve_result << std::endl
        //          << qpres.segment(0, MODEL_DOF).transpose() << std::endl;
    }
    else
    {
        //std::cout << "qphots" << std::endl;
        setup_result = QP_torque3_.hotstart(H, g, A, lbA, ubA, lb, ub);
        solve_result = QP_torque3_.solve(qpres);

        //std::cout << solve_result << std::endl
        //          << qpres.segment(0, MODEL_DOF).transpose() << std::endl;
    }*/

    //QP_torque3_.setup()

    //std::cout << "////" << std::endl;
    //std::cout << qpres.segment(MODEL_DOF, contact_dof).transpose() << std::endl;
    //std::cout << (R * qpres.segment(MODEL_DOF, contact_dof)).transpose() << std::endl;
    //std::cout << (R.transpose() * qpres.segment(MODEL_DOF, contact_dof)).transpose() << std::endl;

    if (solve_result)
    {
        task_torque = qpres.segment(0, MODEL_DOF);
    }
    else
    {
        std::cout << solve_result << "qp3 solve failed. changing to gravity compensation" << std::endl;
        Robot.task_control_switch = false;
        Robot.contact_redistribution_mode = 0;
        task_torque = gravity_compensation_torque(Robot);
        QP_torque.InitializeProblemSize(variable_size, constraint_size);
    }

    return task_torque;
}

VectorQd WholebodyController::task_control_torque_QP2_with_contactforce_feedback(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    VectorQd task_torque;
    VectorXd f_star_qp_;
    VectorXd contactforce_ft;

    //VectorQd gravity_torque = gravity_compensation_torque(Robot, dc.fixedgravity);
    double friction_ratio = 0.1;
    double friction_ratio_z = 0.001;
    //qptest
    double foot_x_length = 0.12;
    double foot_y_length = 0.04;

    Robot.task_dof = J_task.rows();

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_inv = (DyrosMath::pinv_glsSVD(J_task.transpose())).transpose();
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    MatrixXd J_com = Robot.link_[COM_id].Jac_COM_p;
    MatrixXd lambda_com_inv = J_com * Robot.A_matrix_inverse * Robot.N_C * J_com.transpose();
    MatrixXd J_com_inv_T = lambda_com_inv.inverse() * J_com * Robot.A_matrix_inverse * Robot.N_C;

    Vector3d fstar_com;
    fstar_com = lambda_com_inv * J_com_inv_T * Robot.J_task_T * Robot.lambda * f_star_;

    double zmp_com_e_x, zmp_com_e_y;
    zmp_com_e_x = Robot.com_.pos(0) - Robot.com_.pos(2) * fstar_com(0) / 9.81;
    zmp_com_e_y = Robot.com_.pos(1) - Robot.com_.pos(2) * fstar_com(1) / 9.81;

    double dist_l, dist_r;
    dist_l = sqrt((Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Left_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Left_Foot].xpos_contact(1) - zmp_com_e_y));
    dist_r = sqrt((Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) * (Robot.link_[Right_Foot].xpos_contact(0) - zmp_com_e_x) + (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y) * (Robot.link_[Right_Foot].xpos_contact(1) - zmp_com_e_y));

    double ratio_r, ratio_l;
    ratio_r = dist_l / (dist_l + dist_r);
    ratio_l = dist_r / (dist_l + dist_r);

    static int task_dof, contact_dof;
    int constraint_per_contact = 14;
    bool qpt_info = false;

    if ((task_dof != Robot.task_dof) || (contact_dof != 6 * Robot.contact_index))
    {
        task_dof = Robot.task_dof;
        contact_dof = 6 * Robot.contact_index;
        std::cout << "############################" << std::endl
                  << "QP initialize ! " << std::endl
                  << "Task Dof    = " << Robot.task_dof << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        std::cout << std::endl
                  << "############################" << std::endl;

        qpt_info = true;
    }

    int variable_size = MODEL_DOF + contact_dof + task_dof;
    int constraint_size = task_dof + contact_dof + constraint_per_contact * Robot.contact_index;

    //QP initialize!
    QP_torque.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::

    MatrixXd N_task;
    N_task.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    N_task = MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) - Robot.J_task_inv * Robot.J_task;

    double ea_weight = 1.0;
    //W = Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * N_task.transpose() * Robot.A_matrix * N_task * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    //g.segment(0, MODEL_DOF) = -ea_weight * Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * N_task.transpose() * Robot.A_matrix * N_task * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    g.segment(0, MODEL_DOF) = -ea_weight * Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = ea_weight * W; // + 0.01 * MatrixXd::Identity(MODEL_DOF,MODEL_DOF);

    //fstar regulation ::
    double fstar_weight = 100.0;
    H.block(MODEL_DOF + contact_dof, MODEL_DOF + contact_dof, task_dof, task_dof) = fstar_weight * MatrixXd::Identity(task_dof, task_dof);
    g.segment(MODEL_DOF + contact_dof, task_dof) = -fstar_weight * f_star_;

    if (Robot.showdata)
    {
        Robot.showdata = false;
    }

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);

    for (int i = 0; i < Robot.contact_index; i++)
    {
        Fsl(6 * i + 0, 6 * i + 0) = 0.0001;
        Fsl(6 * i + 1, 6 * i + 1) = 0.0001;
        Fsl(6 * i + 2, 6 * i + 2) = 0.0001;
        Fsl(6 * i + 3, 6 * i + 3) = 0.01;
        Fsl(6 * i + 4, 6 * i + 4) = 0.01;
        Fsl(6 * i + 5, 6 * i + 5) = 0.01;
    }

    double rr = DyrosMath::minmax_cut(ratio_r / ratio_l * 10, 1, 10);
    double rl = DyrosMath::minmax_cut(ratio_l / ratio_r * 10, 1, 10);

    if (Robot.ee_[0].contact && Robot.ee_[1].contact)
    {
        if (Robot.qp2nd)
        {
            Fsl(0, 0) = 0.0001 * rr;
            Fsl(1, 1) = 0.0001 * rr;

            Fsl(3, 3) = 0.01 * rr;
            Fsl(4, 4) = 0.01 * rr;

            Fsl(6, 6) = 0.0001 * rl;
            Fsl(7, 7) = 0.0001 * rl;

            Fsl(9, 9) = 0.01 * rl;
            Fsl(10, 10) = 0.01 * rl;
        }
    }

    H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl.transpose() * Fsl;

    //Rigid Body Dynamcis Equality Constraint
    A.block(0, 0, task_dof, MODEL_DOF) = Robot.J_task_inv_T * Robot.Slc_k_T;
    A.block(0, MODEL_DOF + contact_dof, task_dof, task_dof) = -Robot.lambda;
    lbA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;
    ubA.segment(0, task_dof) = Robot.lambda * f_star_ + Robot.J_task_inv_T * Robot.G;

    //Contact Force Equality constraint
    A.block(task_dof, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(task_dof, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //std::cout << "calc done!" << std::endl;
    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 3 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 4 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 12, MODEL_DOF + 5 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 12, MODEL_DOF + 2 + 6 * i) = -friction_ratio_z;
        A(task_dof + contact_dof + i * constraint_per_contact + 13, MODEL_DOF + 5 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 13, MODEL_DOF + 2 + 6 * i) = -friction_ratio_z;

        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }
    }

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(task_dof + contact_dof + i) = 0.0;
        ubA(task_dof + contact_dof + i) = 100000.0;
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -300;
        ub(i) = 300;
    }
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -10000;
        ub(MODEL_DOF + i) = 10000;
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        ub(MODEL_DOF + 6 * i + 2) = -25;
        ub(MODEL_DOF + 6 * i + 5) = 10000;
        lb(MODEL_DOF + 6 * i + 5) = -10000;
    }

    for (int i = 0; i < task_dof; i++)
    {
        lb(MODEL_DOF + contact_dof + i) = -10000;
        ub(MODEL_DOF + contact_dof + i) = 10000;
    }

    QP_torque.EnableEqualityCondition(1E-6);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    VectorXd qpres;
    if (QP_torque.SolveQPoases(1000, qpres))
    {
        task_torque = qpres.segment(0, MODEL_DOF);
    }
    else
    {
        task_torque = gravity_compensation_torque(Robot);
        Robot.task_control_switch = false;
        Robot.contact_redistribution_mode = 0;
    }

    //VectorXd fc = qpres.segment(MODEL_DOF, contact_dof);

    return task_torque;
}

VectorQd WholebodyController::task_control_torque_QP_gravity(RobotData &Robot)
{
    VectorQd task_torque;
    VectorXd f_star_qp_;
    Eigen::MatrixXd J_task;
    J_task.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);
    J_task.block(0, 6, MODEL_DOF, MODEL_DOF) = MatrixXd::Identity(MODEL_DOF, MODEL_DOF);
    //VectorQd gravity_torque = gravity_compensation_torque(Robot, dc.fixedgravity);
    double friction_ratio = 0.3;
    //qptest
    double foot_x_length = 0.12;
    double foot_y_length = 0.04;

    Robot.task_dof = J_task.rows();

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;

    static int task_dof, contact_dof;
    int constraint_per_contact = 8;
    bool qpt_info = false;

    if ((task_dof != Robot.task_dof) || (contact_dof != 6 * Robot.contact_index))
    {
        task_dof = Robot.task_dof;
        contact_dof = 6 * Robot.contact_index;
        std::cout << "############################" << std::endl
                  << "QP initialize ! " << std::endl
                  << "Task Dof    = " << Robot.task_dof << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        std::cout << std::endl
                  << "############################" << std::endl;

        qpt_info = true;
    }

    int variable_size = MODEL_DOF + contact_dof;
    int constraint_size = task_dof + contact_dof + constraint_per_contact * Robot.contact_index;

    //QP initialize!
    QP_torque.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::
    W = Robot.Slc_k * Robot.N_C.transpose() * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = W + MatrixXd::Identity(MODEL_DOF, MODEL_DOF);
    g.segment(0, MODEL_DOF) = -Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        //Fsl(6 * i + 0, 6 * i + 0) = 0.0001;
        //Fsl(6 * i + 1, 6 * i + 1) = 0.0001;
        Fsl(6 * i + 3, 6 * i + 3) = 0.001;
        Fsl(6 * i + 4, 6 * i + 4) = 0.001;
        Fsl(6 * i + 5, 6 * i + 5) = 0.001;
    }
    //H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl;

    //Rigid Body Dynamcis Equality Constraint
    A.block(0, 0, task_dof, MODEL_DOF) = Robot.J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;
    lbA.segment(0, task_dof) = Robot.J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    ubA.segment(0, task_dof) = Robot.J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.G;

    //Contact Force Equality constraint
    A.block(task_dof, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(task_dof, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(task_dof, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(task_dof + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(task_dof + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(task_dof + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }
    }

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(task_dof + contact_dof + i) = 0.0;
        ubA(task_dof + contact_dof + i) = 100000.0;
    }

    //std::cout << "calc done!" << std::endl;
    //Torque bound setting
    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -300;
        ub(i) = 300;
    }

    //ContactForce bound setting
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -1000;
        ub(MODEL_DOF + i) = 1000;
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        ub(MODEL_DOF + 6 * i + 2) = 0.0;
        //ub(MODEL_DOF + 6 * i + 5) = 0.05;
        //lb(MODEL_DOF + 6 * i + 5) = -0.05;
    }

    QP_torque.EnableEqualityCondition(1E-6);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    VectorXd qpres = QP_torque.SolveQPoases(100, false);
    task_torque = qpres.segment(0, MODEL_DOF);

    return task_torque; // + gravity_torque;
}

VectorXd WholebodyController::check_fstar(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    MatrixXd J_com = Robot.link_[COM_id].Jac_COM_p;
    MatrixXd lambda_com_inv = J_com * Robot.A_matrix_inverse * Robot.N_C * J_com.transpose();
    MatrixXd J_com_inv_T = lambda_com_inv.inverse() * J_com * Robot.A_matrix_inverse * Robot.N_C;

    Vector3d fstar_com;
    fstar_com = lambda_com_inv * J_com_inv_T * Robot.J_task_T * Robot.lambda * f_star_;

    double zmp_com_e_x, zmp_com_e_y;
    zmp_com_e_x = Robot.com_.pos(0) - Robot.com_.pos(2) * fstar_com(0) / 9.81;
    zmp_com_e_y = Robot.com_.pos(1) - Robot.com_.pos(2) * fstar_com(1) / 9.81;

    bool b1 = (zmp_com_e_x > Robot.link_[Left_Foot].xpos_contact(0) + Robot.ee_[0].cs_x_length) && (zmp_com_e_x > Robot.link_[Right_Foot].xpos_contact(0) + Robot.ee_[0].cs_x_length);

    bool b2 = (zmp_com_e_x < Robot.link_[Left_Foot].xpos_contact(0) - Robot.ee_[0].cs_x_length) && (zmp_com_e_x < Robot.link_[Right_Foot].xpos_contact(0) - Robot.ee_[0].cs_x_length);

    bool b3 = (zmp_com_e_y > Robot.link_[Left_Foot].xpos_contact(1) + Robot.ee_[0].cs_y_length) && (zmp_com_e_y > Robot.link_[Right_Foot].xpos_contact(1) + Robot.ee_[0].cs_y_length);

    bool b4 = (zmp_com_e_y < Robot.link_[Left_Foot].xpos_contact(1) - Robot.ee_[0].cs_y_length) && (zmp_com_e_y < Robot.link_[Right_Foot].xpos_contact(1) - Robot.ee_[0].cs_y_length);

    if (b1 || b2 || b3 || b4)
    {
        std::cout << "Control command out of support polygon! " << std::endl;
    }
}
VectorQd WholebodyController::contact_torque_calc_from_QP2(RobotData &Robot, VectorQd command_torque)
{
}
VectorQd WholebodyController::contact_torque_calc_from_QP(RobotData &Robot, VectorQd command_torque)
{

    if (Robot.contact_index > 1)
    {
        static int contact_dof;
        static bool print_data_qp_ = false;

        int constraint_per_contact = 10;
        if (Robot.init_qp || (contact_dof != 6 * Robot.contact_index))
        {
            print_data_qp_ = true;
            std::cout << "############################" << std::endl
                      << "QP contact torque calc initialize ! " << std::endl
                      << "Task Dof    = " << Robot.task_dof << std::endl
                      << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                      << "Contact Link : ";
            for (int i = 0; i < Robot.contact_index; i++)
            {
                std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
            }
            std::cout << std::endl
                      << "############################" << std::endl;
            contact_dof = 6 * Robot.contact_index;
            QP_test.InitializeProblemSize(6 * Robot.contact_index, 6 + constraint_per_contact * Robot.contact_index + 6 * Robot.contact_index);
        }

        VectorXd ContactForce__ = get_contact_force(Robot, command_torque);

        //std::cout << "zmp : " << GetZMPpos(Robot, ContactForce__) << std::endl
        //          << std::endl
        //          << std::endl;

        double a1 = 0.0;
        double a2 = 1.0;
        //qptest

        MatrixXd H, A, M, R;
        H.setZero(6 * Robot.contact_index, 6 * Robot.contact_index);
        M.setZero(6 * Robot.contact_index, 6 * Robot.contact_index);
        R.setZero(6 * Robot.contact_index, 6 * Robot.contact_index);
        for (int i = 0; i < Robot.contact_index; i++)
        {
            //M(6 * i, 6 * i) = 5.0;
            //M(6 * i + 1, 6 * i + 1) = 5.0;
            //M(6 * i + 2, 6 * i + 2) = 0.2;
            //M(6 * i + 3, 6 * i + 3) = 50.0;
            //M(6 * i + 4, 6 * i + 4) = 50.0;
            //M(6 * i + 5, 6 * i + 5) = 1.0;
        }
        for (int i = 0; i < Robot.contact_index; i++)
        {
            R.block(6 * i, 6 * i, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            R.block(6 * i + 3, 6 * i + 3, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }

        //H = a1 * MatrixXd::Identity(Robot.contact_index * 6, Robot.contact_index * 6) + a2 * R.transpose() * M * R;

        A.setZero(6 + constraint_per_contact * Robot.contact_index + 6 * Robot.contact_index, 6 * Robot.contact_index);
        for (int i = 0; i < Robot.contact_index; i++)
        {
            A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
            A.block(3, 6 * i, 3, 3) = DyrosMath::skm(Robot.ee_[Robot.ee_idx[i]].cp_ - Robot.com_.pos);
        }

        for (int i = 0; i < Robot.contact_index; i++)
        {
            A(6 + i * constraint_per_contact + 0, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
            A(6 + i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 1, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
            A(6 + i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

            A(6 + i * constraint_per_contact + 2, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
            A(6 + i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 3, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
            A(6 + i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

            A(6 + i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
            A(6 + i * constraint_per_contact + 4, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
            A(6 + i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 5, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

            A(6 + i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
            A(6 + i * constraint_per_contact + 6, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
            A(6 + i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 7, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

            A(6 + i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
            A(6 + i * constraint_per_contact + 8, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
            A(6 + i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
            A(6 + i * constraint_per_contact + 9, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        }
        A.block(6, 0, constraint_per_contact * Robot.contact_index, contact_dof) =
            A.block(6, 0, constraint_per_contact * Robot.contact_index, contact_dof) * R;

        A.block(6 + Robot.contact_index * constraint_per_contact, 0, contact_dof, contact_dof) = R;
        /*
        A.block(6 + Robot.contact_index * constraint_per_contact, 0, 6 * Robot.contact_index, 6 * Robot.contact_index) = R;

        MatrixXd tmp = A.block(6, 0, Robot.contact_index * constraint_per_contact, 6 * Robot.contact_index);

        A.block(6, 0, Robot.contact_index * constraint_per_contact, 6 * Robot.contact_index) = tmp * R;*/
        VectorXd force_res = A.block(0, 0, 6, Robot.contact_index * 6) * ContactForce__;
        VectorXd g, lb, ub, lbA, ubA;
        g.setZero(Robot.contact_index * 6);
        g = 0.0 * ContactForce__;

        lbA.setZero(6 + constraint_per_contact * Robot.contact_index + 6 * Robot.contact_index);
        ubA.setZero(6 + constraint_per_contact * Robot.contact_index + 6 * Robot.contact_index);
        lbA.segment(0, 6) = force_res;
        ubA.segment(0, 6) = force_res;
        ub.setZero(6 * Robot.contact_index);
        lb.setZero(6 * Robot.contact_index);
        for (int i = 0; i < 6 * Robot.contact_index; i++)
        {
            lbA(6 + constraint_per_contact * Robot.contact_index + i) = -1000.0;
            ubA(6 + constraint_per_contact * Robot.contact_index + i) = 1000.0;
        }
        double zforce = 0;
        double zforce_des = 2;
        double trans_time = Robot.contact_transition_time;
        for (int i = 0; i < Robot.contact_index; i++)
        {
            if (Robot.ee_[Robot.ee_idx[i]].contact_transition_mode == 1)
            {
                lbA(6 + constraint_per_contact * Robot.contact_index + i * 6 + 2) = -1000 * (Robot.control_time_ - Robot.ee_[Robot.ee_idx[i]].contact_time) / trans_time;
                ubA(6 + constraint_per_contact * Robot.contact_index + i * 6 + 2) = -zforce_des * (Robot.control_time_ - Robot.ee_[Robot.ee_idx[i]].contact_time) / trans_time;
                //std::cout << lb(MODEL_DOF + 6 * i + 2) << "\t" << ub(MODEL_DOF + 6 * i + 2) << std::endl;
            }
            else if (Robot.ee_[Robot.ee_idx[i]].contact_transition_mode == 0)
            {
                lbA(6 + constraint_per_contact * Robot.contact_index + i * 6 + 2) = -1000 * (Robot.ee_[Robot.ee_idx[i]].contact_time + trans_time - Robot.control_time_) / trans_time;
                ubA(6 + constraint_per_contact * Robot.contact_index + i * 6 + 2) = -zforce_des * (Robot.ee_[Robot.ee_idx[i]].contact_time + trans_time - Robot.control_time_) / trans_time;
                //std::cout << lb(MODEL_DOF + 6 * i + 2) << "\t" << ub(MODEL_DOF + 6 * i + 2) << std::endl;
            }
            else
            {
                lbA(6 + constraint_per_contact * Robot.contact_index + i * 6 + 2) = -1000;
                ubA(6 + constraint_per_contact * Robot.contact_index + i * 6 + 2) = -zforce_des;
            }
        }

        for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
        {
            lbA(6 + i) = 0.0;
            ubA(6 + i) = 10000.0;
        }

        QP_test.EnableEqualityCondition(0.0001);
        QP_test.UpdateMinProblem(H, g);
        QP_test.UpdateSubjectToAx(A, lbA, ubA);
        //QP_test.UpdateSubjectToX(lb, ub);

        //ROS_INFO("l8");
        VectorXd force_redistribute; // = QP_test.SolveQPoases(200, false);

        if (QP_test.SolveQPoases(100, force_redistribute))
        {
        }
        else
        {
            std::cout << "qp contact torque solve error" << std::endl;
        }

        if (Robot.contact_index > 2)
        {
            std::cout << "////////////////////////////////////////////" << std::endl;
            //std::cout << "A*fc  : " << std::endl
            //          << (A * force_redistribute).transpose().segment(6 + constraint_per_contact * Robot.contact_index, 6 * Robot.contact_index) << std::endl;
            //std::cout << "ubA : " << std::endl
            //         << ubA.segment(6 + constraint_per_contact * Robot.contact_index, 6 * Robot.contact_index) << std::endl;
            //std::cout << "contact force : " << std::endl
            //          << force_redistribute.transpose() << std::endl;
            std::cout << "local force : " << std::endl
                      << (R * force_redistribute).transpose() << std::endl;
        }

        //int setup_result = QP_contact.setup(H, g, A, lbA, ubA, lb, ub);
        //int solve_result = QP_contact.solve(force_redistribute);
        /*
        if (false)
        {
            if (print_data_qp_)
            {
                std::cout << "############################################" << std::endl;
                std::cout << "############################################" << std::endl;
                //std::cout << "Eigen v : " << H.eigenvalues() << std::endl;
                //std::cout << "setup res : " << setup_result << std::endl;

                std::cout << "solve res : " << solve_result << std::endl;
                std::cout << "Contact Force with torque : " << std::endl;
                std::cout << ContactForce__.transpose() << std::endl;
                std::cout << " lambda * fstar :" << std::endl;
                std::cout << (Robot.lambda.block(0, 0, 3, 3) * Robot.f_star.segment(0, 3)).transpose() << std::endl;

                std::cout << "lambda without " << std::endl;
                std::cout << (Robot.J_task * Robot.A_matrix_inverse * Robot.J_task_T).inverse() << std::endl;

                std::cout << "lambda with" << std::endl;
                std::cout << Robot.lambda << std::endl;

                std::cout << "mass*f_star : " << std::endl
                          << (Robot.total_mass * Robot.f_star).segment(0, 3).transpose() << "  mg  : " << Robot.total_mass * -9.81 << std::endl;

                std::cout << force_redistribute << std::endl;

                std::cout << "############################################" << std::endl;
                std::cout << " H : " << std::endl
                          << H << std::endl;
                std::cout << "############################################" << std::endl;
                std::cout << " g : " << std::endl
                          << g.transpose() << std::endl;
                std::cout << "############################################" << std::endl;
                std::cout << " A : " << std::endl
                          << A << std::endl;
                std::cout << "############################################" << std::endl;
                std::cout << " lba : " << std::endl
                          << lbA.transpose() << std::endl;
                std::cout << "############################################" << std::endl;
                std::cout << " uba : " << std::endl
                          << ubA.transpose() << std::endl;
                std::cout << "############################################" << std::endl;
                std::cout << " lb : " << std::endl
                          << lb.transpose() << std::endl;
                std::cout << "############################################" << std::endl;
                std::cout << " ua : " << std::endl
                          << ub.transpose() << std::endl;
                
                Eigen::Vector12d fc_redis;qqq
                double fc_ratio;
                fc_redis.setZero();
                Eigen::VectorXd fc_rvc;
                Eigen::VectorXd TorqueContact = contact_force_redistribution_torque(Robot, command_torque, fc_redis, fc_ratio);

                fc_rvc = get_contact_force(Robot, TorqueContact + command_torque);
                fc_rvc = fc_redis;
                std::cout << "fc by yslee" << std::endl;
                std::cout << fc_rvc.transpose() << std::endl
                          << "############################################" << std::endl;
                std::cout << "A*fc_redis:" << std::endl;
                std::cout << A * fc_rvc << std::endl;
                std::cout << "############################################" << std::endl;
                
                print_data_qp_ = false;
            }

            std::cout << "qp error .... 2nd easy trial ..." << std::endl;

            for (int i = 0; i < Robot.contact_index; i++)
            {
                lbA(6 + i * constraint_per_contact + 4) = -1000.0;
                lbA(6 + i * constraint_per_contact + 5) = -1000.0;
                lbA(6 + i * constraint_per_contact + 6) = -1000.0;
                lbA(6 + i * constraint_per_contact + 7) = -1000.0;
            }
            setup_result = QP_contact.setup(H, g, A, lbA, ubA, lb, ub);
            solve_result = QP_contact.solve(force_redistribute);

            if (solve_result == 1)
            {
                std::cout << "second trial success" << std::endl;
            }
            else
            {
                std::cout << "second trial failed" << std::endl;

            } //std::cout<<"############################################"<<std::endl;
            //QP_test.PrintMinProb();
            //QP_test.PrintSubjectToAx();
            //QP_test.PrintSubjectTox();
        }*/

        //ROS_INFO("l9");
        result_temp = force_redistribute;

        //ROS_INFO("l10");
        VectorXd torque_contact_ = contact_force_custom(Robot, command_torque, ContactForce__, force_redistribute);

        //std::cout << "###########################" << std::endl;
        //std::cout << "redistribute" << std::endl;
        //std::cout << force_redistribute << std::endl;
        //std::cout << "position of lHand" <<std::endl;
        //std::cout << Robot.link_[Left_Hand].xpos_contact <<std::endl;
        // std::cout << "resultant force" << std::endl;
        // std::cout << force_res << std::endl;
        // std::cout << "A matrix " << std::endl;
        // std::cout << A << std::endl;
        // std::cout << "lbA" << std::endl;
        // std::cout << lbA << std::endl;
        // std::cout << "ubA" << std::endl;
        // std::cout << ubA << std::endl;
        //std::cout << "ub" << std::endl;
        //std::cout << ub << std::endl;
        // std::cout << "lb" << std::endl;
        // std::cout << lb << std::endl;

        //ROS_INFO("l2");
        return torque_contact_;
    }
    return VectorXd::Zero(MODEL_DOF);
}

VectorQd WholebodyController::footRotateAssist(RobotData &Robot, bool left, bool right)
{
    //if prelanding approach engaged
    //enable roll/yaw control with ankle joint.

    //Right foot
    Vector3d RF_rot, RF_ang_v;
    Vector3d LF_rot, LF_ang_v;

    Vector3d RF_eulr, LF_eulr;
    Vector3d RF_eulr_l, LF_eulr_l;

    RF_eulr = DyrosMath::rot2Euler_tf(Robot.link_[Right_Foot].Rotm);
    LF_eulr = DyrosMath::rot2Euler_tf(Robot.link_[Left_Foot].Rotm);

    RF_eulr_l = DyrosMath::rot2Euler_tf(DyrosMath::rotateWithZ(-RF_eulr(2)) * Robot.link_[Right_Foot].Rotm);
    LF_eulr_l = DyrosMath::rot2Euler_tf(DyrosMath::rotateWithZ(-LF_eulr(2)) * Robot.link_[Left_Foot].Rotm);

    RF_ang_v = DyrosMath::rotateWithZ(-RF_eulr(2)) * Robot.link_[Right_Foot].v;
    LF_ang_v = DyrosMath::rotateWithZ(-RF_eulr(2)) * Robot.link_[Right_Foot].v;

    VectorQd torque_assist;
    torque_assist.setZero();

    double pitch_p, pitch_d, roll_p, roll_d;

    pitch_p = 100;
    pitch_d = 20;
    roll_p = 100;
    roll_d = 20;

    if (left)
    {
        torque_assist(4) = -pitch_p * LF_eulr_l(1) - pitch_d * LF_ang_v(1); //pitch
        torque_assist(5) = -roll_p * LF_eulr_l(0) - roll_d * LF_ang_v(0);   //roll
    }

    if (right)
    {
        torque_assist(10) = -pitch_p * RF_eulr_l(1) - pitch_d * RF_ang_v(1);
        torque_assist(11) = -roll_p * RF_eulr_l(0) - roll_d * RF_ang_v(0);
        //std::cout << " LF eulr x : " << RF_eulr_l(1) << "  torque_assist : " << torque_assist(10) << " LF eulr y : " << RF_eulr_l(0) << "  torque_assist : " << torque_assist(11) << std::endl;
    }

    //get foot orientation

    //get foot angular velocity

    //simply, orientation, foot angular velocity controller.

    return torque_assist;
}

/*
VectorQd WholebodyController::contact_torque_calc_from_QP_wall(VectorQd command_torque, double wall_friction_ratio)
{
    VectorXd ContactForce__ = get_contact_force(command_torque);
    QP_test.InitializeProblemSize(contact_index * 6, 6 + contact_index);
    MatrixXd H, A;
    H.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        H(6 * i, 6 * i) = 1;
        H(6 * i + 1, 6 * i + 1) = 1;
        H(6 * i + 2, 6 * i + 2) = 0.01;
        H(6 * i + 3, 6 * i + 3) = 100;
        H(6 * i + 4, 6 * i + 4) = 100;
        H(6 * i + 5, 6 * i + 5) = 100;
    }
    A.setZero(6 + contact_index, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
        A.block(3, 6 * i, 3, 3) = DyrosMath::skm(link_[contact_part[i]].xpos_contact - com_.pos);
    }
    VectorXd force_res = A.block(0, 0, 6, contact_index * 6) * ContactForce__;
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(contact_index * 6);
    lbA.setZero(6 + contact_index);
    ubA.setZero(6 + contact_index);
    lbA.segment(0, 6) = force_res;
    ubA.segment(0, 6) = force_res;
    ub.setZero(contact_index * 6);
    lb.setZero(contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {
        A(6 + i, 1 + 6 * i) = 1.0;

        ubA(6 + i) = 0.0;
        lbA(6 + i) = 0.0;
        if (contact_part[i] == Right_Foot)
        {
            A(6 + i, 2 + 6 * i) = -wall_friction_ratio;
        }
        if (contact_part[i] == Right_Hand)
        {
            A(6 + i, 2 + 6 * i) = -wall_friction_ratio;
        }
        /*if (contact_part[i] == Left_Foot)
        {
            A(6 + i, 2 + 6 * i) = wall_friction_ratio;
        }
        if (contact_part[i] == Left_Hand)
        {
            A(6 + i, 2 + 6 * i) = wall_friction_ratio;
        } 
}
for (int i = 0; i < contact_index * 6; i++)
{
    lb(i) = -1000;
    ub(i) = 1000;
}

for (int i = 0; i < contact_index; i++)
    ub(2 + 6 * i) = 0;

QP_test.EnableEqualityCondition(0.001);
QP_test.UpdateMinProblem(H, g);
QP_test.UpdateSubjectToAx(A, lbA, ubA);
QP_test.UpdateSubjectToX(lb, ub);
VectorXd force_redistribute = QP_test.SolveQPoases(100);

std::cout << "Contact Force now :  " << std::endl;
std::cout << ContactForce__ << std::endl;
std::cout << "Contact Force Redistribution : " << std::endl;
std::cout << force_redistribute << std::endl;

VectorQd torque_contact_ = contact_force_custom(command_torque, ContactForce__, force_redistribute);
return torque_contact_;
}

VectorQd WholebodyController::contact_torque_calc_from_QP_wall_mod2(VectorQd command_torque, double wall_friction_ratio)
{
    double a1, a2;

    a1 = 10.0; //Contactforce control ratio
    a2 = 1.0;  //moment minimize ratio

    VectorXd ContactForce__ = get_contact_force(command_torque);
    QP_test.InitializeProblemSize(contact_index * 6, 6 + contact_index * 5);
    MatrixXd H, A;
    H.setZero(contact_index * 6, contact_index * 6);

    MatrixXd M;
    M.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        M(6 * i, 6 * i) = 1.0;
        M(6 * i + 1, 6 * i + 1) = 1.0;
        M(6 * i + 2, 6 * i + 2) = 0.1;
        M(6 * i + 3, 6 * i + 3) = 100.0;
        M(6 * i + 4, 6 * i + 4) = 100.0;
        M(6 * i + 5, 6 * i + 5) = 100.0;
    }

    H = a1 * MatrixXd::Identity(contact_index * 6, contact_index * 6) + a2 * M;

    A.setZero(6 + contact_index * 5, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        A.block(0, 6 * i, 6, 6) = Matrix6d::Identity();
        A.block(3, 6 * i, 3, 3) = DyrosMath::skm(link_[contact_part[i]].xpos_contact - com_.pos);
    }
    VectorXd force_res = A.block(0, 0, 6, contact_index * 6) * ContactForce__;
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(contact_index * 6);

    g = -a1 * ContactForce__;
    lbA.setZero(6 + contact_index * 5);
    ubA.setZero(6 + contact_index * 5);
    lbA.segment(0, 6) = force_res;
    ubA.segment(0, 6) = force_res;
    ub.setZero(contact_index * 6);
    lb.setZero(contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {
        A(6 + i * 5, 1 + 6 * i) = 1.0;
        A(6 + i * 5 + 1, 3 + 6 * i) = 1.0;
        A(6 + i * 5 + 2, 3 + 6 * i) = 1.0;
        A(6 + i * 5 + 3, 4 + 6 * i) = 1.0;
        A(6 + i * 5 + 4, 4 + 6 * i) = 1.0;

        if (contact_part[i] == Right_Foot)
        {
            A(6 + i * 5, 2 + 6 * i) = -wall_friction_ratio;
            ubA(6 + i * 5) = 0.0;
            lbA(6 + i * 5) = -1000.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.03;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.03;
            lbA(6 + i * 5 + 1) = 0.0;
            ubA(6 + i * 5 + 1) = 1000.0;
            lbA(6 + i * 5 + 2) = -1000.0;
            ubA(6 + i * 5 + 2) = 0;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.05;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.05;
            lbA(6 + i * 5 + 3) = 0.0;
            ubA(6 + i * 5 + 3) = 1000.0;
            lbA(6 + i * 5 + 4) = -1000.0;
            ubA(6 + i * 5 + 4) = 0;
        }
        if (contact_part[i] == Right_Hand)
        {
            A(6 + i * 5, 2 + 6 * i) = -wall_friction_ratio;

            ubA(6 + i * 5) = 0.0;
            lbA(6 + i * 5) = -1000.0;

            A(6 + i * 5 + 1, 6 * i + 1) = -0.02;
            A(6 + i * 5 + 2, 6 * i + 1) = 0.02;
            lbA(6 + i * 5 + 1) = 0.0;
            ubA(6 + i * 5 + 1) = 1000.0;
            lbA(6 + i * 5 + 2) = -1000.0;
            ubA(6 + i * 5 + 2) = 0;

            A(6 + i * 5 + 3, 6 * i + 1) = -0.02;
            A(6 + i * 5 + 4, 6 * i + 1) = 0.02;
            lbA(6 + i * 5 + 3) = 0.0;
            ubA(6 + i * 5 + 3) = 1000.0;
            lbA(6 + i * 5 + 4) = -1000.0;
            ubA(6 + i * 5 + 4) = 0;
        }
        if (contact_part[i] == Left_Foot)
        {
            A(6 + i * 5, 2 + 6 * i) = wall_friction_ratio;

            ubA(6 + i * 5) = 1000.0;
            lbA(6 + i * 5) = 0.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.03;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.03;
            lbA(6 + i * 5 + 1) = -1000.0;
            ubA(6 + i * 5 + 1) = 0.0;
            lbA(6 + i * 5 + 2) = 0.0;
            ubA(6 + i * 5 + 2) = 1000.0;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.05;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.05;
            lbA(6 + i * 5 + 3) = -1000.0;
            ubA(6 + i * 5 + 3) = 0.0;
            lbA(6 + i * 5 + 4) = 0.0;
            ubA(6 + i * 5 + 4) = 1000.0;
        }
        if (contact_part[i] == Left_Hand)
        {
            A(6 + i * 5, 2 + 6 * i) = wall_friction_ratio;

            ubA(6 + i * 5) = 1000.0;
            lbA(6 + i * 5) = 0.0;

            A(6 + i * 5 + 1, 1 + 6 * i) = -0.02;
            A(6 + i * 5 + 2, 1 + 6 * i) = 0.02;

            A(6 + i * 5 + 3, 1 + 6 * i) = -0.02;
            A(6 + i * 5 + 4, 1 + 6 * i) = 0.02;

            lbA(6 + i * 5 + 1) = -1000.0;
            ubA(6 + i * 5 + 1) = 0.0;
            lbA(6 + i * 5 + 2) = 0.0;
            ubA(6 + i * 5 + 2) = 1000.0;

            lbA(6 + i * 5 + 3) = -1000.0;
            ubA(6 + i * 5 + 3) = 0.0;
            lbA(6 + i * 5 + 4) = 0.0;
            ubA(6 + i * 5 + 4) = 1000.0;
        }
    }
    for (int i = 0; i < contact_index * 6; i++)
    {
        lb(i) = -1000;
        ub(i) = 1000;
    }

    for (int i = 0; i < contact_index; i++)
        ub(2 + 6 * i) = 0;

    QP_test.EnableEqualityCondition(0.01);
    QP_test.UpdateMinProblem(H, g);
    QP_test.UpdateSubjectToAx(A, lbA, ubA);
    QP_test.UpdateSubjectToX(lb, ub);
    VectorXd force_redistribute = QP_test.SolveQPoases(200);

    std::cout << "Contact Force now :  " << std::endl;
    std::cout << ContactForce__ << std::endl;
    std::cout << "Contact Force Redistribution : " << std::endl;
    std::cout << force_redistribute << std::endl;

    VectorQd torque_contact_ = contact_force_custom(command_torque, ContactForce__, force_redistribute);
    result_temp = force_redistribute;
    return torque_contact_;
}
*/

VectorQd WholebodyController::CP_control_init(RobotData &Robot, double dT)
{
    double w_ = sqrt(9.81 / Robot.com_.pos(2));
    double b_ = exp(w_ * dT);

    Vector2d CP_displace;
    CP_displace(0) = 0.0;
    CP_displace(1) = 0.015;

    CP_ref[0] = Robot.com_.pos.segment(0, 2);
    CP_ref[1] = Robot.link_[Left_Foot].xpos.segment(0, 2) - CP_displace;
    CP_ref[2] = Robot.link_[Right_Foot].xpos.segment(0, 2) + CP_displace;
    CP_ref[3] = Robot.com_.pos.segment(0, 2);
}

VectorQd WholebodyController::CP_controller()
{
}

Vector6d WholebodyController::zmp_controller(RobotData &Robot, Vector2d ZMP, double height)
{
    double w_ = sqrt(9.81 / Robot.com_.pos(2));
    Vector3d desired_accel;
    desired_accel.segment(0, 2) = pow(w_, 2) * (Robot.com_.pos.segment(0, 2) - ZMP);
    desired_accel(2) = 0.0;
    Vector3d desired_vel = Robot.com_.vel + desired_accel * abs(Robot.d_time_);
    desired_vel(2) = 0.0;
    Vector3d desired_pos = Robot.com_.pos + desired_vel * abs(Robot.d_time_);
    desired_pos(2) = height;
    Eigen::Vector3d kp_, kd_;
    kp_ << 400, 400, 400;
    kd_ << 40, 40, 40;
    Vector3d fstar = getfstar(Robot, kp_, kd_, desired_pos, Robot.com_.pos, desired_vel, Robot.com_.vel);

    Vector3d fstar_r;
    fstar_r(0) = -ZMP(1) / (Robot.com_.mass * 9.81);
    fstar_r(1) = -ZMP(0) / (Robot.com_.mass * 9.81);
    fstar_r(2) = 0;

    Vector6d r_z;
    r_z.segment(0, 3) = fstar;
    r_z.segment(3, 3) = fstar_r;

    return r_z;
}

VectorQd WholebodyController::gravity_compensation_torque_QP(RobotData &Robot)
{
    VectorQd task_torque;
    static VectorQd torque_before;
    VectorXd f_star_qp_;

    //VectorQd gravity_torque = gravity_compensation_torque(Robot, dc.fixedgravity);
    double friction_ratio = 0.1;
    double friction_ratio_z = 0.01;
    //qptest
    double foot_x_length = 0.12;
    double foot_y_length = 0.05;

    Robot.G.setZero(MODEL_DOF + 6);

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }

    static int contact_dof;
    int constraint_per_contact = 14;
    bool qpt_info = false;

    if ((contact_dof != 6 * Robot.contact_index))
    {
        contact_dof = 6 * Robot.contact_index;
        std::cout << "############################" << std::endl
                  << "QP GC initialize ! " << std::endl
                  << "Contact Dof = " << Robot.contact_index * 6 << std::endl
                  << "Contact Link : ";
        for (int i = 0; i < Robot.contact_index; i++)
        {
            std::cout << Robot.link_[Robot.contact_part[i]].name << "\t";
        }
        std::cout << std::endl
                  << "############################" << std::endl;

        qpt_info = true;
    }

    int variable_size = MODEL_DOF + contact_dof;
    int constraint_size = MODEL_DOF_VIRTUAL + contact_dof + constraint_per_contact * Robot.contact_index;

    //QP initialize!
    QP_torque.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //H.block(0, 0, MODEL_DOF, MODEL_DOF) = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;

    // Ea minimization ::
    double ea_weight = 1.0;
    W = Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T; // + 0.1*Robot.Slc_k * Robot.A_matrix_inverse * Robot.Slc_k_T;
    g.segment(0, MODEL_DOF) = -ea_weight * Robot.Slc_k * Robot.A_matrix_inverse * Robot.N_C * Robot.G;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = ea_weight * W; // + 0.01 * MatrixXd::Identity(MODEL_DOF,MODEL_DOF);

    if (Robot.showdata)
    {
        Robot.showdata = false;
    }

    // contact force minimization
    MatrixXd Fsl;
    Fsl.setZero(contact_dof, contact_dof);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Fsl(6 * i + 0, 6 * i + 0) = 0.0001;
        Fsl(6 * i + 1, 6 * i + 1) = 0.0001;
        Fsl(6 * i + 2, 6 * i + 2) = 0.0001;
        Fsl(6 * i + 3, 6 * i + 3) = 0.001;
        Fsl(6 * i + 4, 6 * i + 4) = 0.001;
        Fsl(6 * i + 5, 6 * i + 5) = 0.00001;
    }

    Eigen::MatrixXd J_g;
    J_g.setZero(MODEL_DOF, MODEL_DOF + 6);
    J_g.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();

    H.block(MODEL_DOF, MODEL_DOF, contact_dof, contact_dof) = Fsl.transpose() * Fsl;
    std::cout << "Dyn Const1" << std::endl;

    MatrixXd Temp1 = Robot.I37 - Robot.J_C.transpose() * Robot.J_C_INV_T;
    std::cout << "Dyn Const1" << std::endl;
    MatrixXd Temp2 = Temp1 * Robot.Slc_k_T;
    std::cout << "Dyn Const1" << std::endl;

    //Rigid Body Dynamcis Equality Constraint
    A.block(0, 0, MODEL_DOF_VIRTUAL, MODEL_DOF) = Temp2;
    std::cout << "Dyn Const2" << std::endl;
    lbA.segment(0, MODEL_DOF_VIRTUAL) = Robot.G - Robot.J_C.transpose() * Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G;
    std::cout << "Dyn Const3" << std::endl;
    ubA.segment(0, MODEL_DOF_VIRTUAL) = Robot.G - Robot.J_C.transpose() * Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G;
    std::cout << "Dyn Const4" << std::endl;
    //Contact Force Equality constraint
    A.block(MODEL_DOF_VIRTUAL, 0, contact_dof, MODEL_DOF) = Robot.J_C_INV_T * Robot.Slc_k_T;
    A.block(MODEL_DOF_VIRTUAL, MODEL_DOF, contact_dof, contact_dof) = -MatrixXd::Identity(contact_dof, contact_dof);
    lbA.segment(MODEL_DOF_VIRTUAL, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;
    ubA.segment(MODEL_DOF_VIRTUAL, contact_dof) = Robot.J_C_INV_T * Robot.G; // - Robot.J_C_INV_T * Robot.Slc_k_T * gravity_torque;

    //std::cout << "calc done!" << std::endl;
    //Contact Force inequality constraint
    for (int i = 0; i < Robot.contact_index; i++)
    {
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 0, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 1, MODEL_DOF + 4 + 6 * i) = 1.0;

        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 2, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 3, MODEL_DOF + 3 + 6 * i) = 1.0;

        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 0 + 6 * i) = 1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 4, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 0 + 6 * i) = -1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 5, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 1 + 6 * i) = 1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 6, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 1 + 6 * i) = -1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 7, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 3 + 6 * i) = 1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 8, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 3 + 6 * i) = -1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 9, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 4 + 6 * i) = 1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 10, MODEL_DOF + 2 + 6 * i) = -friction_ratio;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 4 + 6 * i) = -1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 11, MODEL_DOF + 2 + 6 * i) = -friction_ratio;

        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 12, MODEL_DOF + 5 + 6 * i) = 1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 12, MODEL_DOF + 2 + 6 * i) = -friction_ratio_z;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 13, MODEL_DOF + 5 + 6 * i) = -1.0;
        A(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + 13, MODEL_DOF + 2 + 6 * i) = -friction_ratio_z;

        //May cause error for hand contact!
        for (int j = 0; j < constraint_per_contact; j++)
        {
            //A(task_dof+contact_dof+i*constraint_per_contact+j,)

            A.block(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) = A.block(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
            A.block(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) = A.block(MODEL_DOF_VIRTUAL + contact_dof + i * constraint_per_contact + j, MODEL_DOF + 6 * i + 3, 1, 3) * Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        }
    }

    for (int i = 0; i < constraint_per_contact * Robot.contact_index; i++)
    {
        lbA(MODEL_DOF_VIRTUAL + contact_dof + i) = 0.0;
        ubA(MODEL_DOF_VIRTUAL + contact_dof + i) = 100000.0;
    }

    //std::cout << "calc done!" << std::endl;
    //Torque bound setting
    for (int i = 0; i < MODEL_DOF; i++)
    {
        lb(i) = -300;
        ub(i) = 300;
    }
    for (int i = 0; i < contact_dof; i++)
    {
        lb(MODEL_DOF + i) = -10000;
        ub(MODEL_DOF + i) = 10000;
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        ub(MODEL_DOF + 6 * i + 2) = -20;
        ub(MODEL_DOF + 6 * i + 5) = 10000;
        lb(MODEL_DOF + 6 * i + 5) = -10000;
    }

    //std::cout << "calc done!" << std::endl;
    QP_torque.EnableEqualityCondition(0.0001);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);
    QP_torque.UpdateSubjectToX(lb, ub);
    VectorXd qpres;
    //if()
    if (QP_torque.SolveQPoases(100, qpres))
    {
        task_torque = qpres.segment(0, MODEL_DOF);
    }
    else
    {
        std::cout << "qp solve failed. " << std::endl;
        Robot.task_control_switch = false;
        Robot.contact_redistribution_mode = 0;
        task_torque = torque_before;
    }
    torque_before = task_torque;
    return task_torque;
}

VectorQd WholebodyController::task_control_torque_hqp_step(RobotData &Robot, MatrixXd &J_task, VectorXd &f_star)
{
    int task_dof = f_star.size();
    int contact_index = Robot.contact_index;
    int contact_dof = contact_index * 6 - 6;
    int variable_size = task_dof + contact_dof;
    int constraint_per_contact = 10;
    int constraint_size = contact_index * constraint_per_contact + MODEL_DOF;
    MatrixXd Scf_;
    Scf_.setZero(contact_dof, contact_dof + 6);
    Scf_.block(0, 0, contact_dof, contact_dof).setIdentity();

    Robot.torque_grav = gravity_compensation_torque(Robot);
    Eigen::MatrixXd NwJw = Robot.qr_V2.transpose() * (Scf_ * Robot.J_C_INV_T * Robot.Slc_k_T * Robot.qr_V2.transpose()).inverse();

    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();

    Eigen::MatrixXd Jkt;
    getJkt(Robot, J_task, Jkt);

    Eigen::MatrixXd A;

    A.setZero(constraint_size, variable_size);
    A.block(0, 0, MODEL_DOF, task_dof) = Jkt * Robot.lambda;
    A.block(0, task_dof, MODEL_DOF, contact_dof) = NwJw;

    Eigen::VectorXd lbA, ubA;
    lbA.setZero(MODEL_DOF + Robot.contact_index * constraint_per_contact);
    ubA.setZero(MODEL_DOF + Robot.contact_index * constraint_per_contact);

    lbA.segment(0, MODEL_DOF) = -torque_limit - Jkt * Robot.lambda * f_star - Robot.torque_grav;
    ubA.segment(0, MODEL_DOF) = torque_limit - Jkt * Robot.lambda * f_star - Robot.torque_grav;

    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
    }

    Eigen::MatrixXd Sf;
    Sf.setZero(contact_index, contact_index * 6);
    Sf(0, 2) = 1;
    Sf(1, 8) = 1;

    A.block(MODEL_DOF, 0, Robot.contact_index * constraint_per_contact, task_dof) = Af * Robot.J_C_INV_T * Robot.Slc_k_T * Jkt * Robot.lambda;
    A.block(MODEL_DOF, task_dof, Robot.contact_index * constraint_per_contact, contact_dof) = Af * Robot.J_C_INV_T * Robot.Slc_k_T * NwJw;

    lbA.segment(MODEL_DOF, Robot.contact_index * constraint_per_contact) =
        Af * (Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G - Robot.J_C_INV_T * Robot.Slc_k_T * (Robot.torque_grav + Jkt * Robot.lambda * f_star));
    for (int i = 0; i < contact_index * constraint_per_contact; i++)
        ubA(MODEL_DOF + i) = 1E+6;

    MatrixXd H;
    VectorXd g;

    H.setZero(variable_size, variable_size);
    H.block(0, 0, task_dof, task_dof).setIdentity();
    g.setZero(variable_size);

    if (Robot.init_qp)
    {
        QP_torque.InitializeProblemSize(variable_size, constraint_size);
        Robot.init_qp = false;
    }

    QP_torque.EnableEqualityCondition(0.0001);
    QP_torque.UpdateMinProblem(H, g);
    QP_torque.UpdateSubjectToAx(A, lbA, ubA);

    VectorXd qpres;
    //return Jkt * (f_star) + Robot.torque_grav;
    std::chrono::steady_clock::time_point t_bf = std::chrono::steady_clock::now();
    if (QP_torque.SolveQPoases(100, qpres) == 0)
    {
        return Robot.torque_grav;
    }
    else
    {
        static std::chrono::steady_clock::time_point t_now = std::chrono::steady_clock::now();

        std::chrono::steady_clock::time_point t_st = std::chrono::steady_clock::now();
        //std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t_st - t_now).count() << "calc time : " << std::chrono::duration_cast<std::chrono::microseconds>(t_st - t_start).count() << "qo time : " << std::chrono::duration_cast<std::chrono::microseconds>(t_st - t_bf).count() << " : SOLVED" << qpres.transpose() << std::endl;

        t_now = std::chrono::steady_clock::now();
        return Jkt * Robot.lambda * (f_star + qpres.segment(0, task_dof)) + NwJw * qpres.segment(task_dof, contact_dof) + Robot.torque_grav;
    }
}
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> WholebodyController::pinv_QR(MatrixXd &A)
{
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    //qr.setThreshold(10e-10);
    int rank = qr.rank();

    int cols, rows;

    cols = A.cols();
    rows = A.rows();

    if (rank == 0)
    {
        std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ret(A, A);
        return ret;
    }
    else
    {
        if (cols > rows)
        {
            Eigen::MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
            Eigen::MatrixXd Rpsinv2(rows, cols);

            Rpsinv2.setZero();
            Rpsinv2.topLeftCorner(rank, rank) = R.inverse();

            Eigen::MatrixXd P;
            P = qr.householderQ().transpose();

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ret((qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose(), P.block(rank, 0, P.rows() - rank, P.cols()));
            return ret;
        }
        else
        {
            Eigen::MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
            Eigen::MatrixXd Rpsinv2(cols, rows);
            Rpsinv2.setZero();
            Rpsinv2.topLeftCorner(rank, rank) = R.inverse();

            Eigen::MatrixXd P;
            P = qr.householderQ().transpose();

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ret((qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose(), P.block(rank, 0, P.rows() - rank, P.cols()));
            return ret;
        }
    }
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> WholebodyController::getjkt_m(MatrixVVd &Amat_inv, MatrixXd &Nc, MatrixXd &Winv, MatrixXd &Jtask)
{
    MatrixXd linv = Jtask * Amat_inv * Nc * Jtask.transpose();
    MatrixXd lambda = linv.inverse();
    MatrixXd Q_ = lambda * Jtask * Amat_inv * Nc.rightCols(MODEL_DOF);
    MatrixXd Q_temp, Q_temp_inv;
    Q_temp = Q_ * Winv * Q_.transpose();
    Q_temp_inv = DyrosMath::pinv_QR(Q_temp);
    std::pair<MatrixXd, MatrixXd> ret(Winv * Q_.transpose() * Q_temp_inv, lambda);

    return ret;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> WholebodyController::getjkt_t(RobotData &Robot, MatrixXd &Jtask)
{
    MatrixXd linv = Jtask * Robot.A_matrix_inverse * Robot.N_C * Jtask.transpose();
    MatrixXd lambda = linv.inverse();
    MatrixXd Q_ = lambda * Jtask * Robot.A_matrix_inverse * Robot.N_C.rightCols(MODEL_DOF);
    MatrixXd Q_temp, Q_temp_inv;

    Q_temp = Q_ * Robot.W_inv * Q_.transpose();
    Q_temp_inv = DyrosMath::pinv_QR(Q_temp);
    std::pair<MatrixXd, MatrixXd> ret(Robot.W_inv * Q_.transpose() * Q_temp_inv, lambda);

    return ret;
}

// VectorXd WholebodyController::hqp_step_contact_calc(CQuadraticProgram &qphqp, RobotData &Robot, VectorXd torque_prev, MatrixXd &Null_task, MatrixXd &Jkt, MatrixXd &lambda, VectorXd f_star, bool init)
// {

// }

VectorXd WholebodyController::hqp_contact_calc(CQuadraticProgram &qphqp, RobotData_fast &Robot_fast, VectorXd torque_prev, bool init)
{
    //return fstar & contact force;

    // std::chrono::steady_clock::time_point t[11];

    // t[0] = std::chrono::steady_clock::now();
    int contact_index = Robot_fast.contact_index;
    int contact_dof = contact_index * 6 - 6;
    int variable_size = contact_dof;
    int constraint_per_contact = 11;
    int constraint_size = contact_index * constraint_per_contact + MODEL_DOF;

    Eigen::MatrixXd A;
    Eigen::VectorXd lbA, ubA;
    A.setZero(constraint_size, variable_size);
    lbA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    ubA.setZero(MODEL_DOF + contact_index * constraint_per_contact);

    //Torque Limit constraint
    //Eigen::MatrixXd Ntorque_task = Null_task * Jkt * lambda;
    //A.block(0, 0, MODEL_DOF, task_dof) = Ntorque_task;
    A.block(0, 0, MODEL_DOF, contact_dof) = Robot_fast.NwJw;

    lbA.segment(0, MODEL_DOF) = -torque_limit - torque_prev - Robot_fast.torque_grav;
    ubA.segment(0, MODEL_DOF) = torque_limit - torque_prev - Robot_fast.torque_grav;
    // t[1] = std::chrono::steady_clock::now();

    Eigen::MatrixXd Rf;
    Rf.setZero(contact_index * 6, contact_index * 6);
    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        Rf.block(0 + i * 6, 0 + i * 6, 3, 3) = Robot_fast.ee_[Robot_fast.ee_idx[i]].rotm.inverse();
        Rf.block(3 + i * 6, 3 + i * 6, 3, 3) = Robot_fast.ee_[Robot_fast.ee_idx[i]].rotm.inverse();

        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio_z;

        Af(i * constraint_per_contact + 10, 2 + 6 * i) = 1.0;
    }

    // Eigen::MatrixXd Sf;
    // Sf.setZero(contact_index, contact_index * 6);
    // Sf(0, 2) = 1;s
    // Sf(1, 8) = 1;
    // t[2] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Atemp;
    Atemp = Af * Rf * Robot_fast.J_C_INV_T.rightCols(MODEL_DOF);
    // t[3] = std::chrono::steady_clock::now();
    //A.block(MODEL_DOF, 0, Robot.contact_index * constraint_per_contact, task_dof) = Atemp * Ntorque_task;
    A.block(MODEL_DOF, 0, contact_index * constraint_per_contact, contact_dof) = Atemp * Robot_fast.NwJw;
    // t[4] = std::chrono::steady_clock::now();

    lbA.segment(MODEL_DOF, contact_index * constraint_per_contact) =
        Af * Rf * (Robot_fast.P_C - Robot_fast.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot_fast.torque_grav));
    ubA.segment(MODEL_DOF, contact_index * constraint_per_contact) =
        Af * Rf * (Robot_fast.P_C - Robot_fast.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot_fast.torque_grav));

    for (int i = 0; i < contact_index; i++)
    {
        for (int j = 0; j < constraint_per_contact - 1; j++)
            ubA(MODEL_DOF + i * constraint_per_contact + j) += 1E+6;

        ubA(MODEL_DOF + i * constraint_per_contact + 10) += -100;
        lbA(MODEL_DOF + i * constraint_per_contact + 10) += -1E+6;
    }

    MatrixXd H;
    VectorXd g;

    H.setZero(variable_size, variable_size);
    MatrixXd S_contact(contact_index * 6, contact_index * 6);
    S_contact.setIdentity();
    for (int i = 0; i < contact_index; i++)
        S_contact(i * 6 + 2, i * 6 + 2) = 0.0;
    H = Robot_fast.NwJw.transpose() * Robot_fast.J_C_INV_T.rightCols(MODEL_DOF).transpose() * S_contact * Robot_fast.J_C_INV_T.rightCols(MODEL_DOF) * Robot_fast.NwJw;
    g.setZero(variable_size);
    g = Robot_fast.NwJw.transpose() * (Robot_fast.J_C_INV_T.rightCols(MODEL_DOF).transpose() * (S_contact * (Robot_fast.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot_fast.torque_grav) - Robot_fast.P_C)));
    // t[7] = std::chrono::steady_clock::now();

    if (init)
    {
        qphqp.InitializeProblemSize(variable_size, constraint_size);
    }

    qphqp.EnableEqualityCondition(0.0001);
    qphqp.UpdateMinProblem(H, g);
    qphqp.UpdateSubjectToAx(A, lbA, ubA);

    VectorXd qpres;

    if (qphqp.SolveQPoases(100, qpres))
    {
        return qpres;
    }
    else
    {
        return VectorXd::Zero(variable_size);
    }
}

std::pair<VectorXd, VectorXd> WholebodyController::hqp_step_calc(CQuadraticProgram &qphqp, RobotData_fast &Robot_fast, VectorXd torque_prev, MatrixXd &Null_task, MatrixXd &Jkt, MatrixXd &lambda, VectorXd f_star, bool init)
{
    //return fstar & contact force;
    // std::chrono::steady_clock::time_point t[11];
    // t[0] = std::chrono::steady_clock::now();
    int task_dof = f_star.size();
    int contact_index = Robot_fast.contact_index;
    int contact_dof = contact_index * 6 - 6;
    int variable_size = task_dof + contact_dof;
    int constraint_per_contact = 11;
    int constraint_size = contact_index * constraint_per_contact + MODEL_DOF;
    Eigen::MatrixXd A;
    Eigen::MatrixXd Ntorque_task = Null_task * Jkt * lambda;
    A.setZero(constraint_size, variable_size);
    A.block(0, 0, MODEL_DOF, task_dof) = Ntorque_task;
    A.block(0, task_dof, MODEL_DOF, contact_dof) = Robot_fast.NwJw;
    Eigen::VectorXd lbA, ubA;
    lbA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    ubA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    lbA.segment(0, MODEL_DOF) = -torque_limit - torque_prev - Ntorque_task * f_star - Robot_fast.torque_grav;
    ubA.segment(0, MODEL_DOF) = torque_limit - torque_prev - Ntorque_task * f_star - Robot_fast.torque_grav;
    // t[1] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);

    Eigen::MatrixXd Rf;
    Rf.setZero(contact_index * 6, contact_index * 6);

    for (int i = 0; i < contact_index; i++)
    {

        Rf.block(0 + i * 6, 0 + i * 6, 3, 3) = Robot_fast.ee_[Robot_fast.ee_idx[i]].rotm.inverse();
        Rf.block(3 + i * 6, 3 + i * 6, 3, 3) = Robot_fast.ee_[Robot_fast.ee_idx[i]].rotm.inverse();

        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio_z;

        Af(i * constraint_per_contact + 10, 2 + 6 * i) = 1.0;
    }
    // Eigen::MatrixXd Sf;
    // Sf.setZero(contact_index, contact_index * 6);
    // Sf(0, 2) = 1;s
    // Sf(1, 8) = 1;
    // t[2] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Atemp;
    Atemp = Af * Rf * Robot_fast.J_C_INV_T.rightCols(MODEL_DOF);
    // t[3] = std::chrono::steady_clock::now();
    A.block(MODEL_DOF, 0, contact_index * constraint_per_contact, task_dof) = Atemp * Ntorque_task;
    A.block(MODEL_DOF, task_dof, contact_index * constraint_per_contact, contact_dof) = Atemp * Robot_fast.NwJw;
    // t[4] = std::chrono::steady_clock::now();
    lbA.segment(MODEL_DOF, contact_index * constraint_per_contact) =
        Af * Rf * (Robot_fast.P_C - Robot_fast.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot_fast.torque_grav + Ntorque_task * f_star));

    ubA.segment(MODEL_DOF, contact_index * constraint_per_contact) =
        Af * Rf * (Robot_fast.P_C - Robot_fast.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot_fast.torque_grav + Ntorque_task * f_star));

    // t[5] = std::chrono::steady_clock::now();
    for (int i = 0; i < contact_index; i++)
    {
        for (int j = 0; j < constraint_per_contact - 1; j++)
            ubA(MODEL_DOF + i * constraint_per_contact + j) += 1E+6;

        ubA(MODEL_DOF + i * constraint_per_contact + 10) += -100;
        lbA(MODEL_DOF + i * constraint_per_contact + 10) += -1E+6;
    }

    // t[6] = std::chrono::steady_clock::now();
    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    H.block(0, 0, task_dof, task_dof).setIdentity();
    g.setZero(variable_size);
    // t[7] = std::chrono::steady_clock::now();
    if (init)
    {
        qphqp.InitializeProblemSize(variable_size, constraint_size);
    }
    qphqp.EnableEqualityCondition(0.0001);
    qphqp.UpdateMinProblem(H, g);
    qphqp.UpdateSubjectToAx(A, lbA, ubA);
    // t[8] = std::chrono::steady_clock::now();
    VectorXd qpres;
    // t[9] = std::chrono::steady_clock::now();
    if (qphqp.SolveQPoases(100, qpres))
    {
        std::pair<VectorXd, VectorXd> ret(qpres.segment(0, task_dof), qpres.segment(task_dof, contact_dof));
        // t[10] = std::chrono::steady_clock::now();
        // std::cout << "hqp qp prepare : ";
        // for (int i = 0; i < 10; i++)
        // {
        //     std::cout << i << " : " << std::chrono::duration_cast<std::chrono::microseconds>(t[i + 1] - t[i]).count() << "\t";
        // }
        // std::cout << "total : " << std::chrono::duration_cast<std::chrono::microseconds>(t[10] - t[0]).count() << "\t";
        // std::cout << std::endl;
        return ret;
    }
    else
    {
        std::pair<VectorXd, VectorXd> ret(VectorXd::Zero(task_dof), VectorXd::Zero(contact_dof));
        return ret;
    }
}

VectorXd WholebodyController::hqp_damping_calc(CQuadraticProgram &qphqp, RobotData_fast &Robot_fast, VectorXd torque_prev, MatrixXd &Null_task, bool init)
{
    //std::cout<<"dc1"<<std::endl;
    int contact_index = Robot_fast.contact_index;
    int contact_dof = contact_index * 6 - 6;
    int variable_size = MODEL_DOF + contact_dof;
    int constraint_per_contact = 10;
    int constraint_size = contact_index * constraint_per_contact + MODEL_DOF;
    Eigen::MatrixXd A;
    Eigen::VectorXd lbA, ubA;
    A.setZero(constraint_size, variable_size);
    lbA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    ubA.setZero(MODEL_DOF + contact_index * constraint_per_contact);

    Eigen::VectorXd torque_damping = -Robot_fast.q_dot_ * 0.1;
    //std::cout<<"dc1"<<std::endl;
    //Torque Limit constraint
    //Eigen::MatrixXd Ntorque_task = Null_task * Jkt * lambda;
    //std::cout<<"dc1"<<std::endl;
    A.block(0, 0, MODEL_DOF, MODEL_DOF) = Null_task;
    //std::cout<<"dc1"<<std::endl;
    A.block(0, MODEL_DOF, MODEL_DOF, contact_dof) = Robot_fast.NwJw;
    lbA.segment(0, MODEL_DOF) = -torque_limit - torque_prev - Robot_fast.torque_grav - Null_task * torque_damping;
    ubA.segment(0, MODEL_DOF) = torque_limit - torque_prev - Robot_fast.torque_grav - Null_task * torque_damping;
    //std::cout<<"dc1"<<std::endl;
    // t[1] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot_fast.ee_[Robot_fast.ee_idx[i]].friction_ratio_z;
    }
    // Eigen::MatrixXd Sf;
    // Sf.setZero(contact_index, contact_index * 6);
    // Sf(0, 2) = 1;s
    // Sf(1, 8) = 1;
    // t[2] = std::chrono::steady_clock::now();
    //std::cout<<"dc1"<<std::endl;
    Eigen::MatrixXd Atemp;
    Atemp = Af * Robot_fast.J_C_INV_T.rightCols(MODEL_DOF);
    // t[3] = std::chrono::steady_clock::now();
    A.block(MODEL_DOF, 0, Robot_fast.contact_index * constraint_per_contact, MODEL_DOF) = Atemp * Null_task;
    A.block(MODEL_DOF, MODEL_DOF, contact_index * constraint_per_contact, contact_dof) = Atemp * Robot_fast.NwJw;
    // t[4] = std::chrono::steady_clock::now();
    lbA.segment(MODEL_DOF, contact_index * constraint_per_contact) =
        Af * (Robot_fast.P_C - Robot_fast.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot_fast.torque_grav + Null_task * torque_damping));
    // t[5] = std::chrono::steady_clock::now();
    for (int i = 0; i < contact_index * constraint_per_contact; i++)
        ubA(MODEL_DOF + i) = 1E+6;
    // t[6] = std::chrono::steady_clock::now();
    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    g.setZero(variable_size);

    //std::cout<<"dc1"<<std::endl;
    H.block(0, 0, MODEL_DOF, MODEL_DOF) = MatrixXd::Identity(MODEL_DOF, MODEL_DOF);
    //H = Robot.NwJw.transpose() * Robot.J_C_INV_T.rightCols(MODEL_DOF).transpose() * S_contact * Robot.J_C_INV_T.rightCols(MODEL_DOF) * Robot.NwJw;
    //g.segment(0, MODEL_DOF) = -Robot_fast.q_dot_ * 0.1;
    //g = Robot.NwJw.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).transpose() * (S_contact * (Robot.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot.torque_grav) - Robot.P_C)));
    // t[7] = std::chrono::steady_clock::now();
    //std::cout<<"dc1"<<std::endl;
    lbA = lbA.array() - 1.0E-6;
    ubA = ubA.array() + 1.0E-6;

    if (init)
    {
        qphqp.InitializeProblemSize(variable_size, constraint_size);
    }
    qphqp.EnableEqualityCondition(0.0001);
    qphqp.UpdateMinProblem(H, g);
    qphqp.UpdateSubjectToAx(A, lbA, ubA);

    VectorXd qpres;
    if (qphqp.SolveQPoases(100, qpres))
    {

        return qpres.segment(0, MODEL_DOF);
    }
    else
    {
        return VectorXd::Zero(variable_size);
    }
}

VectorXd WholebodyController::hqp_contact_calc(CQuadraticProgram &qphqp, RobotData &Robot, VectorXd torque_prev, bool init)
{
    //return fstar & contact force;
    // std::chrono::steady_clock::time_point t[11];
    // t[0] = std::chrono::steady_clock::now();
    int contact_index = Robot.contact_index;
    int contact_dof = contact_index * 6 - 6;
    int variable_size = contact_dof;
    int constraint_per_contact = 10;
    int constraint_size = contact_index * constraint_per_contact + MODEL_DOF;
    Eigen::MatrixXd A;
    Eigen::VectorXd lbA, ubA;
    A.setZero(constraint_size, variable_size);
    lbA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    ubA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    //Torque Limit constraint
    //Eigen::MatrixXd Ntorque_task = Null_task * Jkt * lambda;
    //A.block(0, 0, MODEL_DOF, task_dof) = Ntorque_task;
    A.block(0, 0, MODEL_DOF, contact_dof) = Robot.NwJw;
    lbA.segment(0, MODEL_DOF) = -Robot.torque_limit - torque_prev - Robot.torque_grav;
    ubA.segment(0, MODEL_DOF) = Robot.torque_limit - torque_prev - Robot.torque_grav;
    // t[1] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
    }
    // Eigen::MatrixXd Sf;
    // Sf.setZero(contact_index, contact_index * 6);
    // Sf(0, 2) = 1;s
    // Sf(1, 8) = 1;
    // t[2] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Atemp;
    Atemp = Af * Robot.J_C_INV_T.rightCols(MODEL_DOF);
    // t[3] = std::chrono::steady_clock::now();
    //A.block(MODEL_DOF, 0, Robot.contact_index * constraint_per_contact, task_dof) = Atemp * Ntorque_task;
    A.block(MODEL_DOF, 0, contact_index * constraint_per_contact, contact_dof) = Atemp * Robot.NwJw;
    // t[4] = std::chrono::steady_clock::now();
    lbA.segment(MODEL_DOF, contact_index * constraint_per_contact) =
        Af * (Robot.P_C - Robot.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot.torque_grav));
    // t[5] = std::chrono::steady_clock::now();
    for (int i = 0; i < contact_index * constraint_per_contact; i++)
        ubA(MODEL_DOF + i) = 1E+6;
    // t[6] = std::chrono::steady_clock::now();
    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    MatrixXd S_contact(contact_index * 6, contact_index * 6);
    S_contact.setIdentity();
    for (int i = 0; i < contact_index; i++)
        S_contact(i * 6 + 2, i * 6 + 2) = 0.0;
    H = Robot.NwJw.transpose() * Robot.J_C_INV_T.rightCols(MODEL_DOF).transpose() * S_contact * Robot.J_C_INV_T.rightCols(MODEL_DOF) * Robot.NwJw;
    g.setZero(variable_size);
    g = Robot.NwJw.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).transpose() * (S_contact * (Robot.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot.torque_grav) - Robot.P_C)));
    // t[7] = std::chrono::steady_clock::now();

    lbA = lbA.array() - 1.0E-6;
    ubA = ubA.array() + 1.0E-6;

    if (init)
    {
        qphqp.InitializeProblemSize(variable_size, constraint_size);
    }
    qphqp.EnableEqualityCondition(0.0001);
    qphqp.UpdateMinProblem(H, g);
    qphqp.UpdateSubjectToAx(A, lbA, ubA);
    VectorXd qpres;
    if (qphqp.SolveQPoases(100, qpres))
    {
        return qpres;
    }
    else
    {
        Robot.qp_error = true;
    }
}

std::pair<VectorXd, VectorXd> WholebodyController::hqp_step_calc(CQuadraticProgram &qphqp, RobotData &Robot, VectorXd torque_prev, MatrixXd &Null_task, MatrixXd &Jkt, MatrixXd &lambda, VectorXd f_star, bool init)
{
    //return fstar & contact force;
    // std::chrono::steady_clock::time_point t[11];
    // t[0] = std::chrono::steady_clock::now();
    int task_dof = f_star.size();
    int contact_index = Robot.contact_index;
    int contact_dof = contact_index * 6 - 6;
    int variable_size = task_dof + contact_dof;
    int constraint_per_contact = 10;
    int constraint_size = contact_index * constraint_per_contact + MODEL_DOF;
    Eigen::MatrixXd A;
    Eigen::MatrixXd Ntorque_task = Null_task * Jkt * lambda;
    A.setZero(constraint_size, variable_size);
    A.block(0, 0, MODEL_DOF, task_dof) = Ntorque_task;
    A.block(0, task_dof, MODEL_DOF, contact_dof) = Robot.NwJw;
    Eigen::VectorXd lbA, ubA;
    lbA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    ubA.setZero(MODEL_DOF + contact_index * constraint_per_contact);
    lbA.segment(0, MODEL_DOF) = -Robot.torque_limit - torque_prev - Ntorque_task * f_star - Robot.torque_grav;
    ubA.segment(0, MODEL_DOF) = Robot.torque_limit - torque_prev - Ntorque_task * f_star - Robot.torque_grav;
    // t[1] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {
        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
    }

    // Eigen::MatrixXd Sf;
    // Sf.setZero(contact_index, contact_index * 6);
    // Sf(0, 2) = 1;s
    // Sf(1, 8) = 1;
    // t[2] = std::chrono::steady_clock::now();
    Eigen::MatrixXd Atemp;
    Atemp = Af * Robot.J_C_INV_T.rightCols(MODEL_DOF);
    // t[3] = std::chrono::steady_clock::now();
    A.block(MODEL_DOF, 0, contact_index * constraint_per_contact, task_dof) = Atemp * Ntorque_task;
    A.block(MODEL_DOF, task_dof, contact_index * constraint_per_contact, contact_dof) = Atemp * Robot.NwJw;
    // t[4] = std::chrono::steady_clock::now();
    lbA.segment(MODEL_DOF, contact_index * constraint_per_contact) =
        Af * (Robot.P_C - Robot.J_C_INV_T.rightCols(MODEL_DOF) * (torque_prev + Robot.torque_grav + Ntorque_task * f_star));
    // t[5] = std::chrono::steady_clock::now();
    for (int i = 0; i < contact_index * constraint_per_contact; i++)
        ubA(MODEL_DOF + i) = 1E+6;
    // t[6] = std::chrono::steady_clock::now();
    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    H.block(0, 0, task_dof, task_dof).setIdentity();
    g.setZero(variable_size);
    // t[7] = std::chrono::steady_clock::now();
    if (init)
    {
        qphqp.InitializeProblemSize(variable_size, constraint_size);
    }
    qphqp.EnableEqualityCondition(0.0001);
    qphqp.UpdateMinProblem(H, g);
    qphqp.UpdateSubjectToAx(A, lbA, ubA);
    // t[8] = std::chrono::steady_clock::now();
    VectorXd qpres;
    // t[9] = std::chrono::steady_clock::now();
    if (qphqp.SolveQPoases(100, qpres))
    {
        std::pair<VectorXd, VectorXd> ret(qpres.segment(0, task_dof), qpres.segment(task_dof, contact_dof));
        // t[10] = std::chrono::steady_clock::now();
        // std::cout << "hqp qp prepare : ";
        // for (int i = 0; i < 10; i++)
        // {
        //     std::cout << i << " : " << std::chrono::duration_cast<std::chrono::microseconds>(t[i + 1] - t[i]).count() << "\t";
        // }
        // std::cout << "total : " << std::chrono::duration_cast<std::chrono::microseconds>(t[10] - t[0]).count() << "\t";
        // std::cout << std::endl;
        return ret;
    }
    else
    {
        Robot.qp_error = true;
    }
}

std::pair<VectorXd, VectorXd> WholebodyController::hqp2_step_calc(CQuadraticProgram &qphqp, RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp, bool init)
{
    int heirarchy = Jtask_hqp.size();
    std::vector<int> task_dof;
    for (int i = 0; i < heirarchy; i++)
    {
        task_dof.push_back(fstar_hqp[i].size());
    }

    int contact_index = Robot.contact_index;
    int contact_dof = contact_index * 6;
    //qacc torque contact_force fstar_slack
    int variable_size = MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof + task_dof.back();
    //qddot torque contactforce slack
    int constraint_per_contact = 11;
    int total_td = 0;
    for (int i = 0; i < task_dof.size(); i++)
    {
        total_td += task_dof[i];
    }
    int constraint_size = MODEL_DOF_VIRTUAL + total_td + contact_dof + contact_index * constraint_per_contact + MODEL_DOF;

    Eigen::MatrixXd A;
    A.setZero(constraint_size, variable_size);
    Eigen::VectorXd lbA, ubA;
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //A*qddot + h + Jc^T * F_c = S^T * torque
    A.block(0, 0, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = Robot.A_matrix;
    A.block(0, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL, MODEL_DOF) = -Robot.Slc_k_T;
    A.block(0, MODEL_DOF_VIRTUAL + MODEL_DOF, MODEL_DOF_VIRTUAL, contact_dof) = Robot.J_C.transpose();

    lbA.segment(0, MODEL_DOF_VIRTUAL) = -Robot.G;
    ubA.segment(0, MODEL_DOF_VIRTUAL) = -Robot.G;

    //std::cout<<"t1"<<std::endl;
    //J1 * qddot + j1dot * qdot = fstar + w1

    Eigen::MatrixXd Itask;
    for (int i = 0; i < heirarchy; i++)
    {
        Itask.setIdentity(task_dof[i], task_dof[i]);
        int td_start = 0;
        for (int j = 0; j < i; j++)
        {
            td_start += task_dof[j];
        }
        //std::cout<<"tdstart = "<<td_start<<std::endl;
        A.block(MODEL_DOF_VIRTUAL + td_start, 0, task_dof[i], MODEL_DOF_VIRTUAL) = Jtask_hqp[i];

        if ((i + 1) == heirarchy)
            A.block(MODEL_DOF_VIRTUAL + td_start, MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof, task_dof[i], task_dof[i]) = -Itask;

        lbA.segment(MODEL_DOF_VIRTUAL + td_start, task_dof[i]) = fstar_hqp[i];
        ubA.segment(MODEL_DOF_VIRTUAL + td_start, task_dof[i]) = fstar_hqp[i];
    }
    //std::cout<<"t1"<<std::endl;

    //Jc * qddot + Jc * qdot = 0

    int contact_space_constraint = MODEL_DOF_VIRTUAL + total_td;

    A.block(contact_space_constraint, 0, contact_dof, MODEL_DOF_VIRTUAL) = Robot.J_C;

    //contact constraint;
    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);
    Eigen::MatrixXd Rf;
    Rf.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {

        Rf.block(0 + i * 6, 0 + i * 6, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.inverse();
        Rf.block(3 + i * 6, 3 + i * 6, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.inverse();

        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;

        Af(i * constraint_per_contact + 10, 2 + 6 * i) = 1.0;
    }

    //std::cout<<"t1"<<std::endl;
    Eigen::MatrixXd Atemp;
    Atemp = Af * Rf; // * Robot.J_C_INV_T.rightCols(MODEL_DOF);
    int contact_index_start = contact_space_constraint + contact_dof;
    // t[3] = std::chrono::steady_clock::now();
    A.block(contact_index_start, MODEL_DOF_VIRTUAL + MODEL_DOF, contact_index * constraint_per_contact, contact_dof) = Atemp;

    // t[4] = std::chrono::steady_clock::now();
    // lbA.segment(contact_index_start, contact_index * constraint_per_contact) =
    //     Af * Rf * (Robot.P_C);

    // ubA.segment(contact_index_start, contact_index * constraint_per_contact) =
    //     Af * Rf * (Robot.P_C);

    // t[5] = std::chrono::steady_clock::now();
    for (int i = 0; i < contact_index; i++)
    {
        for (int j = 0; j < constraint_per_contact - 1; j++)
            ubA(contact_index_start + i * constraint_per_contact + j) += 1E+6;

        ubA(contact_index_start + i * constraint_per_contact + 10) += -100;
        lbA(contact_index_start + i * constraint_per_contact + 10) += -1E+6;
    }

    //std::cout<<"t1"<<std::endl;
    int tlimit_index = contact_index_start + constraint_per_contact * contact_index;

    A.block(tlimit_index, MODEL_DOF_VIRTUAL, MODEL_DOF, MODEL_DOF) = MatrixQQd::Identity();

    ubA.segment(tlimit_index, MODEL_DOF) = torque_limit;
    lbA.segment(tlimit_index, MODEL_DOF) = -torque_limit;

    lbA = lbA.array() - 1.0E-6;
    ubA = ubA.array() + 1.0E-6;

    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    g.setZero(variable_size);

    H = MatrixXd::Identity(variable_size, variable_size) * 1E-4;

    //std::cout<<"dc1"<<std::endl;
    H.block(MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof, MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof, task_dof.back(), task_dof.back()).setIdentity(); // = MatrixXd::Identity(task_dof, task_dof);

    //H.block(0,0 , MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL).setIdentity();

    //std::cout<<"t1"<<std::endl;
    if (init)
    {
        qphqp.InitializeProblemSize(variable_size, constraint_size);
    }
    qphqp.EnableEqualityCondition(0.0001);
    qphqp.UpdateMinProblem(H, g);
    qphqp.UpdateSubjectToAx(A, lbA, ubA);

    //std::cout<<"t1"<<std::endl;
    VectorXd qpres;
    if (qphqp.SolveQPoases(100, qpres))
    {
        std::pair<VectorXd, VectorXd> ret(qpres.segment(MODEL_DOF_VIRTUAL, MODEL_DOF), qpres.segment(MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof, task_dof.back()));
        return ret;
    }
    else
    {
        std::pair<VectorXd, VectorXd> ret(VectorXd::Zero(MODEL_DOF), VectorXd::Zero(task_dof.back()));
        return ret;
    }
}

VectorQd WholebodyController::hqp2_contact_calc(CQuadraticProgram &qphqp, RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp, bool init)
{

    int heirarchy = Jtask_hqp.size();
    std::vector<int> task_dof;
    for (int i = 0; i < heirarchy; i++)
    {
        task_dof.push_back(fstar_hqp[i].size());
    }

    //std::cout << "h : " << heirarchy << " fstar size : " << fstar_hqp.size() << std::endl;

    int contact_index = Robot.contact_index;
    int contact_dof = contact_index * 6;
    //qacc torque contact_force fstar_slack
    int variable_size = MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof;
    //qddot torque contactforce slack
    int constraint_per_contact = 11;
    int total_td = 0;
    for (int i = 0; i < task_dof.size(); i++)
    {
        total_td += task_dof[i];
    }

    //std::cout << "total dof : " << total_td << std::endl;

    int constraint_size = MODEL_DOF_VIRTUAL + total_td + contact_dof + contact_index * constraint_per_contact + MODEL_DOF;

    Eigen::MatrixXd A;
    A.setZero(constraint_size, variable_size);
    Eigen::VectorXd lbA, ubA;
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    //A*qddot + h + Jc^T * F_c = S^T * torque
    A.block(0, 0, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = Robot.A_matrix;
    A.block(0, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL, MODEL_DOF) = -Robot.Slc_k_T;
    A.block(0, MODEL_DOF_VIRTUAL + MODEL_DOF, MODEL_DOF_VIRTUAL, contact_dof) = Robot.J_C.transpose();

    lbA.segment(0, MODEL_DOF_VIRTUAL) = -Robot.G;
    ubA.segment(0, MODEL_DOF_VIRTUAL) = -Robot.G;

    //std::cout<<"t1"<<std::endl;
    //J1 * qddot + j1dot * qdot = fstar + w1

    for (int i = 0; i < heirarchy; i++)
    {
        int td_start = 0;
        for (int j = 0; j < i; j++)
        {
            td_start += task_dof[j];
        }
        //std::cout<<"tdstart = "<<td_start<<std::endl;
        A.block(MODEL_DOF_VIRTUAL + td_start, 0, task_dof[i], MODEL_DOF_VIRTUAL) = Jtask_hqp[i];

        //if ((i + 1) == heirarchy)
        //A.block(MODEL_DOF_VIRTUAL + td_start, MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof, task_dof[i], task_dof[i]) = -Itask;

        lbA.segment(MODEL_DOF_VIRTUAL + td_start, task_dof[i]) = fstar_hqp[i];
        ubA.segment(MODEL_DOF_VIRTUAL + td_start, task_dof[i]) = fstar_hqp[i];

        //std::cout << "task stacking ...  : " << MODEL_DOF_VIRTUAL + td_start << " with " << task_dof[i] << "dof , size of fstar : "<<fstar_hqp[i].size() << std::endl;
    }
    //std::cout<<"t1"<<std::endl;

    //Jc * qddot + Jc * qdot = 0

    int contact_space_constraint = MODEL_DOF_VIRTUAL + total_td;
    //     std::cout << "contact_space_constraint index start ... ...  : "  <<contact_space_constraint<< std::endl;

    A.block(contact_space_constraint, 0, contact_dof, MODEL_DOF_VIRTUAL) = Robot.J_C;

    //contact constraint;
    Eigen::MatrixXd Af;
    Af.setZero(contact_index * constraint_per_contact, contact_index * 6);
    Eigen::MatrixXd Rf;
    Rf.setZero(contact_index * 6, contact_index * 6);
    for (int i = 0; i < contact_index; i++)
    {

        Rf.block(0 + i * 6, 0 + i * 6, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();
        Rf.block(3 + i * 6, 3 + i * 6, 3, 3) = Robot.ee_[Robot.ee_idx[i]].rotm.transpose();

        Af(i * constraint_per_contact + 0, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 0, 4 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 1, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_x_length;
        Af(i * constraint_per_contact + 1, 4 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 2, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 2, 3 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 3, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].cs_y_length;
        Af(i * constraint_per_contact + 3, 3 + 6 * i) = 1.0;

        Af(i * constraint_per_contact + 4, 0 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 4, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 5, 0 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 5, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 6, 1 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 6, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;
        Af(i * constraint_per_contact + 7, 1 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 7, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio;

        Af(i * constraint_per_contact + 8, 5 + 6 * i) = 1.0;
        Af(i * constraint_per_contact + 8, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;
        Af(i * constraint_per_contact + 9, 5 + 6 * i) = -1.0;
        Af(i * constraint_per_contact + 9, 2 + 6 * i) = -Robot.ee_[Robot.ee_idx[i]].friction_ratio_z;

        Af(i * constraint_per_contact + 10, 2 + 6 * i) = 1.0;
    }

    //std::cout<<"t1"<<std::endl;
    Eigen::MatrixXd Atemp;
    Atemp = Af * Rf; // * Robot.J_C_INV_T.rightCols(MODEL_DOF);
    int contact_index_start = contact_space_constraint + contact_dof;
    // t[3] = std::chrono::steady_clock::now();
    A.block(contact_index_start, MODEL_DOF_VIRTUAL + MODEL_DOF, contact_index * constraint_per_contact, contact_dof) = Atemp;
    //std::cout << "contact index start ... ...  : "  <<contact_index_start<< std::endl;

    // t[4] = std::chrono::steady_clock::now();
    // lbA.segment(contact_index_start, contact_index * constraint_per_contact) =
    //     Af * Rf * (Robot.P_C);

    // ubA.segment(contact_index_start, contact_index * constraint_per_contact) =
    //     Af * Rf * (Robot.P_C);

    // t[5] = std::chrono::steady_clock::now();
    for (int i = 0; i < contact_index; i++)
    {
        for (int j = 0; j < constraint_per_contact - 1; j++)
            ubA(contact_index_start + i * constraint_per_contact + j) += 1E+6;

        ubA(contact_index_start + i * constraint_per_contact + 10) += -100;
        lbA(contact_index_start + i * constraint_per_contact + 10) += -1E+6;
    }

    //std::cout<<"t1"<<std::endl;
    int tlimit_index = contact_index_start + constraint_per_contact * contact_index;

    A.block(tlimit_index, MODEL_DOF_VIRTUAL, MODEL_DOF, MODEL_DOF) = MatrixQQd::Identity();
    //std::cout << "tlimit_index start ... ...  : "  <<tlimit_index<< std::endl;
    ubA.segment(tlimit_index, MODEL_DOF) = torque_limit;
    lbA.segment(tlimit_index, MODEL_DOF) = -torque_limit;

    lbA = lbA.array() - 1E-6;
    ubA = ubA.array() + 1E-6;

    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    g.setZero(variable_size);

    H = MatrixXd::Identity(variable_size, variable_size) * 1E-4;

    //std::cout<<"dc1"<<std::endl;
    H.block(MODEL_DOF_VIRTUAL + MODEL_DOF, MODEL_DOF_VIRTUAL + MODEL_DOF, contact_dof, contact_dof).setIdentity(); // = MatrixXd::Identity(task_dof, task_dof);

    for (int i = 0; i < contact_index; i++)
        H(MODEL_DOF_VIRTUAL + MODEL_DOF + i * 6 + 2, MODEL_DOF_VIRTUAL + MODEL_DOF + i * 6 + 2) = 1E-4;
    //H.block(0,0 , MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL).setIdentity();

    //std::cout<<"t1"<<std::endl;
    if (init)
    {
        qphqp.InitializeProblemSize(variable_size, constraint_size);
    }
    qphqp.EnableEqualityCondition(1E-5);
    qphqp.UpdateMinProblem(H, g);
    qphqp.UpdateSubjectToAx(A, lbA, ubA);

    //std::cout<<"t1"<<std::endl;
    VectorXd qpres;
    if (qphqp.SolveQPoases(500, qpres))
    {
        //std::pair<VectorXd, VectorXd> ret(qpres.segment(MODEL_DOF_VIRTUAL, MODEL_DOF), qpres.segment(MODEL_DOF_VIRTUAL + MODEL_DOF + contact_dof, task_dof.back()));
        return qpres.segment(MODEL_DOF_VIRTUAL, MODEL_DOF);
    }
    else
    {
        //std::pair<VectorXd, VectorXd> ret(VectorXd::Zero(MODEL_DOF), VectorXd::Zero(task_dof.back()));
        return VectorXd::Zero(MODEL_DOF);
    }
}

VectorQd WholebodyController::task_control_torque_hqp2(RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp)
{
    int he = Jtask_hqp.size(); // he = 4?

    QP_hqp.resize(he + 1);
    std::pair<VectorXd, VectorXd> ret;

    // for(int i=0;i<fstar_hqp.size();i++)
    //    std::cout << fstar_hqp[i].transpose()<<std::endl;
    std::chrono::steady_clock::time_point ht[10];
    int h_idx = 0;
    for (int i = 0; i < he; i++)
    {
        std::vector<MatrixXd> jsub(&Jtask_hqp[0], &Jtask_hqp[i] + 1);
        std::vector<VectorXd> fsub(&fstar_hqp[0], &fstar_hqp[i] + 1);
        ht[h_idx++] = std::chrono::steady_clock::now();
        ret = hqp2_step_calc(QP_hqp[i], Robot, jsub, fsub, Robot.init_qp);
        ht[h_idx++] = std::chrono::steady_clock::now();
        fstar_hqp[i] = fstar_hqp[i] + ret.second;
    }
    // ret = hqp2_damping_calc(QP_hqp[he], Robot, Jtask_hqp, fstar_hqp, Robot.init_qp);
    // fstar_hqp.push_back(ret.second);

    // for(int i=0;i<fstar_hqp.size();i++)
    //    std::cout << fstar_hqp[i].transpose()<<std::endl;

    ht[h_idx++] = std::chrono::steady_clock::now();
    VectorQd torque_qp = hqp2_contact_calc(QP_hqp[he], Robot, Jtask_hqp, fstar_hqp, Robot.init_qp);
    ht[h_idx++] = std::chrono::steady_clock::now();

    data_out2 << Robot.control_time_ << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(ht[1] - ht[0]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(ht[3] - ht[2]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(ht[5] - ht[4]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(ht[7] - ht[6]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(ht[9] - ht[8]).count() << " \t" << std::endl;

    Robot.init_qp = false;

    return ret.first;
}

VectorQd WholebodyController::task_control_torque_hqp_threaded(RobotData_fast Robot_fast, bool &init_qp)
{
    std::chrono::steady_clock::time_point t[20];
    int tcnt = 0;
    t[tcnt++] = std::chrono::steady_clock::now(); //200us
    int hqp_size = Robot_fast.Jtask_hqp.size();
    std::vector<std::future<std::pair<MatrixXd, MatrixXd>>> hqp_ret;
    for (int i = 0; i < hqp_size; i++)
    {
        hqp_ret.push_back(std::async(std::launch::async, &WholebodyController::getjkt_m, this, std::ref(Robot_fast.A_matrix_inverse), std::ref(Robot_fast.N_C), std::ref(Robot_fast.W_inv), std::ref(Robot_fast.Jtask_hqp[i])));
    }
    //std::cout<<"c2"<<std::endl;
    t[tcnt++] = std::chrono::steady_clock::now(); //30us
    int contact_dof = Robot_fast.contact_index * 6 - 6;
    t[tcnt++] = std::chrono::steady_clock::now(); //6us
    //std::cout<<"c3"<<std::endl;
    Robot_fast.NwJw = Robot_fast.qr_V2.transpose() * (Robot_fast.J_C_INV_T.topRightCorner(contact_dof, MODEL_DOF) * Robot_fast.qr_V2.transpose()).inverse();
    //std::cout<<"c4"<<std::endl;
    t[tcnt++] = std::chrono::steady_clock::now(); //10us
    std::vector<std::pair<MatrixXd, MatrixXd>> ans;
    ans.push_back(hqp_ret[0].get());
    //std::cout<<"c5"<<std::endl;
    t[tcnt++] = std::chrono::steady_clock::now(); //26us
    MatrixXd Null_task = MatrixQQd::Identity();   // - ans[0].first * Robot.lambda * Jtask_hqp[0] * Robot.A_matrix_inverse * Robot.N_C* Robot.Slc_k_T;
    VectorXd torque_prev;
    torque_prev.setZero(MODEL_DOF);
    std::vector<std::pair<VectorXd, VectorXd>> qp_ans;
    QP_hqp.resize(hqp_size + 2);
    int task1_idx = tcnt;
    t[tcnt++] = std::chrono::steady_clock::now(); //2us
    //std::cout<<"c6"<<std::endl;
    qp_ans.push_back(hqp_step_calc(QP_hqp[0], Robot_fast, torque_prev, Null_task, ans[0].first, ans[0].second, Robot_fast.fstar_hqp[0], init_qp));
    t[tcnt++] = std::chrono::steady_clock::now(); //42us
    if (hqp_size > 1)
    {
        for (int i = 1; i < hqp_size; i++)
        {
            ans.push_back(hqp_ret[i].get());
            t[tcnt++] = std::chrono::steady_clock::now(); //1 //1us
            torque_prev = torque_prev + Null_task * (ans[i - 1].first * (ans[i - 1].second * (Robot_fast.fstar_hqp[i - 1] + qp_ans[i - 1].first)));
            Null_task = Null_task * (MatrixQQd::Identity() - ans[i - 1].first * ans[i - 1].second * Robot_fast.Jtask_hqp[i - 1] * Robot_fast.A_matrix_inverse * Robot_fast.N_C.rightCols(MODEL_DOF));
            t[tcnt++] = std::chrono::steady_clock::now(); //38us //43us

            qp_ans.push_back(hqp_step_calc(QP_hqp[i], Robot_fast, torque_prev, Null_task, ans[i].first, ans[i].second, Robot_fast.fstar_hqp[i], init_qp));
            t[tcnt++] = std::chrono::steady_clock::now(); //50us //30us
        }
    }
    torque_prev = torque_prev + Null_task * ans.back().first * ans.back().second * (Robot_fast.fstar_hqp.back() + qp_ans.back().first);


    Null_task = Null_task * (MatrixQQd::Identity() - ans.back().first * ans.back().second * Robot_fast.Jtask_hqp.back() * Robot_fast.A_matrix_inverse * Robot_fast.N_C.rightCols(MODEL_DOF));
    int damping_idx = tcnt;
    t[tcnt++] = std::chrono::steady_clock::now();
    VectorXd damping_slack = hqp_damping_calc(QP_hqp[hqp_size + 1], Robot_fast, torque_prev, Null_task, init_qp);

    int contact_idx = tcnt;
    Eigen::VectorXd torque_damping = -Robot_fast.q_dot_ * 0.1;

    t[tcnt++] = std::chrono::steady_clock::now();
    torque_prev = torque_prev + Null_task * (torque_damping + damping_slack);
    VectorXd fc_opt = hqp_contact_calc(QP_hqp[hqp_size], Robot_fast, torque_prev, init_qp);

    //AAAAnd Contact Force Redistribution with QP!!
    t[tcnt] = std::chrono::steady_clock::now(); //0us
    init_qp = false;
    //return Robot.torque_grav + ans[0].first * ans[0].second * (fstar_hqp[0] + qp_ans[0].first) + Robot.NwJw * qp_ans[0].second;

    /*std::cout << "hqp calc : ";
    for (int i = 0; i < tcnt; i++)
    {
        //std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t[i + 1] - t[i]).count() << "\t";
    }

    
    std::cout << "  total : " << std::chrono::duration_cast<std::chrono::microseconds>(ts[tcnt] - t[0]).count() << std::endl;
    */
    // static double max_time = 0;
    // static double all_time = 0;
    // static double aall_time = 0;
    // static int tcc = 0;

    // double cctime = std::chrono::duration_cast<std::chrono::microseconds>(t[5] - t[0]).count();
    // all_time += cctime;
    // aall_time += cctime;

    // if (cctime > max_time)
    //     max_time = cctime;
    // static int tc = 0;
    // tc++;
    // tcc++;
    // if (tc > 2000)
    // {
    //     std::cout << " avg compute time : " << all_time / tc << "us"
    //               << "total : " << aall_time / tcc << " max : " << max_time << std::endl;
    //     all_time = 0;DK
    //     tc = 0;
    // }

    data_out1 << Robot_fast.control_time_ << " \t";
    for (int i = 0; i < qp_ans.size(); i++)
    {
        data_out1 << qp_ans[i].first.transpose() << " \t";
    }
    data_out1 << damping_slack.transpose() << std::endl;
    Robot_fast.com_time = std::chrono::duration_cast<std::chrono::microseconds>(t[damping_idx + 1] - t[damping_idx]).count();

    data_out2 << Robot_fast.control_time_<<" "
              << std::chrono::duration_cast<std::chrono::microseconds>(t[task1_idx + 1] - t[task1_idx]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(t[task1_idx + 4] - t[task1_idx + 3]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(t[task1_idx + 7] - t[task1_idx + 6]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(t[damping_idx + 1] - t[damping_idx]).count() << " \t"
              << std::chrono::duration_cast<std::chrono::microseconds>(t[contact_idx + 1] - t[contact_idx]).count() << " \t" << std::endl;
    //data_out3 << Robot_fast.control_time_;

    return torque_prev + Robot_fast.NwJw * fc_opt + Robot_fast.torque_grav;
}
void WholebodyController::copy_robot_fast(RobotData &Robot, RobotData_fast &Robot_fast, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp)
{
    Robot_fast.torque_grav = gravity_compensation_torque(Robot);
    Robot_fast.control_time_ = Robot.control_time_;
    Robot_fast.A_matrix_inverse = Robot.A_matrix_inverse;
    Robot_fast.W_inv = Robot.W_inv;
    Robot_fast.N_C = Robot.N_C;
    Robot_fast.P_C = Robot.P_C;
    Robot_fast.J_C_INV_T = Robot.J_C_INV_T;
    Robot_fast.qr_V2 = Robot.qr_V2;
    Robot_fast.fstar_hqp = fstar_hqp;
    Robot_fast.Jtask_hqp = Jtask_hqp;
    Robot_fast.contact_index = Robot.contact_index;

    Robot_fast.q_dot_ = Robot.q_dot_;

    for (int i = 0; i < 4; i++)
    {
        Robot_fast.ee_[i] = Robot.ee_[i];
        Robot_fast.ee_idx[i] = Robot.ee_idx[i];
    }
}

VectorQd WholebodyController::task_control_torque_hqp(RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp)
{
    std::chrono::steady_clock::time_point t[20];
    int tcnt = 0;
    t[tcnt++] = std::chrono::steady_clock::now();
    set_contact_multithread(Robot, 1, 1);
    //std::cout<<"c1"<<std::endl;
    int hqp_size = Jtask_hqp.size();
    std::vector<std::future<std::pair<MatrixXd, MatrixXd>>> hqp_ret;

    t[tcnt++] = std::chrono::steady_clock::now(); //200us
    for (int i = 0; i < hqp_size; i++)
    {
        //MatrixXd tempMa = Robot.A_matrix_inverse * Robot.N_C;
        hqp_ret.push_back(std::async(std::launch::async, &WholebodyController::getjkt_t, this, std::ref(Robot), std::ref(Jtask_hqp[i])));
    }
    //std::cout<<"c2"<<std::endl;
    t[tcnt++] = std::chrono::steady_clock::now(); //30us
    int contact_dof = Robot.contact_index * 6 - 6;
    Robot.torque_grav = gravity_compensation_torque(Robot);
    t[tcnt++] = std::chrono::steady_clock::now(); //6us
    //std::cout<<"c3"<<std::endl;
    Robot.NwJw = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.topRightCorner(contact_dof, MODEL_DOF) * Robot.qr_V2.transpose()).inverse();
    //std::cout<<"c4"<<std::endl;
    t[tcnt++] = std::chrono::steady_clock::now(); //10us
    std::vector<std::pair<MatrixXd, MatrixXd>> ans;
    ans.push_back(hqp_ret[0].get());
    //std::cout<<"c5"<<std::endl;
    t[tcnt++] = std::chrono::steady_clock::now(); //26us
    MatrixXd Null = MatrixQQd::Identity();        // - ans[0].first * Robot.lambda * Jtask_hqp[0] * Robot.A_matrix_inverse * Robot.N_C* Robot.Slc_k_T;
    VectorXd torque_prev;
    torque_prev.setZero(MODEL_DOF);
    std::vector<std::pair<VectorXd, VectorXd>> qp_ans;
    QP_hqp.resize(hqp_size + 1);
    t[tcnt++] = std::chrono::steady_clock::now(); //2us
    //std::cout<<"c6"<<std::endl;
    qp_ans.push_back(hqp_step_calc(QP_hqp[0], Robot, torque_prev, Null, ans[0].first, ans[0].second, fstar_hqp[0], Robot.init_qp));
    t[tcnt++] = std::chrono::steady_clock::now(); //42us
    if (hqp_size > 1)
    {
        for (int i = 1; i < hqp_size; i++)
        {
            ans.push_back(hqp_ret[i].get());
            t[tcnt++] = std::chrono::steady_clock::now(); //1 //1us
            torque_prev = torque_prev + Null * (ans[i - 1].first * (ans[i - 1].second * (fstar_hqp[i - 1] + qp_ans[i - 1].first)));
            Null = Null * (MatrixQQd::Identity() - ans[i - 1].first * ans[i - 1].second * Jtask_hqp[i - 1] * Robot.A_matrix_inverse * Robot.N_C.rightCols(MODEL_DOF));
            t[tcnt++] = std::chrono::steady_clock::now(); //38us //43us
            qp_ans.push_back(hqp_step_calc(QP_hqp[i], Robot, torque_prev, Null, ans[i].first, ans[i].second, fstar_hqp[i], Robot.init_qp));
            t[tcnt++] = std::chrono::steady_clock::now(); //50us //30us
        }
    }
    torque_prev = torque_prev + Null * ans.back().first * ans.back().second * (fstar_hqp.back() + qp_ans.back().first);
    VectorXd fc_opt = hqp_contact_calc(QP_hqp.back(), Robot, torque_prev, Robot.init_qp);
    //AAAAnd Contact Force Redistribution with QP!!
    t[tcnt] = std::chrono::steady_clock::now(); //0us
    Robot.init_qp = false;
    //return Robot.torque_grav + ans[0].first * ans[0].second * (fstar_hqp[0] + qp_ans[0].first) + Robot.NwJw * qp_ans[0].second;
    // std::cout << "hqp calc : ";
    // for (int i = 0; i < tcnt; i++)
    // {
    //     std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t[i + 1] - t[i]).count() << "\t";
    // }
    // std::cout << "  total : " << std::chrono::duration_cast<std::chrono::microseconds>(t[tcnt] - t[0]).count() << std::endl;
    static double max_time = 0;
    static double all_time = 0;
    static double aall_time = 0;
    static int tcc = 0;

    double cctime = std::chrono::duration_cast<std::chrono::microseconds>(t[5] - t[0]).count();
    all_time += cctime;
    aall_time += cctime;

    if (cctime > max_time)
        max_time = cctime;
    static int tc = 0;
    tc++;
    tcc++;
    if (tc > 2000)
    {
        std::cout << " avg compute time : " << all_time / tc << "us"
                  << "total : " << aall_time / tcc << " max : " << max_time << std::endl;
        all_time = 0;
        tc = 0;
    }
    return torque_prev + Robot.NwJw * fc_opt + Robot.torque_grav;
}

VectorQd WholebodyController::gravity_compensation_torque(RobotData &Robot, bool fixed, bool redsvd)
{
    Robot.G.setZero(MODEL_DOF + 6);
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
        Robot.G -= Robot.link_[i].Jac_COM_p.transpose() * Robot.link_[i].Mass * Robot.Grav_ref;
    }
    if (fixed)
        return Robot.G.segment(6, MODEL_DOF);

    Robot.torque_grav = (Robot.W_inv * (Robot.A_matrix_inverse.bottomRows(MODEL_DOF) * (Robot.N_C * Robot.G)));

    Robot.P_C = Robot.Lambda_c * (Robot.J_C * (Robot.A_matrix_inverse * Robot.G));
    Robot.contact_calc = false;
    return Robot.torque_grav;
}

VectorQd WholebodyController::task_control_torque(RobotData &Robot, MatrixXd J_task, VectorXd f_star_)
{
    std::cout << "This solver deprecated!" << std::endl;
    Robot.task_dof = J_task.rows();

    //Task Control Torque;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;
    Robot.Q = Robot.J_task_inv_T * Robot.Slc_k_T;
    Robot.Q_T_ = Robot.Q.transpose();
    Robot.Q_temp = Robot.Q * Robot.W_inv * Robot.Q_T_;
    Robot.Q_temp_inv = DyrosMath::pinv_glsSVD(Robot.Q_temp);

    //_F=lambda*(f_star);
    //Jtemp=J_task_inv_T*Slc_k_T;
    //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
    //Q.svd(s2,u2,v2);

    VectorQd torque_task;
    if (Robot.task_force_control)
    {
        VectorXd F_;
        F_.resize(Robot.task_dof);
        F_ = Robot.lambda * Robot.task_selection_matrix * f_star_;
        VectorXd F_2;
        F_2 = F_ + Robot.task_desired_force;
        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * F_2;
    }
    else if (Robot.task_force_control_feedback)
    {
        VectorXd F_;
        F_.resize(Robot.task_dof);

        static double right_i, left_i;

        double pd = 0.1;
        double pi = 4.0;

        double left_des = -50.0;
        double right_des = 50.0;

        double right_err = Robot.task_desired_force(10) + Robot.task_feedback_reference(1);
        double left_err = Robot.task_desired_force(16) + Robot.task_feedback_reference(7);

        right_i += right_err * Robot.d_time_;
        left_i += left_err * Robot.d_time_;

        VectorXd fc_fs;
        fc_fs = Robot.task_desired_force;
        fc_fs.setZero();

        fc_fs(10) = pd * right_err + pi * right_i;
        fc_fs(16) = pd * left_err + pi * left_i;
        F_ = Robot.lambda * (Robot.task_selection_matrix * f_star_ + fc_fs);

        VectorXd F_2;
        F_2 = F_ + Robot.task_desired_force;

        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * F_2;
    }
    else if (Robot.zmp_control)
    {
        int zmp_dof = 6;
        VectorXd F_;
        F_.resize(Robot.task_dof);
        Robot.task_selection_matrix.setIdentity(Robot.task_dof, Robot.task_dof);
        Robot.task_selection_matrix.block(0, 0, 2, 2).setZero();
        //Robot.task_selection_matrix.block(1, 1, 1, 1).setZero();

        F_ = Robot.lambda * Robot.task_selection_matrix * f_star_;

        //double kpf = 200.0;
        //double kvf = 1.0;

        Vector2d Fd_com;
        Fd_com.setZero();
        Fd_com = 9.81 / Robot.com_.pos(2) * (Robot.com_.pos.segment(0, 2) - Robot.ZMP_desired.segment(0, 2));

        //rk_.ZMP_error(1) = 0.0;

        //Robot.ZMP_command = Robot.ZMP_command + 0.5 * Robot.ZMP_error; //+ rk_.ZMP_mod;
        //Fd_com(1) = Robot.zmp_gain * 9.81 / (Robot.com_.pos(2) - Robot.link_[Right_Foot].xpos(2) * 0.5 - Robot.link_[Left_Foot].xpos(2) * 0.5) * (Robot.com_.pos(1) - Robot.ZMP_command(1));
        //rk_.ZMP_command = rk_.ZMP_command + 0.5 * rk_.ZMP_error + rk_.ZMP_mod;

        //Fd_com(1) = zmp_gain * 9.81 / rk_.com_.pos(2) * (rk_.com_.pos(1) - ZMP_task(1) - 1.0*rk_.ZMP_error(1)) * rk_.com_.mass;

        //Robot.task_desired_force.setZero(Robot.task_dof);

        //Robot.task_desired_force(1) = Fd_com(1); // - kpf*rk_.ZMP_error(1);//-kvf*rk_.com_.vel(1);
        //std::cout << Fd_com(1) << "\t" << rk_.ZMP_error(1) << std::endl;
        //task_desired_force(1) = Fd_com(1);// + kpf*rk_.ZMP_error(1)-kvf*rk_.com_.vel(1);
        //task_desired_force.segment(0, 2) = Fd_com;
        //task_desired_force(3) = ZMP_task(1) * (com_.mass * 9.81);
        //task_desired_force(4) = ZMP_task(0) * (com_.mass * 9.81);
        //F_2 = F_ + task_desired_force;

        f_star_.segment(0, 2) = Fd_com;

        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_;
    }
    else if (Robot.zmp_feedback_control)
    {
        int zmp_dof = 6;
        VectorXd F_;
        F_.resize(Robot.task_dof);
        Robot.task_selection_matrix.setIdentity(Robot.task_dof, Robot.task_dof);
        Robot.task_selection_matrix.block(0, 0, 2, 2).setZero();
        //Robot.task_selection_matrix.block(1, 1, 1, 1).setZero();
        F_ = Robot.lambda * Robot.task_selection_matrix * f_star_;
        //double kpf = 200.0;
        //double kvf = 1.0;
        Vector2d Fd_com;
        Fd_com.setZero();
        Fd_com = 9.81 / Robot.com_.pos(2) * (Robot.com_.pos.segment(0, 2) - Robot.ZMP_command.segment(0, 2));
        //rk_.ZMP_error(1) = 0.0;
        //Robot.ZMP_command = Robot.ZMP_command + 0.5 * Robot.ZMP_error; //+ rk_.ZMP_mod;
        //Fd_com(1) = Robot.zmp_gain * 9.81 / (Robot.com_.pos(2) - Robot.link_[Right_Foot].xpos(2) * 0.5 - Robot.link_[Left_Foot].xpos(2) * 0.5) * (Robot.com_.pos(1) - Robot.ZMP_command(1));
        //rk_.ZMP_command = rk_.ZMP_command + 0.5 * rk_.ZMP_error + rk_.ZMP_mod;
        //Fd_com(1) = zmp_gain * 9.81 / rk_.com_.pos(2) * (rk_.com_.pos(1) - ZMP_task(1) - 1.0*rk_.ZMP_error(1)) * rk_.com_.mass;
        //Robot.task_desired_force.setZero(Robot.task_dof);
        //Robot.task_desired_force(1) = Fd_com(1); // - kpf*rk_.ZMP_error(1);//-kvf*rk_.com_.vel(1);
        //std::cout << Fd_com(1) << "\t" << rk_.ZMP_error(1) << std::endl;
        //task_desired_force(1) = Fd_com(1);// + kpf*rk_.ZMP_error(1)-kvf*rk_.com_.vel(1);
        //task_desired_force.segment(0, 2) = Fd_com;
        //task_desired_force(3) = ZMP_task(1) * (com_.mass * 9.81);
        //task_desired_force(4) = ZMP_task(0) * (com_.mass * 9.81);
        //F_2 = F_ + task_desired_force;
        f_star_.segment(0, 2) = Fd_com;
        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_;
    }
    else
    {
        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_;
    }
    //W.svd(s,u,v);
    //V2.resize(28,6);
    //V2.zero();

    if (Robot.showdata)
    {
        std::cout << "J_task" << std::endl
                  << Robot.J_task << std::endl;
        std::cout << "Q" << std::endl
                  << Robot.Q << std::endl;
        std::cout << "W_inv" << std::endl
                  << Robot.W_inv << std::endl;
        std::cout << "Qtemp, det :" << Robot.Q_temp.determinant() << std::endl
                  << Robot.Q_temp << std::endl;
        std::cout << "QtempInv" << std::endl
                  << Robot.Q_temp_inv << std::endl;
        std::cout << "torque task : " << std::endl
                  << torque_task << std::endl;
        std::cout << "A_matrix : " << Robot.A_matrix.determinant() << std::endl;
        std::cout << Robot.A_matrix << std::endl;
        std::cout << "A_matrix : " << Robot.A_matrix_inverse.determinant() << std::endl;
        std::cout << Robot.A_matrix_inverse << std::endl;
        std::cout << "lambda : " << Robot.lambda.determinant() << std::endl;
        std::cout << Robot.lambda << std::endl;
        std::cout << "lambda_inv : " << Robot.lambda_inv.determinant() << std::endl;
        std::cout << Robot.lambda_inv << std::endl;
        std::cout << "jtanc det  : " << std::endl;
        std::cout << Robot.J_task * Robot.A_matrix_inverse * Robot.N_C << std::endl;

        Robot.showdata = false;
    }

    return torque_task;
}

VectorQd WholebodyController::task_control_torque_force_control(RobotData &Robot, MatrixXd J_task, VectorXd desiredForce)
{
    std::cout << "This solver deprecated!" << std::endl;
    Robot.task_dof = J_task.rows();
    //Task Control Torque;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();
    Robot.J_task_T = J_task.transpose();
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;
    Robot.Q = Robot.J_task_inv_T * Robot.Slc_k_T;
    Robot.Q_T_ = Robot.Q.transpose();
    Robot.Q_temp = Robot.Q * Robot.W_inv * Robot.Q_T_;
    Robot.Q_temp_inv = DyrosMath::pinv_glsSVD(Robot.Q_temp);
    //_F=lambda*(f_star);
    //Jtemp=J_task_inv_T*Slc_k_T;
    //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
    //Q.svd(s2,u2,v2);
    VectorQd torque_task;
    torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * desiredForce + gravity_compensation_torque(Robot);
    //W.svd(s,u,v);
    //V2.resize(28,6);
    //V2.zero();
    return torque_task;
}
MatrixXd WholebodyController::GetTaskLambda(RobotData &Robot, MatrixXd J_task)
{
    Robot.lambda_calc = true;
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * J_task.transpose();
    Robot.lambda = Robot.lambda_inv.inverse();
    return Robot.lambda;
}

void WholebodyController::getSupportPolygon(RobotData &Robot, std::vector<Eigen::Vector2d> &edge_point_list)
{
    int contact_index = Robot.contact_index;
    std::vector<Eigen::Vector2d> ep;
    std::vector<Eigen::Vector2d> ep_origin;
    std::vector<double> angle_list;
    ep.resize(contact_index * 4);
    //std::cout << "contact points : " << std::endl;
    for (int i = 0; i < contact_index; i++)
    {
        //std::cout << i << "idx : " << Robot.ee_idx[i] << "  x : " << Robot.ee_[Robot.ee_idx[i]].xpos(0) << "  y : " << Robot.ee_[Robot.ee_idx[i]].xpos(1) << std::endl;
        Vector3d fl[4];
        fl[0] << Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        fl[1] << Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, -Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        fl[2] << -Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, -Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        fl[3] << -Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        for (int j = 0; j < 4; j++)
        {
            ep[i * 4 + j] = (Robot.ee_[Robot.ee_idx[i]].cp_ + Robot.ee_[Robot.ee_idx[i]].rotm * fl[j]).segment(0, 2);
        }
    }
    ep_origin.resize(ep.size());
    std::copy(ep.begin(), ep.end(), ep_origin.begin());
    //check lowest y point
    int lowest_point = 0;
    for (int i = 0; i < contact_index * 4; i++)
    {
        if (ep[lowest_point](1) > ep[i](1))
        {
            lowest_point = i;
        }
    }
    Vector2d zeroPoint = ep[lowest_point];
    ep.erase(ep.begin() + lowest_point);
    angle_list.resize(ep.size());
    //std::cout << "zero point x : " << zeroPoint(0) << "  y : " << zeroPoint(1) << std::endl;
    for (int i = 0; i < ep.size(); i++)
    {
        angle_list[i] = DyrosMath::getOrientation2d(Vector2d(1, 0), ep[i] - zeroPoint);
        if (angle_list[i] < 0)
            angle_list[i] = angle_list[i] + 2 * 3.1415;
    }
    int start_index, end_index;
    start_index = DyrosMath::findMinAdr(angle_list);
    end_index = DyrosMath::findMaxAdr(angle_list);
    Vector2d startPoint = ep[start_index];
    Vector2d endPoint = ep[end_index];
    //for (int i = 0; i < angle_list.size(); i++)
    //   std::cout << angle_list[i] << "\t" << std::endl;
    //std::cout << "start p  x : " << startPoint(0) << "  y : " << startPoint(1) << std::endl;
    //std::cout << "end p  x : " << endPoint(0) << "  y : " << endPoint(1) << std::endl;
    edge_point_list.push_back(zeroPoint);
    edge_point_list.push_back(startPoint);
    ep.erase(ep.begin() + start_index);

    int origin_size = ep.size();
    for (int i = 0; i < origin_size; i++)
    {
        angle_list.resize(ep.size());
        for (int j = 0; j < ep.size(); j++)
        {
            angle_list[j] = DyrosMath::getOrientation2d(edge_point_list[i + 1] - edge_point_list[i], ep[j] - edge_point_list[i + 1]);
            if (angle_list[j] < 0)
                angle_list[j] = angle_list[j] + 2 * 3.1415;
        }
        int idx = DyrosMath::findMinAdr(angle_list);
        edge_point_list.push_back(ep[idx]);
        if (ep[idx] == endPoint)
        {
            //std::cout << "End Found" << std::endl;
            break;
        }
        ep.erase(ep.begin() + idx);
    }
}

Vector2d WholebodyController::fstar_regulation(RobotData &Robot, Vector3d f_star)
{
    //Check Feedback f_star is over limit
    //think fstar as desired acceleration
    //only for COM
    //if desired COM is over
    //support polygon check
    int contact_index = Robot.contact_index;
    std::vector<Eigen::Vector2d> ep;
    std::vector<Eigen::Vector2d> ep_origin;
    std::vector<double> angle_list;
    ep.resize(contact_index * 4);
    //std::cout << "contact points : " << std::endl;
    for (int i = 0; i < contact_index; i++)
    {
        //std::cout << i << "idx : " << Robot.ee_idx[i] << "  x : " << Robot.ee_[Robot.ee_idx[i]].xpos(0) << "  y : " << Robot.ee_[Robot.ee_idx[i]].xpos(1) << std::endl;
        Vector3d fl[4];
        fl[0] << Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        fl[1] << Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, -Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        fl[2] << -Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, -Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        fl[3] << -Robot.ee_[Robot.ee_idx[i]].cs_x_length * 0.5, Robot.ee_[Robot.ee_idx[i]].cs_y_length * 0.5, 0;
        for (int j = 0; j < 4; j++)
        {
            ep[i * 4 + j] = (Robot.ee_[Robot.ee_idx[i]].cp_ + Robot.ee_[Robot.ee_idx[i]].rotm * fl[j]).segment(0, 2);
        }
    }

    ep_origin.resize(ep.size());
    std::copy(ep.begin(), ep.end(), ep_origin.begin());
    //check lowest y point
    int lowest_point = 0;
    for (int i = 0; i < contact_index * 4; i++)
    {
        if (ep[lowest_point](1) > ep[i](1))
        {
            lowest_point = i;
        }
    }
    Vector2d zeroPoint = ep[lowest_point];

    ep.erase(ep.begin() + lowest_point);
    angle_list.resize(ep.size());
    //std::cout << "zero point x : " << zeroPoint(0) << "  y : " << zeroPoint(1) << std::endl;

    for (int i = 0; i < ep.size(); i++)
    {
        angle_list[i] = DyrosMath::getOrientation2d(Vector2d(1, 0), ep[i] - zeroPoint);
        if (angle_list[i] < 0)
            angle_list[i] = angle_list[i] + 2 * 3.1415;
    }

    int start_index, end_index;
    start_index = DyrosMath::findMinAdr(angle_list);
    end_index = DyrosMath::findMaxAdr(angle_list);
    Vector2d startPoint = ep[start_index];
    Vector2d endPoint = ep[end_index];

    //for (int i = 0; i < angle_list.size(); i++)
    //   std::cout << angle_list[i] << "\t" << std::endl;
    //std::cout << "start p  x : " << startPoint(0) << "  y : " << startPoint(1) << std::endl;
    //std::cout << "end p  x : " << endPoint(0) << "  y : " << endPoint(1) << std::endl;

    std::vector<Eigen::Vector2d> edge_point_list;

    edge_point_list.push_back(zeroPoint);
    edge_point_list.push_back(startPoint);
    ep.erase(ep.begin() + start_index);

    int origin_size = ep.size();

    for (int i = 0; i < origin_size; i++)
    {
        angle_list.resize(ep.size());
        for (int j = 0; j < ep.size(); j++)
        {
            angle_list[j] = DyrosMath::getOrientation2d(edge_point_list[i + 1] - edge_point_list[i], ep[j] - edge_point_list[i + 1]);
            if (angle_list[j] < 0)
                angle_list[j] = angle_list[j] + 2 * 3.1415;
        }
        int idx = DyrosMath::findMinAdr(angle_list);
        edge_point_list.push_back(ep[idx]);
        if (ep[idx] == endPoint)
        {
            //std::cout << "End Found" << std::endl;
            break;
        }

        ep.erase(ep.begin() + idx);
    }

    //current com position
    Vector3d fstar_regulated;

    fstar_regulated(2) = f_star(2);
    f_star(2) = 0;
    Vector2d p_com = Robot.com_.pos.segment(0, 2);
    Vector2d zmp_by_fstar;

    zmp_by_fstar = p_com - Robot.com_.pos(2) * f_star.segment(0, 2) / (f_star(2) + 9.81);
    int ep_size = edge_point_list.size();
    edge_point_list.push_back(edge_point_list[0]);

    Vector2d zmp_r;

    Eigen::MatrixXd edgePointMat(2, edge_point_list.size());

    for (int i = 0; i < edge_point_list.size(); i++)
    {
        edgePointMat.block(0, i, 2, 1) = edge_point_list[i];
    }

    if (DyrosMath::isInPolygon(zmp_by_fstar, edgePointMat))
    {
        fstar_regulated.segment(0, 2) = f_star.segment(0, 2);
    }
    else
    {
        std::cout << Robot.control_time_ << "////// fstar regulation activate ////// " << std::endl;
        //std::cout << "x : ";
        for (int i = 0; i < edge_point_list.size(); i++)
        {
            std::cout << edge_point_list[i](0) << "\t" << std::endl;
        }
        //std::cout << std::endl
        //          << "y : ";
        for (int i = 0; i < edge_point_list.size(); i++)
        {
            std::cout << edge_point_list[i](1) << "\t" << std::endl;
        }
        //std::cout << std::endl;

        for (int i = 0; i < ep_size; i++)
        {
            if (DyrosMath::checkIntersect(edge_point_list[i], edge_point_list[i + 1], p_com, zmp_by_fstar))
            {
                zmp_r = DyrosMath::getIntersectPoint(edge_point_list[i], edge_point_list[i + 1], p_com, zmp_by_fstar);

                //std::cout << "found at " << i << std::endl;
                std::cout << "com pos x : " << p_com(0) << "  y : " << p_com(1) << std::endl;
                std::cout << "zmp fstar x : " << zmp_by_fstar(0) << "  y : " << zmp_by_fstar(1) << std::endl;
                std::cout << "ip x : " << zmp_r(0) << "  y : " << zmp_r(1) << std::endl;
                std::cout << "fstar origin : " << f_star.transpose() << std::endl;
                //std::cout << "fstar origin : " << fstar_regulated.transpose() << std::endl;
            }
        }
        fstar_regulated.segment(0, 2) = (f_star(2) + 9.81) / Robot.com_.pos(2) * (p_com - zmp_r);

        //std::cout << Robot.control_time_ << " org x : " << zmp_by_fstar(0) << " y : " << zmp_by_fstar(1) << " reg x : " << zmp_r(0) << " y : " << zmp_r(1) << std::endl;
    }

    Eigen::Vector3d com_f = Robot.total_mass * (fstar_regulated - Robot.Grav_ref);
    if (abs(com_f(0)) > abs(com_f(2) * Robot.friction_ratio))
    {
        //std::cout << "original fx : " << fstar_regulated(0) << std::endl;
        fstar_regulated(0) = fstar_regulated(0) / abs(com_f(0)) * abs(com_f(2) * Robot.friction_ratio);
        //std::cout << "modified fx : " << fstar_regulated(0) << std::endl;
    }
    if (abs(com_f(1)) > abs(com_f(2) * Robot.friction_ratio))
    {
        //std::cout << "original fy : " << fstar_regulated(1) << std::endl;
        fstar_regulated(1) = fstar_regulated(1) / abs(com_f(1)) * abs(com_f(2) * Robot.friction_ratio);
        //std::cout << "modified fy : " << fstar_regulated(1) << std::endl;
    }

    return fstar_regulated.segment(0, 2);
}

VectorQd WholebodyController::task_control_torque_with_acc_cr(RobotData &Robot, MatrixXd J_task, VectorXd f_star_acc, VectorXd f_star_feedback)
{

    Robot.task_dof = J_task.rows();
    Robot.J_task_T = Robot.J_task.transpose();

    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * J_task.transpose();
    Robot.lambda = Robot.lambda_inv.inverse();

    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;
    Robot.Q = Robot.J_task_inv_T * Robot.Slc_k_T;
    Robot.Q_T_ = Robot.Q.transpose();

    Robot.Q_temp = Robot.Q * Robot.W_inv * Robot.Q_T_;
    Robot.Q_temp_inv = DyrosMath::pinv_glsSVD(Robot.Q_temp);

    VectorQd torque_task_acc;
    VectorQd torque_task;
    VectorQd torque_contact;
    torque_task_acc = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_acc + gravity_compensation_torque(Robot);
    torque_contact = contact_torque_calc_from_QP(Robot, torque_task);
    torque_task = torque_task_acc + torque_contact + Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_feedback;

    return torque_task;
}

void WholebodyController::getJkt(RobotData &Robot, MatrixXd &J_task, MatrixXd &Jkt)
{
    Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * J_task.transpose();
    Robot.lambda = Robot.lambda_inv.inverse();
    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;
    Robot.Q = Robot.J_task_inv_T * Robot.Slc_k_T;
    Robot.Q_T_ = Robot.Q.transpose();
    Robot.Q_temp = Robot.Q * Robot.W_inv * Robot.Q_T_;
    Robot.Q_temp_inv = DyrosMath::pinv_QR(Robot.Q_temp);
    Jkt = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv;
}

MatrixXd WholebodyController::getJkt_f(RobotData &Robot, MatrixXd &J_task, MatrixXd &lambda)
{
    MatrixXd linv = J_task * Robot.A_matrix_inverse * Robot.N_C * J_task.transpose();
    lambda = linv.inverse();

    MatrixXd Q_ = lambda * J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.Slc_k_T;
    MatrixXd Q_t_ = Q_.transpose();
    MatrixXd Q_temp, Q_temp_inv;
    Q_temp = Q_ * Robot.W_inv * Q_t_;
    Q_temp_inv = DyrosMath::pinv_QR(Q_temp);
    return Robot.W_inv * Q_t_ * Q_temp_inv;
}

VectorQd WholebodyController::task_control_torque_with_gravity(RobotData &Robot, MatrixXd J_task, VectorXd f_star_, bool force_control)
{
    Robot.task_dof = J_task.rows();
    Robot.J_task_T = Robot.J_task.transpose();

    Robot.task_force_control = force_control;

    if (Robot.task_force_control && Robot.lambda_calc)
    {
    }
    else if (Robot.task_force_control)
    {
        Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * J_task.transpose();
        Robot.lambda = Robot.lambda_inv.inverse();
    }
    else
    {
        Robot.lambda_inv = J_task * Robot.A_matrix_inverse * Robot.N_C * J_task.transpose();
        Robot.lambda = Robot.lambda_inv.inverse();
    }

    //Task Control Torque;

    Robot.J_task_inv_T = Robot.lambda * J_task * Robot.A_matrix_inverse * Robot.N_C;
    Robot.Q = Robot.J_task_inv_T * Robot.Slc_k_T;
    Robot.Q_T_ = Robot.Q.transpose();

    Robot.Q_temp = Robot.Q * Robot.W_inv * Robot.Q_T_;
    Robot.Q_temp_inv = DyrosMath::pinv_QR(Robot.Q_temp);

    //_F=lambda*(f_star);
    //Jtemp=J_task_inv_T*Slc_k_T;
    //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
    //Q.svd(s2,u2,v2);

    VectorQd torque_task;

    if (Robot.task_force_control)
    {

        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * f_star_ + gravity_compensation_torque(Robot);
    }
    else
    {
        torque_task = Robot.W_inv * Robot.Q_T_ * Robot.Q_temp_inv * Robot.lambda * f_star_ + gravity_compensation_torque(Robot);
    }
    Robot.task_force_control = false;
    Robot.lambda_calc = false;
    //W.svd(s,u,v);
    //V2.resize(28,6);
    //V2.zero();

    //torque_task = torque_task +

    return torque_task;
}

VectorQd WholebodyController::task_control_torque_motor(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_)
{
    Robot.task_dof = J_task.rows();

    //Task Control Torque;
    Robot.J_task = J_task;
    Robot.J_task_T.resize(MODEL_DOF + 6, Robot.task_dof);
    Robot.J_task_T.setZero();
    Robot.lambda_inv.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda_inv.setZero();
    Robot.lambda.resize(Robot.task_dof, Robot.task_dof);
    Robot.lambda.setZero();

    Robot.J_task_T = J_task.transpose();

    Robot.lambda_motor_inv = J_task * Robot.Motor_inertia_inverse * (Robot.I37 - Robot.J_C.transpose() * Robot.J_C_INV_T) * Robot.J_task_T;

    Robot.lambda_motor = Robot.lambda_motor_inv.inverse();
    Robot.J_task_inv_motor_T = Robot.lambda_motor * J_task * Robot.Motor_inertia_inverse * (Robot.I37 - Robot.J_C.transpose() * Robot.J_C_INV_T);

    Robot.Q_motor = Robot.J_task_inv_motor_T * Robot.Slc_k_T;
    Robot.Q_motor_T_ = Robot.Q_motor.transpose();

    Robot.Q_motor_temp = Robot.Q_motor * Robot.W_inv * Robot.Q_motor_T_;

    Robot.Q_motor_temp_inv = DyrosMath::pinv_QR(Robot.Q_motor_temp);

    VectorQd torque_task;
    torque_task = Robot.W_inv * Robot.Q_motor_T_ * Robot.Q_motor_temp_inv * Robot.lambda_motor * f_star_;

    return torque_task;
}
/*
VectorQd WholebodyController::task_control_torque_custom_force(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(MODEL_DOF + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  F_ = lambda * selection_matrix * f_star_;

  VectorXd F_2;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}

VectorQd WholebodyController::task_control_torque_custom_force_feedback(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(MODEL_DOF + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  static double right_i, left_i;

  double pd = 0.1;
  double pi = 4.0;

  double left_des = -50.0;
  double right_des = 50.0;

  double right_err = desired_force(10) + ft_hand(1);
  double left_err = desired_force(16) + ft_hand(7);

  right_i += right_err * d_time_;
  left_i += left_err * d_time_;

  VectorXd fc_fs; // = desired_force;
  fc_fs = desired_force;
  fc_fs.setZero();

  fc_fs(10) = pd * right_err + pi * right_i;
  fc_fs(16) = pd * left_err + pi * left_i;

  //std::cout << "right : " << fc_fs(10) << std::endl;
  //std::cout << "left : " << fc_fs(16) << std::endl;

  F_ = lambda * (selection_matrix * f_star_ + fc_fs);

  //F_ = selection_matrix * lambda * f_star_;

  VectorXd F_2;

  //desired_force(10) = desired_force(10) + right_des;
  //desired_force(16) = desired_force(16) + left_des;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}
*/
void WholebodyController::set_force_control(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force)
{
    Robot.task_force_control = true;
    Robot.task_selection_matrix = selection_matrix;
    Robot.task_desired_force = desired_force;
}
void WholebodyController::set_force_control_feedback(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{
    Robot.task_force_control_feedback = true;
    Robot.task_selection_matrix = selection_matrix;
    Robot.task_desired_force = desired_force;
    Robot.task_feedback_reference = ft_hand;
}

void WholebodyController::set_zmp_control(RobotData &Robot, Vector2d ZMP, double gain)
{
    Robot.zmp_control = true;
    Robot.ZMP_desired(0) = ZMP(0);
    Robot.ZMP_desired(1) = ZMP(1);
    Robot.zmp_gain = gain;
}

void WholebodyController::set_zmp_feedback_control(RobotData &Robot, Vector2d ZMP, bool &reset_error)
{
    Robot.zmp_feedback_control = true;
    Robot.ZMP_ft = GetZMPpos_fromFT(Robot);
    Robot.ZMP_ft.segment(0, 2) = Robot.com_.ZMP;
    Robot.ZMP_error = Robot.ZMP_desired - Robot.ZMP_ft;

    Robot.ZMP_desired.segment(0, 2) = ZMP;
    if (reset_error)
    {
        std::cout << "error reset!" << std::endl;
        Robot.ZMP_command = Robot.ZMP_desired;
        Robot.ZMP_error.setZero();
        reset_error = false;
    }
    else
    {
        Robot.ZMP_command = Robot.ZMP_command + 0.01 * Robot.ZMP_error;
    }
}

Vector3d WholebodyController::getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now)
{

    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = kp(i) * (p_desired(i) - p_now(i)) + kd(i) * (d_desired(i) - d_now(i));
    }

    return fstar_;
}

Vector3d WholebodyController::getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now)
{

    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(Robot.yaw);
    Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(r_now, r_desired);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (kp(i) * angle_d_global(i) - kd(i) * w_now(i));
    }

    return fstar_;
}

Vector3d WholebodyController::getfstar_tra(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt)
{
    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = kpt(i) * (Robot.link_[link_id].x_traj(i) - Robot.link_[link_id].xpos(i)) + kdt(i) * (Robot.link_[link_id].v_traj(i) - Robot.link_[link_id].v(i));
    }

    return fstar_;
}

Vector3d WholebodyController::getfstar_tra(RobotData &Robot, int link_id)
{
    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = Robot.link_[link_id].a_traj(i) * Robot.link_[link_id].acc_p_gain(i) + Robot.link_[link_id].pos_p_gain(i) * (Robot.link_[link_id].x_traj(i) - Robot.link_[link_id].xpos(i)) + Robot.link_[link_id].pos_d_gain(i) * (Robot.link_[link_id].v_traj(i) - Robot.link_[link_id].v(i));
    }

    return fstar_;
}

Vector3d WholebodyController::getfstar_rot(RobotData &Robot, int link_id, Vector3d kpa, Vector3d kda)
{
    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(Robot.yaw);

    Vector3d angle_d_global = -Rotyaw * DyrosMath::getPhi(Robot.link_[link_id].Rotm, Robot.link_[link_id].r_traj);

    //Matrix3d Rotyaw = DyrosMath::rotateWithZ(yaw);

    //Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(link_[link_id].Rotm, link_[link_id].r_traj);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (kpa(i) * angle_d_global(i) - kda(i) * Robot.link_[link_id].w(i));
    }
    /*
  std::cout << "fstar check " << std::endl
            << link_[link_id].name << std::endl
            << " rotation now " << std::endl
            << link_[link_id].Rotm << std::endl
            << "desired rotation " << std::endl
            << link_[link_id].r_traj << std::endl
            << "angle d " << std::endl
            << angle_d << std::endl
            << "global angle d " << std::endl
            << angle_d_global << std::endl
            << "fstar " << std::endl
            << fstar_ << std::endl
            << " ////////////////////////////////////////////////////////////////" << std::endl;
  */

    return fstar_;
}

Vector3d WholebodyController::getfstar_rot(RobotData &Robot, int link_id)
{
    Vector3d fstar_;

    Matrix3d Rotyaw = DyrosMath::rotateWithZ(Robot.yaw);

    Vector3d angle_d_global = -Rotyaw * DyrosMath::getPhi(Robot.link_[link_id].Rotm, Robot.link_[link_id].r_traj);

    //Matrix3d Rotyaw = DyrosMath::rotateWithZ(yaw);

    //Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(link_[link_id].Rotm, link_[link_id].r_traj);

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = (Robot.link_[link_id].rot_p_gain(i) * angle_d_global(i) + Robot.link_[link_id].rot_d_gain(i) * (Robot.link_[link_id].w_traj(i) - Robot.link_[link_id].w(i)));
    }
    /*
    std::cout << "fstar check " << std::endl
              << Robot.link_[link_id].name << std::endl
              << " rotation now " << std::endl
              << Robot.link_[link_id].Rotm << std::endl
              << "desired rotation " << std::endl
              << Robot.link_[link_id].r_traj << std::endl
              //<< "angle d " << std::endl
              //<< angle_d << std::endl
              << "global angle d " << std::endl
              << angle_d_global << std::endl
              << "fstar " << std::endl
              << fstar_ << std::endl
              << " ////////////////////////////////////////////////////////////////" << std::endl;*/

    return fstar_;
}

Vector6d WholebodyController::getfstar6d(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt, Vector3d kpa, Vector3d kda)
{
    Vector6d f_star;
    f_star.segment(0, 3) = getfstar_tra(Robot, link_id, kpt, kdt);
    f_star.segment(3, 3) = getfstar_rot(Robot, link_id, kpa, kda);
    return f_star;
}

Vector6d WholebodyController::getfstar6d(RobotData &Robot, int link_id)
{
    Vector6d f_star;
    f_star.segment(0, 3) = getfstar_tra(Robot, link_id);
    f_star.segment(3, 3) = getfstar_rot(Robot, link_id);
    return f_star;
}

Vector3d WholebodyController::getfstar_acc_tra(RobotData &Robot, int link_id)
{
    Vector3d fstar_;

    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = Robot.link_[link_id].a_traj(i);
    }

    return fstar_;
}
Vector3d WholebodyController::getfstar_acc_rot(RobotData &Robot, int link_id)
{
    Vector3d fstar_;
    for (int i = 0; i < 3; i++)
    {
        fstar_(i) = Robot.link_[link_id].ra_traj(i);
    }
    return fstar_;
}

VectorQd WholebodyController::contact_force_custom(RobotData &Robot, VectorQd command_torque, Eigen::VectorXd contact_force_now, Eigen::VectorXd contact_force_desired)
{

    MatrixXd V2;

    int singular_dof = 6;
    int contact_dof = Robot.J_C.rows();
    V2 = Robot.qr_V2.transpose();

    MatrixXd Scf_;
    Scf_.setZero(contact_dof - singular_dof, contact_dof);
    Scf_.block(0, 0, contact_dof - singular_dof, contact_dof - singular_dof).setIdentity();

    // std::cout << contact_force_desired << std::endl
    //           << std::endl
    //           << std::endl;
    // std::cout << contact_force_now << std::endl
    //           << std::endl
    //           << std::endl;
    // std::cout << std::endl;

    VectorXd desired_force = contact_force_desired - contact_force_now;

    MatrixXd temp = Scf_ * Robot.J_C_INV_T * Robot.Slc_k_T * V2;
    MatrixXd temp_inv = temp.inverse();
    MatrixXd Vc_ = V2 * temp_inv;

    VectorXd reduced_desired_force = Scf_ * desired_force;
    VectorQd torque_contact_ = Vc_ * reduced_desired_force;

    return torque_contact_;
}

VectorXd WholebodyController::get_contact_force(RobotData &Robot, VectorQd command_torque)
{
    VectorXd contactforce, Cont;
    contactforce.setZero(12);
    Cont = Robot.J_C_INV_T.rightCols(MODEL_DOF) * command_torque - Robot.Lambda_c * (Robot.J_C * (Robot.A_matrix_inverse * Robot.G));
    for (int i = 0; i < 4; i++)
    {
        Robot.ee_[Robot.ee_idx[i]].contact_force.setZero();
    }
    for (int i = 0; i < Robot.contact_index; i++)
    {
        Robot.ee_[Robot.ee_idx[i]].contact_force = Cont.segment(6 * i, 6);
    }

    if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        contactforce = Cont.segment(0, 12);
    else if (Robot.ee_[0].contact)
        contactforce.segment(0, 6) = Cont.segment(0, 6);
    else if (Robot.ee_[1].contact)
        contactforce.segment(6, 6) = Cont.segment(0, 6);

    return contactforce;
}

VectorQd WholebodyController::get_joint_acceleration(RobotData &Robot, VectorQd commnad_torque)
{
    VectorXd jointAcc;
    jointAcc = Robot.A_matrix_inverse * ((Robot.I37 - Robot.J_C.transpose() * Robot.J_C_INV_T) * Robot.Slc_k_T * commnad_torque + Robot.J_C.transpose() * Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G - Robot.G);
    return jointAcc.segment(6, MODEL_DOF);
}

VectorQd WholebodyController::contact_force_redistribution_torque(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta)
{
    //Contact Jacobian task : rightfoot to leftfoot

    int contact_dof_ = Robot.J_C.rows();

    ForceRedistribution.setZero();

    if (contact_dof_ == 12)
    {

        Vector12d ContactForce_ = Robot.J_C_INV_T * Robot.Slc_k_T * command_torque - Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G;

        Vector3d P1_, P2_;

        P1_ = Robot.link_[Left_Foot].xpos_contact - Robot.link_[COM_id].xpos;
        P2_ = Robot.link_[Right_Foot].xpos_contact - Robot.link_[COM_id].xpos;

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

        //std::cout << "fres - calc" << std::endl
        //          << ResultantForce_ << std::endl;

        //J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
        ForceRedistribution = force_rot_yaw.transpose() * ResultRedistribution_;

        //JacobiSVD<MatrixXd> svd(Robot.W, ComputeThinU | ComputeThinV);

        int singular_dof = 6;
        int contact_dof = Robot.J_C.rows();

        Vector12d desired_force;

        desired_force.setZero();
        Eigen::Matrix<double, 6, 12> Scf_;

        bool right_master = false;

        Scf_.setZero();
        Scf_.block(0, 6, 6, 6).setIdentity();

        for (int i = 0; i < 6; i++)
        {
            desired_force(i + 6) = -ContactForce_(i + 6) + ForceRedistribution(i + 6);
        }
        /*
        Eigen::Matrix<float,6,6> temp;
        temp = Scf_ * Robot.J_C_INV_T * Robot.Slc_k_T * Robot.qr_V2.transpose();
        temp.inverse();*/

        return Robot.qr_V2.transpose() * (Scf_ * Robot.J_C_INV_T * Robot.Slc_k_T * Robot.qr_V2.transpose()).inverse() * Scf_ * desired_force;
    }
    else
    {
        return Eigen::VectorXd::Zero(12);
    }
}

VectorQd WholebodyController::contact_force_redistribution_torque_walking(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta, double ratio, int supportFoot)
{
    //Contact Jacobian task : rightfoot to leftfoot

    int contact_dof_ = Robot.J_C.rows();

    VectorQd torque_contact_;

    ForceRedistribution.setZero();

    if (contact_dof_ == 12)
    {

        Vector12d ContactForce_ = Robot.J_C_INV_T * Robot.Slc_k_T * command_torque - Robot.Lambda_c * Robot.J_C * Robot.A_matrix_inverse * Robot.G;

        Vector3d P1_, P2_;

        P1_ = Robot.link_[Left_Foot].xpos_contact - Robot.link_[COM_id].xpos;
        P2_ = Robot.link_[Right_Foot].xpos_contact - Robot.link_[COM_id].xpos;

        Matrix3d Rotyaw = DyrosMath::rotateWithZ(-Robot.yaw);

        Vector3d P1_local, P2_local;
        P1_local = Rotyaw * P1_;
        P2_local = Rotyaw * P2_;

        MatrixXd force_rot_yaw;
        force_rot_yaw.setZero(12, 12);
        for (int i = 0; i < 4; i++)
        {
            force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
        }

        Vector6d ResultantForce_;
        ResultantForce_.setZero();

        Vector12d ResultRedistribution_;
        ResultRedistribution_.setZero();

        torque_contact_.setZero();

        double eta_cust = 0.99;
        double foot_length = 0.26;
        double foot_width = 0.1;

        Vector12d ContactForce_Local_yaw;
        ContactForce_Local_yaw = force_rot_yaw * ContactForce_; //Robot frame based contact force

        //ZMP_pos = GetZMPpos(P1_local, P2_local, ContactForce_Local_yaw);

        ForceRedistributionTwoContactMod2(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, P1_local, P2_local, ContactForce_Local_yaw, ResultantForce_, ResultRedistribution_, eta);

        //std::cout << "fres - calc" << std::endl
        //          << ResultantForce_ << std::endl;

        //J_task * Robot.A_matrix_inverse * Robot.N_C * Robot.J_task_T;
        ForceRedistribution = force_rot_yaw.transpose() * ResultRedistribution_;

        //JacobiSVD<MatrixXd> svd(Robot.W, ComputeThinU | ComputeThinV);

        int singular_dof = 6;
        int contact_dof = Robot.J_C.rows();

        Vector12d desired_force;

        desired_force.setZero();
        MatrixXd Scf_;

        bool right_master;

        if (supportFoot == 0)
        {
            right_master = 1.0;
        }
        else
        {
            right_master = 0.0;
        }

        if (right_master)
        {
            Scf_.setZero(6, 12);
            Scf_.block(0, 0, 6, 6).setIdentity();

            for (int i = 0; i < 6; i++)
            {
                desired_force(i) = -ContactForce_(i) + ForceRedistribution(i) * ratio;
            }
        }
        else
        {

            Scf_.setZero(6, 12);
            Scf_.block(0, 6, 6, 6).setIdentity();

            for (int i = 0; i < 6; i++)
            {
                desired_force(i + 6) = -ContactForce_(i + 6) + ForceRedistribution(i + 6) * ratio;
            }
        }
        MatrixXd temp = Scf_ * Robot.J_C_INV_T * Robot.Slc_k_T * Robot.qr_V2.transpose();
        MatrixXd temp_inv = temp.inverse(); //DyrosMath::pinv_SVD(temp);
        MatrixXd Vc_ = Robot.qr_V2.transpose() * temp_inv;

        Vector6d reduced_desired_force = Scf_ * desired_force;
        torque_contact_ = Vc_ * reduced_desired_force;
    }
    else
    {
        torque_contact_.setZero();
    }

    return torque_contact_;
}

Vector3d WholebodyController::GetZMPpos(RobotData &Robot, bool Local)
{
    Vector3d zmp_pos;
    Vector3d P_;
    zmp_pos.setZero();
    P_.setZero();
    static Vector3d zmp_pos_max = (Eigen::Vector3d() << 0, 0, 0).finished();
    static Vector3d zmp_pos_min = (Eigen::Vector3d() << 0, 0, 0).finished();

    Vector3d zmp[4];
    double total_z = 0;
    for (int i = 0; i < 4; i++)
        zmp[i].setZero();
    for (int i = 0; i < Robot.contact_index; i++)
    {
        zmp[Robot.ee_idx[i]](0) = Robot.ee_[Robot.ee_idx[i]].cp_(0) + (-Robot.ee_[Robot.ee_idx[i]].contact_force(4) - Robot.ee_[Robot.ee_idx[i]].contact_force(0) * (Robot.ee_[Robot.ee_idx[i]].cp_(2) - Robot.ee_[Robot.ee_idx[i]].cp_(2))) / Robot.ee_[Robot.ee_idx[i]].contact_force(2);
        zmp[Robot.ee_idx[i]](1) = Robot.ee_[Robot.ee_idx[i]].cp_(1) + (Robot.ee_[Robot.ee_idx[i]].contact_force(3) - Robot.ee_[Robot.ee_idx[i]].contact_force(1) * (Robot.ee_[Robot.ee_idx[i]].cp_(2) - Robot.ee_[Robot.ee_idx[i]].cp_(2))) / Robot.ee_[Robot.ee_idx[i]].contact_force(2);
        Robot.ee_[Robot.ee_idx[i]].zmp = zmp[Robot.ee_idx[i]];
        total_z += Robot.ee_[Robot.ee_idx[i]].contact_force(2);
        zmp_pos(0) += zmp[Robot.ee_idx[i]](0) * Robot.ee_[Robot.ee_idx[i]].contact_force(2);
        zmp_pos(1) += zmp[Robot.ee_idx[i]](1) * Robot.ee_[Robot.ee_idx[i]].contact_force(2);
    }
    zmp_pos(0) = zmp_pos(0) / total_z;
    zmp_pos(1) = zmp_pos(1) / total_z;

    /*
    if (Local)
    {
        zmp_pos(0) = (-Robot.ContactForce(4) - (Robot.ee_[0].cp_(2) - P_(2)) * Robot.ContactForce(0) + Robot.ee_[0].cp_(0) * Robot.ContactForce(2) - Robot.ContactForce(10) - (Robot.ee_[1].cp_(2) - P_(2)) * Robot.ContactForce(6) + Robot.ee_[1].cp_(0) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
        zmp_pos(1) = (Robot.ContactForce(3) - (Robot.ee_[0].cp_(2) - P_(2)) * Robot.ContactForce(1) + Robot.ee_[0].cp_(1) * Robot.ContactForce(2) + Robot.ContactForce(9) - (Robot.ee_[1].cp_(2) - P_(2)) * Robot.ContactForce(7) + Robot.ee_[1].cp_(1) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
    }
    else
    {
        Robot.ZMP_l(0) = Robot.ee_[0].cp_(0) + (-Robot.ContactForce(4) - Robot.ContactForce(0) * (Robot.ee_[0].cp_(2) - Robot.ee_[0].cp_(2))) / Robot.ContactForce(2);
        Robot.ZMP_l(1) = Robot.ee_[0].cp_(1) + (Robot.ContactForce(3) - Robot.ContactForce(1) * (Robot.ee_[0].cp_(2) - Robot.ee_[0].cp_(2))) / Robot.ContactForce(2);

        Robot.ZMP_r(0) = Robot.ee_[1].cp_(0) + (-Robot.ContactForce(4 + 6) - Robot.ContactForce(0 + 6) * (Robot.ee_[1].cp_(2) - Robot.ee_[1].cp_(2))) / Robot.ContactForce(2 + 6);
        Robot.ZMP_r(1) = Robot.ee_[1].cp_(1) + (Robot.ContactForce(3 + 6) - Robot.ContactForce(1 + 6) * (Robot.ee_[1].cp_(2) - Robot.ee_[1].cp_(2))) / Robot.ContactForce(2 + 6);

        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            zmp_pos(0) = (Robot.ZMP_l(0) * Robot.ContactForce(2) + Robot.ZMP_r(0) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
            zmp_pos(1) = (Robot.ZMP_l(1) * Robot.ContactForce(2) + Robot.ZMP_r(1) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
        }
        else if (Robot.ee_[0].contact) //left contact
        {
            //std::cout << " f0 : " << rk_.ContactForce(0) << " f1 : " << rk_.ContactForce(1) << " f2 : " << rk_.ContactForce(2) << " f3 : " << rk_.ContactForce(3) << " f4 : " << rk_.ContactForce(4) << " f5 : " << rk_.ContactForce(5) << std::endl;
            //std::cout<<"rk_.ContactForce(4) : "<<rk_.ContactForce(4)<<"rk_.ContactForce(2)"
            //std::cout << "x : " << rk_.ContactForce(4) / rk_.ContactForce(2) << "\t";
            //std::cout << "y : " << rk_.ContactForce(3) / rk_.ContactForce(2) << "\t cp x: " << rk_.link_[Left_Foot].xpos_contact(0) << "\t cp y : " << rk_.link_[Left_Foot].xpos_contact(1) << std::endl;
            zmp_pos(0) = Robot.ZMP_l(0);
            zmp_pos(1) = Robot.ZMP_l(1);
            //zmp_pos(0) = (-ContactForce(4) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(0) + rk_.ee_[1].cp_(0) * ContactForce(2) - ContactForce(10) - (rk_.ee_[0].cocp_ntact(2) - P_(2)) * ContactForce(6) + rk_.ee_[0].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            //zmp_pos(1) = (ContactForce(3) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(1) + rk_.ee_[1].cp_(1) * ContactForce(2) + ContactForce(9) - (rk_.ee_[0].cp_(2) - P_(2)) * ContactForce(7) + rk_.ee_[0].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[1].contact) //right contact
        {
            zmp_pos(0) = Robot.ZMP_r(0);
            zmp_pos(1) = Robot.ZMP_r(1);
        }
    }*/
    //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //std::cout << "ZMP position : " << zmp_pos(0) << "\t" << zmp_pos(1) << "\t" << zmp_pos(2) << " " << std::endl;
    //printf("ZMP position : %8.4f  %8.4f  %8.4f  max : %8.4f  %8.4f  %8.4f  min : %8.4f  %8.4f  %8.4f\n", zmp_pos(0), zmp_pos(1), zmp_pos(2), zmp_pos_max(0), zmp_pos_max(1), zmp_pos_max(2), zmp_pos_min(0), zmp_pos_min(1), zmp_pos_min(2));
    for (int i = 0; i < 3; i++)
    {
        if (zmp_pos_max(i) < zmp_pos(i))
            zmp_pos_max(i) = zmp_pos(i);
        if (zmp_pos_min(i) > zmp_pos(i))
            zmp_pos_min(i) = zmp_pos(i);
    }

    if (Robot.control_time_ < 0.01)
    {
        zmp_pos_min.setZero();
        zmp_pos_max.setZero();
    }
    return (zmp_pos);
}

Vector3d WholebodyController::GetZMPpos_fromFT(RobotData &Robot, bool Local)
{
    Vector3d zmp_pos;
    Vector3d P_;
    zmp_pos.setZero();
    P_.setZero();
    static Vector3d zmp_pos_max = (Eigen::Vector3d() << 0, 0, 0).finished();
    static Vector3d zmp_pos_min = (Eigen::Vector3d() << 0, 0, 0).finished();

    if (Local)
    {
        zmp_pos(0) = (-Robot.ContactForce_FT(4) - (Robot.ee_[0].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(0) + Robot.ee_[0].sensor_xpos(0) * Robot.ContactForce_FT(2) - Robot.ContactForce_FT(10) - (Robot.ee_[1].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(6) + Robot.ee_[1].sensor_xpos(0) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
        zmp_pos(1) = (Robot.ContactForce_FT(3) - (Robot.ee_[0].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(1) + Robot.ee_[0].sensor_xpos(1) * Robot.ContactForce_FT(2) + Robot.ContactForce_FT(9) - (Robot.ee_[1].sensor_xpos(2) - P_(2)) * Robot.ContactForce_FT(7) + Robot.ee_[1].sensor_xpos(1) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
    }
    else
    {
        Vector3d zmp_r, zmp_l;
        //std::cout << "sensor xpos x : " << Robot.ee_[0].sensor_xpos(0) << " ee_ xpos x : " << Robot.ee_[0].xpos(0) << " ee_cp_ : " << Robot.ee_[0].cp_(0) << std::endl;
        zmp_l(0) = Robot.ee_[0].cp_(0) + (-Robot.ContactForce_FT(4) - Robot.ContactForce_FT(0) * (Robot.ee_[0].cp_(2) - Robot.ee_[0].cp_(2))) / Robot.ContactForce_FT(2);
        zmp_l(1) = Robot.ee_[0].cp_(1) + (Robot.ContactForce_FT(3) - Robot.ContactForce_FT(1) * (Robot.ee_[0].cp_(2) - Robot.ee_[0].cp_(2))) / Robot.ContactForce_FT(2);

        zmp_r(0) = Robot.ee_[1].cp_(0) + (-Robot.ContactForce_FT(4 + 6) - Robot.ContactForce_FT(0 + 6) * (Robot.ee_[1].cp_(2) - Robot.ee_[1].cp_(2))) / Robot.ContactForce_FT(2 + 6);
        zmp_r(1) = Robot.ee_[1].cp_(1) + (Robot.ContactForce_FT(3 + 6) - Robot.ContactForce_FT(1 + 6) * (Robot.ee_[1].cp_(2) - Robot.ee_[1].cp_(2))) / Robot.ContactForce_FT(2 + 6);

        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            zmp_pos(0) = (zmp_l(0) * Robot.ContactForce_FT(2) + zmp_r(0) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
            zmp_pos(1) = (zmp_l(1) * Robot.ContactForce_FT(2) + zmp_r(1) * Robot.ContactForce_FT(8)) / (Robot.ContactForce_FT(2) + Robot.ContactForce_FT(8));
        }
        else if (Robot.ee_[0].contact) //left contact
        {
            //std::cout << " f0 : " << rk_.ContactForce(0) << " f1 : " << rk_.ContactForce(1) << " f2 : " << rk_.ContactForce(2) << " f3 : " << rk_.ContactForce(3) << " f4 : " << rk_.ContactForce(4) << " f5 : " << rk_.ContactForce(5) << std::endl;
            //std::cout<<"rk_.ContactForce(4) : "<<rk_.ContactForce(4)<<"rk_.ContactForce(2)"
            //std::cout << "x : " << rk_.ContactForce(4) / rk_.ContactForce(2) << "\t";
            //std::cout << "y : " << rk_.ContactForce(3) / rk_.ContactForce(2) << "\t cp x: " << rk_.link_[Left_Foot].xpos_contact(0) << "\t cp y : " << rk_.link_[Left_Foot].xpos_contact(1) << std::endl;
            zmp_pos(0) = zmp_l(0);
            zmp_pos(1) = zmp_l(1);
            //zmp_pos(0) = (-ContactForce(4) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(0) + rk_.ee_[1].cp_(0) * ContactForce(2) - ContactForce(10) - (rk_.ee_[0].cocp_ntact(2) - P_(2)) * ContactForce(6) + rk_.ee_[0].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            //zmp_pos(1) = (ContactForce(3) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(1) + rk_.ee_[1].cp_(1) * ContactForce(2) + ContactForce(9) - (rk_.ee_[0].cp_(2) - P_(2)) * ContactForce(7) + rk_.ee_[0].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[1].contact) //right contact
        {
            zmp_pos(0) = zmp_r(0);
            zmp_pos(1) = zmp_r(1);
        }
    }
    //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //std::cout << "ZMP position : " << zmp_pos(0) << "\t" << zmp_pos(1) << "\t" << zmp_pos(2) << " " << std::endl;
    //printf("ZMP position : %8.4f  %8.4f  %8.4f  max : %8.4f  %8.4f  %8.4f  min : %8.4f  %8.4f  %8.4f\n", zmp_pos(0), zmp_pos(1), zmp_pos(2), zmp_pos_max(0), zmp_pos_max(1), zmp_pos_max(2), zmp_pos_min(0), zmp_pos_min(1), zmp_pos_min(2));
    for (int i = 0; i < 3; i++)
    {
        if (zmp_pos_max(i) < zmp_pos(i))
            zmp_pos_max(i) = zmp_pos(i);
        if (zmp_pos_min(i) > zmp_pos(i))
            zmp_pos_min(i) = zmp_pos(i);
    }

    if (Robot.control_time_ < 0.01)
    {
        zmp_pos_min.setZero();
        zmp_pos_max.setZero();
    }
    return (zmp_pos);
}

Vector3d WholebodyController::GetZMPpos(RobotData &Robot, VectorXd ContactForce, bool Local)
{
    Vector3d zmp_pos;
    Vector3d P_;
    zmp_pos.setZero();
    P_.setZero();
    static Vector3d zmp_pos_max = (Eigen::Vector3d() << 0, 0, 0).finished();
    static Vector3d zmp_pos_min = (Eigen::Vector3d() << 0, 0, 0).finished();

    Vector3d RightFootPos, LeftFootPos;

    LeftFootPos = Robot.ee_[0].cp_;
    RightFootPos = Robot.ee_[1].cp_;

    if (Local)
    {
        zmp_pos(0) = (-ContactForce(4) - (Robot.ee_[0].cp_(2) - P_(2)) * ContactForce(0) + Robot.ee_[0].cp_(0) * ContactForce(2) - ContactForce(10) - (Robot.ee_[1].cp_(2) - P_(2)) * ContactForce(6) + Robot.ee_[1].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        zmp_pos(1) = (ContactForce(3) - (Robot.ee_[0].cp_(2) - P_(2)) * ContactForce(1) + Robot.ee_[0].cp_(1) * ContactForce(2) + ContactForce(9) - (Robot.ee_[1].cp_(2) - P_(2)) * ContactForce(7) + Robot.ee_[1].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
    }
    else
    {
        Vector3d zmp_r, zmp_l;
        //std::cout << "sensor xpos x : " << Robot.ee_[0].sensor_xpos(0) << " ee_ xpos x : " << Robot.ee_[0].xpos(0) << " ee_cp_ : " << Robot.ee_[0].cp_(0) << std::endl;
        zmp_l(0) = Robot.ee_[0].cp_(0) + (-ContactForce(4) - ContactForce(0) * (Robot.ee_[0].cp_(2) - Robot.ee_[0].cp_(2))) / ContactForce(2);
        zmp_l(1) = Robot.ee_[0].cp_(1) + (ContactForce(3) - ContactForce(1) * (Robot.ee_[0].cp_(2) - Robot.ee_[0].cp_(2))) / ContactForce(2);

        zmp_r(0) = Robot.ee_[1].cp_(0) + (-ContactForce(4 + 6) - ContactForce(0 + 6) * (Robot.ee_[1].cp_(2) - Robot.ee_[1].cp_(2))) / ContactForce(2 + 6);
        zmp_r(1) = Robot.ee_[1].cp_(1) + (ContactForce(3 + 6) - ContactForce(1 + 6) * (Robot.ee_[1].cp_(2) - Robot.ee_[1].cp_(2))) / ContactForce(2 + 6);

        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            zmp_pos(0) = (zmp_l(0) * ContactForce(2) + zmp_r(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            zmp_pos(1) = (zmp_l(1) * ContactForce(2) + zmp_r(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[0].contact) //left contact
        {
            //std::cout << " f0 : " << rk_.ContactForce(0) << " f1 : " << rk_.ContactForce(1) << " f2 : " << rk_.ContactForce(2) << " f3 : " << rk_.ContactForce(3) << " f4 : " << rk_.ContactForce(4) << " f5 : " << rk_.ContactForce(5) << std::endl;
            //std::cout<<"rk_.ContactForce(4) : "<<rk_.ContactForce(4)<<"rk_.ContactForce(2)"
            //std::cout << "x : " << rk_.ContactForce(4) / rk_.ContactForce(2) << "\t";
            //std::cout << "y : " << rk_.ContactForce(3) / rk_.ContactForce(2) << "\t cp x: " << rk_.link_[Left_Foot].xpos_contact(0) << "\t cp y : " << rk_.link_[Left_Foot].xpos_contact(1) << std::endl;
            zmp_pos(0) = zmp_l(0);
            zmp_pos(1) = zmp_l(1);
            //zmp_pos(0) = (-ContactForce(4) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(0) + rk_.ee_[1].cp_(0) * ContactForce(2) - ContactForce(10) - (rk_.ee_[0].cocp_ntact(2) - P_(2)) * ContactForce(6) + rk_.ee_[0].cp_(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
            //zmp_pos(1) = (ContactForce(3) - (rk_.ee_[1].cp_(2) - P_(2)) * ContactForce(1) + rk_.ee_[1].cp_(1) * ContactForce(2) + ContactForce(9) - (rk_.ee_[0].cp_(2) - P_(2)) * ContactForce(7) + rk_.ee_[0].cp_(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
        }
        else if (Robot.ee_[1].contact) //right contact
        {
            zmp_pos(0) = zmp_r(0);
            zmp_pos(1) = zmp_r(1);
        }
    }

    //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
    //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));

    //std::cout << dc.time << "ZMP position : " << zmp_pos(0) << "\t" << zmp_pos(1) << "\t" << zmp_pos(2) << " " << std::endl;
    //printf("ZMP position : %8.4f  %8.4f  %8.4f  max : %8.4f  %8.4f  %8.4f  min : %8.4f  %8.4f  %8.4f\n", zmp_pos(0), zmp_pos(1), zmp_pos(2), zmp_pos_max(0), zmp_pos_max(1), zmp_pos_max(2), zmp_pos_min(0), zmp_pos_min(1), zmp_pos_min(2));
    for (int i = 0; i < 3; i++)
    {
        if (zmp_pos_max(i) < zmp_pos(i))
            zmp_pos_max(i) = zmp_pos(i);
        if (zmp_pos_min(i) > zmp_pos(i))
            zmp_pos_min(i) = zmp_pos(i);
    }

    if (Robot.control_time_ < 0.01)
    {
        zmp_pos_min.setZero();
        zmp_pos_max.setZero();
    }
    return (zmp_pos);
}

void WholebodyController::ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{
    Eigen::MatrixXd W;
    W.setZero(6, 12);

    Eigen::Matrix3d P1_hat, P2_hat;
    P1_hat = DyrosMath::skm(P1);
    P2_hat = DyrosMath::skm(P2);

    for (int i = 0; i < 3; i++)
    {
        W(i, i) = 1.0;
        W(i + 3, i + 3) = 1.0;
        W(i, i + 6) = 1.0;
        W(i + 3, i + 9) = 1.0;

        for (int j = 0; j < 3; j++)
        {
            W(i + 3, j) = P1_hat(i, j);
            W(i + 3, j + 6) = P2_hat(i, j);
        }
    }
    ResultantForce.resize(6);
    ResultantForce = W * F12; //F1F2;

    double eta_lb = 1.0 - eta_cust;
    double eta_ub = eta_cust;
    //printf("1 lb %f ub %f\n",eta_lb,eta_ub);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta Mx, A*eta + B < 0
    double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
    double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
    double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
    double a = A * A;
    double b = 2.0 * A * B;
    double c = B * B - C * C;
    double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
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
    else //sol_eta2 ÀÌ upper boundary
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

    //printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta My, A*eta + B < 0
    A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
    B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
    C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
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
    else //sol_eta2 ÀÌ upper boundary
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

    //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
    A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
    B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
    C = staticFrictionCoeff * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;
    sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
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
    else //sol_eta2 ÀÌ upper boundary
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
    //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

    double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

    eta = eta_s;
    if (eta_s > eta_ub)
    {
        eta = eta_ub;
    }
    else if (eta_s < eta_lb)
    {
        eta = eta_lb;
    }

    if ((eta > eta_cust) || (eta < 1.0 - eta_cust))
    {
        eta = 0.5;
    }

    //std::cout<<"ETA :: "<<eta<<std::endl;

    //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

    //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
    //double etaMx = eta*Mx1Mx2;
    //printf("%f %f \n", Mx1Mx2,etaMx);
    //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
    //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
    //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

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
    //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
    //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
    //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

void WholebodyController::ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{

    Eigen::Matrix3d P1_hat, P2_hat;
    P1_hat = DyrosMath::skm(P1);
    P2_hat = DyrosMath::skm(P2);

    Eigen::MatrixXd W;
    W.setZero(6, 12);

    W.block(0, 0, 6, 6) = Eigen::Matrix6d::Identity();
    W.block(0, 6, 6, 6) = Eigen::Matrix6d::Identity();
    W.block(3, 0, 3, 3) = P1_hat;
    W.block(3, 6, 3, 3) = P2_hat;

    // link_[Right_Leg].Rotm;

    // for (int i = 0; i < 3; i++)
    // {
    //   W(i, i) = 1.0;
    //   W(i + 3, i + 3) = 1.0;
    //   W(i, i + 6) = 1.0;
    //   W(i + 3, i + 9) = 1.0;

    //   for (int j = 0; j < 3; j++)
    //   {
    //     W(i + 3, j) = P1_hat(i, j);
    //     W(i + 3, j + 6) = P2_hat(i, j);
    //   }
    // }

    ResultantForce.resize(6);
    ResultantForce = W * F12; //F1F2;

    double eta_lb = 1.0 - eta_cust;
    double eta_ub = eta_cust;
    double A_threshold = 0.001;
    ////printf("1 lb %f ub %f\n",eta_lb,eta_ub);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta Mx, A*eta + B < 0
    double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
    double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
    double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
    double a = A * A;
    double b = 2.0 * A * B;
    double c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0.");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1.");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2.");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3.");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4.");
            }
        }
    }

    ////printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta My, A*eta + B < 0
    A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
    B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
    C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0;");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1;");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2;");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3;");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4;");
            }
        }
    }

    //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
    A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
    B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
    C = staticFrictionCoeff * abs(ResultantForce(2));
    a = A * A;
    b = 2.0 * A * B;
    c = B * B - C * C;

    if (abs(A) < A_threshold)
    {
        if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
        {
        }
        else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
        {
            //printf("0,");
        }
    }
    else
    {
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
        {
            if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
            {
                eta_ub = sol_eta1;
            }
            else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("1,");
            }

            if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
            {
                eta_lb = sol_eta2;
            }
            else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("2,");
            }
        }
        else //sol_eta2 이 upper boundary
        {
            if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
            {
                eta_ub = sol_eta2;
            }
            else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
            {
            }
            else
            {
                //printf("3,");
            }

            if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
            {
                eta_lb = sol_eta1;
            }
            else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
            {
            }
            else
            {
                //printf("4,");
            }
        }
    }
    //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

    double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

    if (eta_s > eta_ub)
    {
        eta = eta_ub;
    }
    else if (eta_s < eta_lb)
    {
        eta = eta_lb;
    }
    else
    {
        eta = eta_s;
    }

    if (eta_ub < eta_lb) //임시...roundoff error로 정확한 해가 안나올때
    {
        //printf("-");
    }
    else if (sqrt(eta_ub * eta_ub + eta_lb * eta_lb) > 1.0) //너무 큰 경계값이 섞여 있을 때
    {
        //printf("_");
    }

    //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

    //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
    //double etaMx = eta*Mx1Mx2;
    //printf("%f %f \n", Mx1Mx2,etaMx);
    //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
    //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
    //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

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
    //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
    //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
    //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

Vector3d WholebodyController::COM_traj_with_zmp(RobotData &Robot)
{
    double tc = sqrt(Robot.com_.pos(2) / 9.81);

    //rk_.link_[COM_id].x_traj(1) = ()
    //rk_.link_[COM_id].a_traj = 9.81/rk_.com_.pos(2)*()
}
