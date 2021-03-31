#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <ros/ros.h>

#include "tocabi_controller/link.h"
#include "tocabi_ecat/shm_msgs.h"

using namespace std;

// RobotData only contains Robot kinematics Data.
// Must be calculated every tick?
// ... hmm....
// ....... hmm....
/*
struct EndEffector
{
    Eigen::Vector3d cp_;
    Eigen::Vector3d xpos;
    Eigen::Vector3d sensor_xpos;
    Eigen::Vector6d contact_force;
    Eigen::Vector6d contact_force_ft;
    Eigen::Matrix3d rotm;
    Eigen::Vector3d zmp;
    double contact_accuracy;
    double contact_time;
    int contact_transition_mode; //-1 nothing to do, 0 disabling, 1 enabling
    double minimum_press_force;
    double friction_ratio;
    double friction_ratio_z;
    double cs_x_length;
    double cs_y_length;
    bool contact = false;
}; */

//Shared Data from stateManager

struct RobotKinematicsData
{
    float control_time_;
    RigidBodyDynamics::Model model_;
    LinkData link_[LINK_NUMBER + 1];

    Eigen::MatrixVVf A_;
    Eigen::MatrixVVf A_inv_;
    Eigen::MatrixVVf Motor_inertia;
    Eigen::MatrixVVf Motor_inertia_inverse;

    Eigen::VectorQd q_;
    Eigen::VectorQd q_dot_;

    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;
};

struct RobotData_origin
{
    //Com com_;
    LinkData link_[LINK_NUMBER + 1];
    double orientation;
    double roll, pitch, yaw;
    double yaw_init = 0.0;
    double total_mass;

    //PositionPDGain
    double Kps[MODEL_DOF];
    double Kvs[MODEL_DOF];
    std::vector<double> vector_kp, vector_kv, vector_NM2CNT;
    std::vector<double> v_com_kp, v_com_kv, v_up_kp, v_up_kv, v_pelv_kp, v_pelv_kv, v_hand_kp, v_hand_kv, v_foot_kp, v_foot_kv;

    Eigen::VectorQd q_desired_;
    Eigen::VectorQd q_dot_desired_;
    Eigen::VectorQd q_;
    Eigen::VectorQd q_init_;
    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorQd q_dot_;
    Eigen::VectorQd q_dot_est;
    Eigen::VectorQd q_dot_est1;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;
    Eigen::VectorVQd q_dot_virtual_lpf_;
    Eigen::VectorQd q_ext_;

    Eigen::VectorQd q_dot_before_;
    Eigen::VectorQd q_dot_diff_;

    Eigen::VectorQd q_ddot_estimate_;

    Eigen::VectorXd ContactForce;
    Eigen::VectorXd ContactForce_qp;
    Eigen::VectorXd torque_qp;
    Eigen::VectorXd qacc_qp;
    Eigen::Vector12d ContactForce_FT;
    Eigen::Vector12d ContactForce_FT_raw;
    Eigen::Vector12d CF_temp;
    Eigen::Vector6d LH_FT, RH_FT;
    Eigen::Vector3d ZMP;
    Eigen::Vector3d ZMP_local;
    Eigen::Vector3d ZMP_desired;
    Eigen::Vector3d ZMP_desired2;
    Eigen::Vector3d ZMP_ft;
    Eigen::Vector3d ZMP_error;
    Eigen::Vector3d ZMP_eqn_calc;
    Eigen::Vector3d ZMP_command;
    Eigen::Vector3d ZMP_mod;
    Eigen::Vector3d ZMP_r;
    Eigen::Vector3d ZMP_l;
    Eigen::Vector3d CP_;
    Eigen::Vector3d CP_desired;
    Eigen::VectorXd TaskForce;

    double com_time;

    //std::future<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> winv_ret;

    Eigen::Vector3d fstar;

    double com_vel_limit;
    double com_acc_limit;

    //bool contact_[ENDEFFECTOR_NUMBER] = {true, true};

    //ee_ : Left to Right
    EndEffector ee_[ENDEFFECTOR_NUMBER]; //ee_ : 0: Left 1: Right

    int contact_index;
    int contact_part[4];
    int ee_idx[4];

    double contact_transition_time;

    double control_time_; // updated by control_base
    double control_time_pre_;
    double d_time_;

    double friction_ratio;

    double start_time_[4];
    double end_time_[4];

    Eigen::MatrixXd task_selection_matrix;
    Eigen::VectorXd task_desired_force;
    Eigen::VectorXd task_feedback_reference;
    Eigen::Vector2d ZMP_task;

    double zmp_gain;

    Eigen::MatrixVVd A_matrix;
    Eigen::MatrixVVd A_;
    Eigen::MatrixVVd A_matrix_inverse;
    Eigen::Matrix6Qd Ag_;
    Eigen::MatrixQQd Cor_;
    Eigen::MatrixQQd M_p;
    Eigen::VectorQd G_;

    Eigen::MatrixVVd Motor_inertia;
    Eigen::MatrixVVd Motor_inertia_inverse;
    Eigen::MatrixXd Lambda_c_motor;
    Eigen::MatrixXd J_task_inv_motor, J_task_inv_motor_T;
    Eigen::MatrixXd lambda_motor_inv, lambda_motor;
    Eigen::MatrixXd W_motor, W_motor_inv;
    Eigen::MatrixXd N_C_motor;
    Eigen::MatrixXd Q_motor, Q_motor_T_, Q_motor_temp, Q_motor_temp_inv; //, Jtemp, Jtemp_2;

    Eigen::MatrixXd J_C, J_C_INV_T;
    Eigen::MatrixXd J_COM;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF_VIRTUAL> J_g;

    Eigen::MatrixXd J_task;
    Eigen::VectorXd f_star;
    Eigen::VectorXd f_star_a_;

    Eigen::MatrixXd Lambda_c;
    Eigen::MatrixXd N_C;
    Eigen::VectorXd P_C;
    Eigen::MatrixVVd I37;

    Eigen::VectorXd contact_force_predict;
    Eigen::Vector3d Grav_ref;

    Eigen::MatrixXd J_task_T, J_task_inv, J_task_inv_T;
    Eigen::MatrixXd lambda_inv, lambda;
    Eigen::MatrixXd W, W_inv;
    Eigen::MatrixXd Q, Q_T_, Q_temp, Q_temp_inv, Jtemp, Jtemp_2;
    Eigen::MatrixXd _F;

    Eigen::VectorXd G;
    Eigen::VectorQd torque_grav_cc;
    Eigen::VectorQd torque_grav;
    Eigen::VectorQd torque_contact;
    Eigen::VectorQd torque_disturbance;
    Eigen::VectorQd torque_limit;

    Eigen::MatrixXd Slc_k, Slc_k_T;
    Eigen::MatrixXd svd_U;
    Eigen::MatrixXd qr_V2;
    Eigen::MatrixXd NwJw;
    Eigen::MatrixXd Scf_;

    int task_dof;

    Eigen::Vector2d p_k_1;
    Eigen::Vector3d ZMP_pos;

    Eigen::Vector3d imu_pos_;
    Eigen::Vector3d imu_vel_;

    //contact redistribution mode selector. 0 : yslee 1: qp 2: off
    int contact_redistribution_mode = 0;

    double fc_redis;

    bool qp_error = false;
    bool contact_calc;
    bool task_force_control;
    bool task_force_control_feedback;
    bool zmp_control;
    bool mpc_init;
    bool showdata;
    bool task_control_switch = false;
    bool target_arrived_[4];
    bool debug;
    bool lambda_calc = false;
    bool init_qp = false;
    bool data_print_switch = false;

    bool data_print = false;

    bool zmp_feedback_control = false;
    bool check = false;
    bool qp2nd = false;
    bool signal_yaw_init = false;

    int Right = 0;
    int Left = 1;

    RigidBodyDynamics::Model model_virtual;
};

struct RobotData
{
    float control_time_;
    RigidBodyDynamics::Model model_;
    LinkData link_[LINK_NUMBER + 1];
    EndEffector ee_[ENDEFFECTOR_NUMBER];

    Eigen::MatrixVVf A_;
    Eigen::MatrixVVf A_inv_;
    Eigen::MatrixVVf Motor_inertia;
    Eigen::MatrixVVf Motor_inertia_inverse;

    Eigen::VectorQd q_;
    Eigen::VectorQd q_dot_;

    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;

    /////////////////////////////////////////////

    Eigen::VectorVQd q_dot_virtual_lpf;
    Eigen::VectorQd q_ext_;
    Eigen::VectorQd q_dot_est_;
    Eigen::VectorQd q_dot_est_1;
    Eigen::VectorQd q_hold_lower_;
    Eigen::VectorQd torque_elmo_;

    Eigen::VectorQd torque_desired;

    //EndEffector ee_[ENDEFFECTOR_NUMBER]; //ee_ : 0: Left 1: Right

    int contact_index;
    int contact_part[4];
    int ee_idx[4];

    //Bools...... might be moved to other..
    bool qp_error = false;
    bool contact_calc;
    bool task_force_control;
    bool task_force_control_feedback;
    bool zmp_control;
    bool mpc_init;
    bool showdata;
    bool task_control_switch = false;
    bool target_arrived_[4];
    bool debug;
    bool lambda_calc = false;
    bool init_qp = false;
    bool data_print_switch = false;
    bool data_print = false;
    bool zmp_feedback_control = false;
    bool check = false;
    bool qp2nd = false;
    bool signal_yaw_init = false;
};

struct DataContainer
{
    ros::NodeHandle nh;
    RobotData rd_;

    SHMmsgs *shm_msgs_;
};