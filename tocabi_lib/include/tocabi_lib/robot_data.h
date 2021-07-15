#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <atomic>

#include "tocabi_lib/link.h"
#include "shm_msgs.h"
#include "tocabi_msgs/TaskCommandQue.h"
#include "tocabi_msgs/TaskCommand.h"

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

struct RobotData
{
    ~RobotData() { std::cout << "rd terminate" << std::endl; }
    /////////////////////////////////////////////////
    ///////////REFRESHING VARIABLES START////////////

    std::atomic<int> us_from_start_{};
    float control_time_ = 0;

    RigidBodyDynamics::Model model_;
    LinkData link_[LINK_NUMBER + 1];
    EndEffector ee_[ENDEFFECTOR_NUMBER];

    Eigen::MatrixVVd A_;
    Eigen::MatrixVVd A_inv_;
    Eigen::MatrixVVd Motor_inertia;
    Eigen::MatrixVVd Motor_inertia_inverse;

    Eigen::VectorQd q_;
    Eigen::VectorQd q_dot_;

    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;

    Eigen::Vector3d imu_lin_acc;
    Eigen::Vector3d imu_ang_vel;
    Eigen::Vector3d imu_ang_vel_before;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    Eigen::VectorVQd q_dot_virtual_lpf;
    Eigen::VectorQd torque_elmo_;
    Eigen::VectorQd q_ext_;
    Eigen::VectorQd q_dot_est_;
    Eigen::VectorQd q_dot_est_1;
    Eigen::VectorQd q_hold_lower_;

    Eigen::Vector6d LF_FT, RF_FT;       //f/t data with local sensor frame
    Eigen::Vector6d LF_CF_FT, RF_CF_FT; //f/t data with global rotation frame

    ///////////////////////////////////////////////
    ///////////REFRESHING VARIABLES END////////////

    Eigen::VectorQd q_desired;
    Eigen::VectorQd q_dot_desired;
    Eigen::VectorQd torque_desired;

    ///////////////////////////////////////////////
    //////////////GAIN VARAIBLES///////////////////
    std::vector<double> pos_kp_v;
    std::vector<double> pos_kv_v;
    Eigen::VectorQd pos_kp;
    Eigen::VectorQd pos_kd;

    ///////////////////////////
    /////Dynamics Data

    // task_dof * MODEL_DOF_VIRTUAL
    MatrixXd J_task;
    MatrixXd J_task_T;
    MatrixXd J_task_inv_T;

    // task_dof X task_dof
    MatrixXd lambda_inv;
    MatrixXd lambda;
    MatrixXd Q;
    MatrixXd Q_T_;
    MatrixXd Q_temp;
    MatrixXd Q_temp_inv;

    //contact_dof * MODEL_DOF_VIRTUAL
    MatrixXd J_C;
    MatrixXd J_C_INV_T;

    MatrixXd I_C;

    MatrixVVd N_C;
    MatrixQQd W, W_inv;
    MatrixXd Lambda_c;
    MatrixXd qr_V2;

    VectorVQd G;
    MatrixXd P_C;

    VectorQd torque_grav;
    VectorQd torque_contact;

    Vector12d fc_redist_;

    VectorVQd non_linear;
    Vector3d grav_ref;

    double total_mass_ = 0;

    //EndEffector ee_[ENDEFFECTOR_NUMBER]; //ee_ : 0: Left 1: Right

    double yaw_init = 0;

    int contact_index = 0;
    int contact_part[4] = {-1, -1, -1, -1};
    int ee_idx[4] = {-1, -1, -1, -1};

    //Task Command
    tocabi_msgs::TaskCommand tc_;
    atomic<bool> task_signal_{};
    tocabi_msgs::TaskCommandQue tc_q_;
    atomic<bool> task_que_signal_{};
    bool tc_init = false;
    bool tc_run = false;
    double tc_time_;

    bool pc_mode = false;
    bool pc_gravity = false;
    VectorQd pc_pos_des;
    VectorQd pc_pos_init;
    double pc_traj_time_;
    double pc_time_;

    //Bools...... might be moved to other..
    bool qp_error = false;
    bool task_control_switch = false;
    bool lambda_calc = false;
    bool init_qp = false;
    bool data_print_switch = false;
    bool data_print = false;
    bool zmp_feedback_control = false;
    bool positionHoldSwitch = false;
    bool check = false;
    bool qp2nd = false;
    bool signal_yaw_init = false;
    volatile bool firstCalc = false;

    bool semode = false;
    bool semode_init = false;

    double state_ctime_total_ = 0;
    double state_ctime_avg_ = 0;

    std::chrono::steady_clock::time_point rc_t_;

    std::chrono::steady_clock::time_point tp_state_;
    // bool contact_calc;
    // bool task_force_control;
    // bool task_force_control_feedback;
    // bool zmp_control;
    // bool mpc_init;
    // bool showdata;
    // bool target_arrived_[4];
    // bool debug;

    ////Temp Sector////
    double time_for_inverse = 0;
    double time_for_inverse_total = 0;
    int count_for_inverse = 0;
    int count_for_inverse_total = 0;

    ///////////////////
};

struct DataContainer
{
    ~DataContainer() { std::cout << "DC terminate" << std::endl; }
    ros::NodeHandle nh;
    RobotData rd_;

    std::vector<float> Kps;
    std::vector<float> Kvs;

    std::atomic<bool> t_c_;
    double torque_command[MODEL_DOF];

    double total_mass_ = 0;

    bool simMode = false;
    SHMmsgs *tc_shm_;

    bool torqueOnSwitch = false;
    bool torqueOffSwitch = false;
    bool emergencySwitch = false;
    bool emergencyStatus = false;
    bool E1Switch = false;
    bool E1Status = false;
    bool E2Switch = false;
    bool E2Status = false;
    bool torqueRisingSeq = false;
    bool toruqeDecreaseSeq = false;
    bool torqueOn = false;

    bool inityawSwitch = false;
    bool ftcalibSwtich = false;
    bool imuResetSwtich = false;
    bool stateEstimateSwitch = false;
    bool safetyResetSwitch = false;

    bool positionControlSwitch = false;

    bool useSimVirtual = false;

    double torqueOnTime = -1;
    double torqueOffTime = -1;

    atomic<bool> triggerThread1;
};

#endif
