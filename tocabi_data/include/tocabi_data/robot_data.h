#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <ros/ros.h>

#include "tocabi_data/link.h"
#include "tocabi_ecat/shm_msgs.h"
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

    ///////////////////////////////////////////

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    ///////////
    Eigen::VectorVQd q_dot_virtual_lpf;
    Eigen::VectorQd q_ext_;
    Eigen::VectorQd q_dot_est_;
    Eigen::VectorQd q_dot_est_1;
    Eigen::VectorQd q_hold_lower_;
    Eigen::VectorQd torque_elmo_;

    Eigen::VectorQd torque_desired;

    ///////////////////////////
    //Dynamics Data

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

    VectorVQd non_linear;
    Vector3d grav_ref;

    double total_mass_ = 0;

    //EndEffector ee_[ENDEFFECTOR_NUMBER]; //ee_ : 0: Left 1: Right

    int contact_index = 0;
    int contact_part[4] = {-1, -1, -1, -1};
    int ee_idx[4] = {-1, -1, -1, -1};

    //Task Command
    tocabi_msgs::TaskCommand tc_;
    atomic<bool> task_signal_{};
    tocabi_msgs::TaskCommandQue tc_q_;
    atomic<bool> task_que_signal_{};

    //Bools...... might be moved to other..
    bool qp_error = false;

    bool task_control_switch = false;
    bool lambda_calc = false;
    bool init_qp = false;
    bool data_print_switch = false;
    bool data_print = false;
    bool zmp_feedback_control = false;
    bool check = false;
    bool qp2nd = false;
    bool signal_yaw_init = false;
    volatile bool firstCalc = false;

    // bool contact_calc;
    // bool task_force_control;
    // bool task_force_control_feedback;
    // bool zmp_control;
    // bool mpc_init;
    // bool showdata;
    // bool target_arrived_[4];
    // bool debug;
};

struct DataContainer
{
    ~DataContainer() { std::cout << "DC terminate" << std::endl; }
    ros::NodeHandle nh;
    RobotData rd_;

    double total_mass_ = 0;

    bool simMode = false;
    SHMmsgs *tc_shm_;

    bool torqueOnSwitch = false;
    bool torqueOffSwitch = false;
    bool emergencySwitch = false;
    bool torqueRisingSeq = false;
    bool toruqeDecreaseSeq = false;
    bool torqueOn = false;

    bool inityawSwitch = false;
    bool ftcalibSwtich = false;
    bool imuResetSwtich = false;
    bool stateEstimateSwitch = false;
    bool safetyResetSwitch = false;

    double torqueOnTime = -1;
    double torqueOffTime = -1;

};

#endif