#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <algorithm>
#include <thread>
#include <future>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/JointState.h>

#include "tocabi_lib/robot_data.h"
#include "mujoco_ros_msgs/SimStatus.h"
#include "mujoco_ros_msgs/JointSet.h"
#include "tocabi_msgs/positionCommand.h"

class StateManager
{
public:
    StateManager(DataContainer &dc_global);
    ~StateManager();

    void *StateThread();
    static void *ThreadStarter(void *context) { return ((StateManager *)context)->StateThread(); }

    void *LoggerThread();
    static void *LoggerStarter(void *context) { return ((StateManager *)context)->LoggerThread(); }

    void SendCommand();

    void GetJointData();
    void InitYaw();
    void GetSensorData();
    void StoreState(RobotData &robotd_);
    void CalcNonlinear();

    void StateEstimate();
    //private functions

    void UpdateKinematics_local(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);
    //update kinematic information with RBDL
    void UpdateKinematics(RigidBodyDynamics::Model &model_l, LinkData *link_p, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);

    void UpdateCMM(RobotData &robotd_, LinkData *link_p);

    void PublishData();

    LinkData link_[LINK_NUMBER + 1];
    LinkData link_local_[LINK_NUMBER + 1];
    int link_id_[LINK_NUMBER + 1];

    Eigen::MatrixVVd A_;
    Eigen::MatrixVVd A_inv_;
    Eigen::MatrixXd A_temp_;
    Eigen::MatrixVVd Motor_inertia;
    Eigen::MatrixVVd Motor_inertia_inverse;

    RigidBodyDynamics::Model model_local_, model_global_;

    DataContainer &dc_;
    RobotData &rd_gl_;
    RobotData rd_;

    float q_a_[MODEL_DOF] = {};
    float q_dot_a_[MODEL_DOF] = {};
    int joint_state_[MODEL_DOF];
    int joint_state_before_[MODEL_DOF];

    int8_t state_elmo_[MODEL_DOF];
    int8_t state_safety_[MODEL_DOF];
    int8_t state_zp_[MODEL_DOF];

    int8_t state_elmo_before_[MODEL_DOF];
    int8_t state_safety_before_[MODEL_DOF];
    int8_t state_zp_before_[MODEL_DOF];

    Eigen::VectorQVQd q_virtual_local_;
    Eigen::VectorVQd q_dot_virtual_local_;
    Eigen::VectorVQd q_ddot_virtual_local_;
    Eigen::VectorQVQd q_virtual_local_yaw_initialized;

    Eigen::Vector6d RF_FT, LF_FT;
    Eigen::Vector6d LF_CF_FT;
    Eigen::Vector6d RF_CF_FT;

    Eigen::Vector3d LF_CP_est;
    Eigen::Vector3d RF_CP_est;

    double rf_s_ratio, lf_s_ratio;

    //Eigen::VectorXf<MODEL_DOF_VIRTUAL> q_virtual_;
    Eigen::VectorQd q_;
    Eigen::VectorQd q_dot_;
    Eigen::VectorQd q_ddot_;

    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;

    float control_time_ = 0;

    double torqueRatio = 0.0;

    double total_mass_ = 0;
    tf2_ros::TransformBroadcaster br;

    //Calc performance measuring...

    void MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2 = 0);
    int64_t total1 = 0, total2 = 0, total_dev1 = 0, total_dev2 = 0;
    float lmax = 0.0, lmin = 10000.00, ldev = 0.0, lavg = 0.0, lat = 0.0;
    float smax = 0.0, smin = 10000.00, sdev = 0.0, savg = 0.0, sat = 0.0;

    //Simmode values..

    void ConnectSim();
    void GetSimData();
    void SendCommand(VectorQf command, std::vector<bool> command_mode);

    ros::Publisher timer_pub_;
    std_msgs::Float32 timer_msg_;

    ros::Subscriber mujoco_sim_command_sub_;
    ros::Publisher mujoco_sim_command_pub_;

    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_msg_;

    ros::Publisher point_pub_;
    geometry_msgs::PolygonStamped point_pub_msg_;

    ros::Publisher status_pub_;

    ros::Publisher head_pose_pub_;
    geometry_msgs::Pose head_pose_msg_;

    ros::Publisher elmo_status_pub_;
    std_msgs::Int8MultiArray elmo_status_msg_;

    ros::Publisher com_status_pub_;
    std_msgs::Float32MultiArray com_status_msg_;

    void SimCommandCallback(const std_msgs::StringConstPtr &msg);
    //void simStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg);

    ros::Subscriber gui_command_sub_;
    ros::Publisher gui_state_pub_;
    std_msgs::Int8MultiArray syspub_msg;

    void GuiCommandCallback(const std_msgs::StringConstPtr &msg);

    void StatusPub(const char *str, ...);

    float sim_time_ = 0;
    float sim_time_before_ = 0;

    bool mujoco_ready = false;
    bool mujoco_init_receive = false;
    bool mujoco_reset = false;
};

#endif