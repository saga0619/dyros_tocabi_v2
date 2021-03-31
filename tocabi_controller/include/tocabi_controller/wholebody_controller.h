#ifndef WHOLEBODY_CONTROLLER_H
#define WHOLEBODY_CONTROLLER_H

//#include "dyros_red_controller/dyros_red_model.h"
#include "tocabi_controller/robot_data.h"
#include "tocabi_controller/qp.h"
#include <vector>
#include <fstream>
using namespace Eigen;
using namespace std;
using namespace qpOASES;

/* Wholebody controller description :
* This library is made by junewhee ahn, based on theory by jaeheung park
* 
*
* task control how to use : 
* 1. set contact status with function 'Wholebody_controller::contact_set_multi(...)'
* 2. get gravity compensation torque with 'Wholebody_controller::gravity_compensation_torque(...)'
* 3. define task dof number
* 4. construct task jacobian 
* 5. define desired task position and time,
* 6. get trajectory of task with function 'link_.Set_Trajectory_from_quintic(....)' or other set_traj functions at link class!
*    initial position of each task can be determined through following function, link_[].Set_initpos()
* 7. get fstar with Wholebody_controller::getfstar6d(...)
* 8. get torque task with Wholebody_controller::task_control_torque(....)
*/

const double NM2CNT_d[MODEL_DOF] =
    {
        3.0, 4.3, 3.8, 3.46, 4.5, 12.33,
        3.0, 4.3, 3.8, 3.46, 4.5, 12.33,
        3.3, 3.3, 3.3,
        15.5, 15.5, 15.5, 15.5, 42.0, 42.0, 95.0, 95.0,
        95.0, 95.0,
        15.5, 15.5, 15.5, 15.5, 42.0, 42.0, 95.0, 95.0};

const double T_LIMIT[MODEL_DOF] =
    {
        1500.0 / 3.0, 1500.0 / 4.3, 1500.0 / 3.8, 1500.0 / 3.46, 1500.0 / 4.5, 1500.0 / 12.33,
        1500.0 / 3.0, 1500.0 / 4.3, 1500.0 / 3.8, 1500.0 / 3.46, 1500.0 / 4.5, 1500.0 / 12.33,
        1500.0 / 3.3, 1500.0 / 3.3, 1500.0 / 3.3,
        1500.0 / 15.5, 1500.0 / 15.5, 1500.0 / 15.5, 1500.0 / 15.5, 1500.0 / 42.0, 1500.0 / 42.0, 1500.0 / 95.0, 1500.0 / 95.0,
        1500.0 / 95.0, 1500.0 / 95.0,
        1500.0 / 15.5, 1500.0 / 15.5, 1500.0 / 15.5, 1500.0 / 15.5, 1500.0 / 42.0, 1500.0 / 42.0, 1500.0 / 95.0, 1500.0 / 95.0};

class WholebodyController
{
public:
  //const VectorQd &current_q_;
  //Main loop wholebody function
  // update kinematics information
  //
  void init(RobotData &Robot);
  void update(RobotData &Robot);

  //set contact status of robot. true for contact false for not contact
  void set_contact(RobotData &Robot);
  void set_contact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand = false, bool right_hand = false);
  void set_contact_multithread(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand = false, bool right_hand = false);

  void calc_winv(RobotData &Robot);
  void get_winv(RobotData &Robot);

  void set_robot_init(RobotData &Robot);
  void set_robot_init_multithread(RobotData &Robot);

  //contact force redistribution by yisoolee method at 2 contact(both foot)
  VectorQd contact_force_redistribution_torque(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta);

  VectorQd contact_force_redistribution_torque_walking(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta, double ratio, int supportFoot);

  //set contact force to desired contact force
  VectorQd contact_force_custom(RobotData &Robot, VectorQd command_torque, Eigen::VectorXd contact_force_now, Eigen::VectorXd contact_force_desired);

  //update gravity compensation torque
  VectorQd gravity_compensation_torque(RobotData &Robot, bool fixed = false, bool redsvd = false);

  VectorQd gravity_compensation_torque_QP(RobotData &Robot);

  //get contact redistribution torque with Quadratic programing
  VectorQd contact_torque_calc_from_QP(RobotData &Robot, VectorQd command_torque);
  VectorQd contact_torque_calc_from_QP2(RobotData &Robot, VectorQd command_torque);

  // Get Contact Redistribution Torque with QP. Wall contact mode.
  //VectorQd contact_torque_calc_from_QP_wall(VectorQd command_torque, double wall_friction_ratio);
  //Get Contact Redistribution Torque with QP. Wall contact mode.
  //VectorQd contact_torque_calc_from_QP_wall_mod2(VectorQd command_torque, double wall_friction_ratio);

  /*
  * Get Task Control Torque.
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);

  VectorQd task_control_torque_with_gravity(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_, bool force_control = false);

  // desired force  = lambda_task * f_star
  VectorQd task_control_torque_force_control(RobotData &Robot, MatrixXd J_task, VectorXd desiredForce);

  VectorQd task_control_torque(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_, int mode);

  VectorQd task_control_torque_motor(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  void copy_robot_fast(RobotData &Robot, RobotData &Robot_fast, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp);

  /*
  * Get Task Control Torque from QP.
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque_QP(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  VectorQd task_control_torque_QP2(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  VectorQd task_control_torque_QP3(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  VectorQd task_control_torque_QP2_with_contactforce_feedback(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  VectorQd task_control_torque_QP_gravity(RobotData &Robot);
  VectorQd task_control_torque_with_acc_cr(RobotData &Robot, MatrixXd J_task, VectorXd f_star_acc, VectorXd f_star_feedback);
  VectorXd check_fstar(RobotData &Robot, Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);
  Vector2d fstar_regulation(RobotData &Robot, Vector3d f_star);
  /*
  * Get Task Control Torque 
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque_custom_force(RobotData &Robot, MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force);

  VectorQd task_control_torque_hqp(RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp);

  VectorQd task_control_torque_hqp_threaded(RobotData Robot_thread, bool &init_qp);

  VectorQd task_control_torque_hqp_step(RobotData &Robot, MatrixXd &J_task, VectorXd &f_star);

  VectorXd hqp_contact_calc(CQuadraticProgram &qphqp, RobotData &Robot, VectorXd torque_prev, bool init);
  VectorXd hqp_contact_calc(CQuadraticProgram &qphqp, RobotData &Robot_fast, VectorXd torque_prev, bool init);
  VectorXd hqp_damping_calc(CQuadraticProgram &qphqp, RobotData &Robot_fast, VectorXd torque_prev, MatrixXd &Null_task, bool init);
  std::pair<VectorXd, VectorXd> hqp_step_calc(CQuadraticProgram &qphqp, RobotData &Robot, VectorXd torque_before, MatrixXd &Null_task, MatrixXd &Jkt, MatrixXd &lambda, VectorXd f_star, bool init);
  std::pair<VectorXd, VectorXd> hqp_step_calc(CQuadraticProgram &qphqp, RobotData &Robot_fast, VectorXd torque_prev, MatrixXd &Null_task, MatrixXd &Jkt, MatrixXd &lambda, VectorXd f_star, bool init);

  VectorQd task_control_torque_hqp2(RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp);
  std::pair<VectorXd, VectorXd> hqp2_step_calc(CQuadraticProgram &qphqp, RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp, bool init);
  //std::pair<VectorXd, VectorXd> hqp2_damping_calc(CQuadraticProgram &qphqp, RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp, bool init);
  VectorQd hqp2_contact_calc(CQuadraticProgram &qphqp, RobotData &Robot, std::vector<MatrixXd> &Jtask_hqp, std::vector<VectorXd> &fstar_hqp, bool init);
  // Get Task Control Torque task jacobian and f_star must be defined.
  VectorQd task_control_torque_custom_force_feedback(RobotData &Robot, MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand);

  //force control with selection matrix. selec 1 for control with fstar 0 for force control
  void set_force_control(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force);

  //force control selection matrix 1 for control with fstar 0 for force control
  void set_force_control_feedback(RobotData &Robot, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand);
  void set_zmp_control(RobotData &Robot, Vector2d ZMP, double gain);
  void set_zmp_feedback_control(RobotData &Data, Vector2d ZMP, bool &reset_error);
  void zmp_feedback_control(Vector3d desired_zmp);

  //Utility functions

  //Get contact force from command torque
  VectorXd get_contact_force(RobotData &Robot, VectorQd command_torque);
  MatrixXd GetTaskLambda(RobotData &Robot, MatrixXd J_task);
  //Get ZMP position from contact forces and both foot position
  Vector3d GetZMPpos(RobotData &Robot, bool Local = false);
  Vector3d GetZMPpos_fromFT(RobotData &Robot, bool Local = false);
  Vector3d GetZMPpos(RobotData &Robot, VectorXd ContactForce, bool Local = false);

  VectorQd footRotateAssist(RobotData &Robot, bool left = true, bool right = true);

  //Eigen::Vector6d Getfstar( );
  Vector3d getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now);
  Vector3d getfstar(RobotData &Robot, Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now);
  Vector3d getfstar_tra(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt);
  Vector3d getfstar_tra(RobotData &Robot, int link_id);
  Vector3d getfstar_rot(RobotData &Robot, int link_id, Vector3d kpa, Vector3d kda);
  Vector3d getfstar_rot(RobotData &Robot, int link_id);
  Vector6d getfstar6d(RobotData &Robot, int link_id, Vector3d kpt, Vector3d kdt, Vector3d kpa, Vector3d kda);
  Vector6d getfstar6d(RobotData &Robot, int link_id);
  Vector3d getfstar_acc_tra(RobotData &Robot, int link_id);
  Vector3d getfstar_acc_rot(RobotData &Robot, int link_id);

  VectorQd get_joint_acceleration(RobotData &Robot, VectorQd commnad_torque);
  Vector3d COM_traj_with_zmp(RobotData &Robot);
  void getSupportPolygon(RobotData &Robot, std::vector<Eigen::Vector2d> &edge_point_list);
  //zmp controller
  void getJkt(RobotData &Robot, MatrixXd &J_task, MatrixXd &Jkt);
  MatrixXd getJkt_f(RobotData &Robot, MatrixXd &J_task, MatrixXd &lambda);

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getjkt_m(MatrixVVd &Amat_inv, MatrixXd &Nc, MatrixXd &Winv, MatrixXd &Jtask);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getjkt_t(RobotData &Robot, MatrixXd &Jtask);
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> pinv_QR(MatrixXd &A);

  void CPpatternGen(RobotData &Robot);
  VectorQd CP_control_init(RobotData &Robot, double dT);
  VectorQd CP_controller();
  Vector6d zmp_controller(RobotData &Robot, Vector2d ZMP, double height);
  Vector2d CP_ref[20];

  //Vector2d getcptraj(double time, Vector2d zmp);

  Vector2d getcpref(RobotData &Robot, double task_time, double future_time);
  //Contact Mode
  const int DOUBLE_SUPPORT = 0;
  const int SINGLE_SUPPORT_LEFT = 1;
  const int SINGLE_SUPPORT_RIGHT = 2;
  const int TRIPPLE_SUPPORT = 3;
  const int QUAD_SUPPORT = 4;

  VectorQd torque_limit;

  std::string print_file_name;
  std::string print_file_name2;

  void CalcAMatrix(RobotData &Robot, MatrixXd &A_matrix);
  
  //QP solver setting
  void QPInitialize();
  void QPReset();
  int nIter;
  CQuadraticProgram QP_test;
  CQuadraticProgram QP_mpc;
  CQuadraticProgram QP_torque;

  std::vector<CQuadraticProgram> QP_hqp;

  VectorXd result_temp;

private:
  //update contact space dynamics
  //void contact_set(int contact_number, int link_id[]);
  void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta);
  void ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta);
};

class CapturePointPattern
{
public:
  void init(RobotData &Robot, int StepNumber, double foot_x_dis, double stepTime);
};

static std::ofstream data_out1;
static std::ofstream data_out2;
static bool print_data_wbc = false;

#endif // WALKING_CONTROLLER_H