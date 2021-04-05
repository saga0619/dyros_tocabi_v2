#include "tocabi_controller/wholebody_controller.h"
#include <Eigen/QR>
#include "ros/ros.h"
#include <vector>
#include <future>
#include <numeric>

namespace wbc_
{
    void SetContact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand, bool right_hand)
    {
        Robot.ee_[0].contact = left_foot;
        Robot.ee_[1].contact = right_foot;
        Robot.ee_[2].contact = left_hand;
        Robot.ee_[3].contact = right_hand;

        Robot.contact_index = 0;
        if (left_foot)
        {
            Robot.contact_part[Robot.contact_index] = TOCABI::Left_Foot;
            Robot.ee_idx[Robot.contact_index] = 0;
            Robot.contact_index++;
        }
        if (right_foot)
        {
            Robot.contact_part[Robot.contact_index] = TOCABI::Right_Foot;
            Robot.ee_idx[Robot.contact_index] = 1;
            Robot.contact_index++;
        }
        if (left_hand)
        {
            Robot.contact_part[Robot.contact_index] = TOCABI::Left_Hand;
            Robot.ee_idx[Robot.contact_index] = 2;
            Robot.contact_index++;
        }
        if (right_hand)
        {
            Robot.contact_part[Robot.contact_index] = TOCABI::Right_Hand;
            Robot.ee_idx[Robot.contact_index] = 3;
            Robot.contact_index++;
        }

        Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);

        Robot.ee_[0].SetContact(Robot.model_, Robot.q_virtual_);
        Robot.ee_[1].SetContact(Robot.model_, Robot.q_virtual_);
        Robot.ee_[2].SetContact(Robot.model_, Robot.q_virtual_);
        Robot.ee_[3].SetContact(Robot.model_, Robot.q_virtual_);

        for (int i = 0; i < Robot.contact_index; i++)
        {
            Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.ee_[Robot.ee_idx[i]].jac_contact;
        }
    }

    void SetContactInit(RobotData &rd_)
    {
        rd_.ee_[0].contact_point << 0.03, 0, -0.1585;
        rd_.ee_[1].contact_point << 0.03, 0, -0.1585;
        rd_.ee_[2].contact_point << 0.0, 0.0, -0.035;
        rd_.ee_[3].contact_point << 0.0, 0.0, -0.035;

        rd_.ee_[0].sensor_point << 0.0, 0.0, -0.09;
        rd_.ee_[1].sensor_point << 0.0, 0.0, -0.09;

        rd_.ee_[0].Initialize(rd_.link_[TOCABI::Left_Foot], 0.15, 0.075, 40, 0.2, 0.2);
        rd_.ee_[1].Initialize(rd_.link_[TOCABI::Left_Foot], 0.15, 0.075, 40, 0.2, 0.2);

        rd_.ee_[2].Initialize(rd_.link_[TOCABI::Left_Foot], 0.02, 0.02, 40, 0.2, 0.2);
        rd_.ee_[3].Initialize(rd_.link_[TOCABI::Left_Foot], 0.02, 0.02, 40, 0.2, 0.2);
    }

    void CalcContact(RobotData &rd_)
    {
        rd_.Lambda_c = rd_.J_C * rd_.A_inv_ * rd_.J_C.transpose();
        rd_.J_C_INV_T = rd_.Lambda_c * rd_.J_C * rd_.A_inv_;

        rd_.N_C = MatrixVVf::Identity() - rd_.J_C.transpose() * rd_.J_C_INV_T;
        rd_.W = rd_.A_.bottomRows(MODEL_DOF) * rd_.N_C.rightCols(MODEL_DOF);

        rd_.W_inv = DyrosMath::pinv_QR(rd_.W, rd_.qr_V2);
    }

    void CalcNonlinear(RobotData &rd_)
    {
        RigidBodyDynamics::Math::VectorNd tau_;
        RigidBodyDynamics::NonlinearEffects(rd_.model_, rd_.q_virtual_, VectorVQf::Zero(), tau_);

        std::cout<<tau_.transpose();
    }

    VectorQf gravity_compensation_torque(RobotData &rd_)
    {
        //RigidBodyDynamics::Comp
        rd_.G.setZero();
        for (int i = 0; i < MODEL_DOF; i++)
            rd_.G -= rd_.link_[i].jac_com.topRows(3).transpose() * rd_.link_[i].mass * rd_.grav_ref;

        rd_.torque_grav = rd_.Lambda_c * (rd_.A_inv_.bottomRows(MODEL_DOF) * (rd_.N_C * rd_.G));
        rd_.P_C = rd_.Lambda_c * (rd_.J_C * (rd_.A_inv_ * rd_.G));

        return rd_.torque_grav;
    }

}