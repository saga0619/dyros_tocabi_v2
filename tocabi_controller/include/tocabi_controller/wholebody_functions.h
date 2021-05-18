#include "tocabi_data/robot_data.h"

using namespace TOCABI;

namespace WBC
{
    void SetContactInit(RobotData &rd_)
    {

        rd_.grav_ref << 0, 0, -9.81;

        rd_.ee_[0].contact_point << 0.03, 0, -0.1585;
        rd_.ee_[1].contact_point << 0.03, 0, -0.1585;
        rd_.ee_[2].contact_point << 0.0, 0.0, -0.035;
        rd_.ee_[3].contact_point << 0.0, 0.0, -0.035;

        rd_.ee_[0].sensor_point << 0.0, 0.0, -0.09;
        rd_.ee_[1].sensor_point << 0.0, 0.0, -0.09;

        rd_.ee_[0].InitializeEE(rd_.link_[TOCABI::Left_Foot], 0.15, 0.075, 40, 0.2, 0.2);
        rd_.ee_[1].InitializeEE(rd_.link_[TOCABI::Right_Foot], 0.15, 0.075, 40, 0.2, 0.2);

        rd_.ee_[2].InitializeEE(rd_.link_[TOCABI::Left_Hand], 0.02, 0.02, 40, 0.2, 0.2);
        rd_.ee_[3].InitializeEE(rd_.link_[TOCABI::Right_Hand], 0.02, 0.02, 40, 0.2, 0.2);
    }

    void CalcContact(RobotData &rd_)
    {
        rd_.I_C.setIdentity(rd_.contact_index * 6, rd_.contact_index * 6);

        rd_.Lambda_c = (rd_.J_C * rd_.A_inv_ * rd_.J_C.transpose()).inverse();

        rd_.J_C_INV_T = rd_.Lambda_c * rd_.J_C * rd_.A_inv_;

        rd_.N_C = MatrixVVd::Identity() - rd_.J_C.transpose() * rd_.J_C_INV_T;

        rd_.W = rd_.A_inv_.bottomRows(MODEL_DOF) * rd_.N_C.rightCols(MODEL_DOF);

        rd_.W_inv = DyrosMath::WinvCalc(rd_.W, rd_.qr_V2);

    }

    void SetContact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand = 0, bool right_hand = 0)
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
            Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.ee_[Robot.ee_idx[i]].jac_contact.cast<double>();
        }

        CalcContact(Robot);
    }

    // void CalcNonlinear(RobotData &rd_)
    // {
    //     RigidBodyDynamics::Math::VectorNd tau_;
    //     RigidBodyDynamics::NonlinearEffects(rd_.model_, rd_.q_virtual_, VectorVQf::Zero(), tau_);

    //     std::cout<<tau_.transpose();
    // }

    VectorQd gravity_compensation_torque(RobotData &rd_)
    {
        rd_.G.setZero();
        for (int i = 0; i < MODEL_DOF + 1; i++)
            rd_.G -= rd_.link_[i].jac_com.cast<double>().topRows(3).transpose() * rd_.link_[i].mass * rd_.grav_ref;


        rd_.torque_grav = rd_.W_inv * (rd_.A_inv_.bottomRows(MODEL_DOF) * (rd_.N_C * rd_.G));
        rd_.P_C = rd_.J_C_INV_T * rd_.G;
       
        return rd_.torque_grav;
    }

    void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
    {
        Eigen::Matrix6x12d W;
        W.setZero();

        W.block(0, 0, 6, 6).setIdentity();
        W.block(0, 6, 6, 6).setIdentity();
        W.block(3, 0, 3, 3) = DyrosMath::skm(P1);
        W.block(3, 6, 3, 3) = DyrosMath::skm(P2);

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
                eta_ub = sol_eta1;

            if (sol_eta2 > eta_lb)
                eta_lb = sol_eta2;
        }
        else //sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
                eta_ub = sol_eta2;

            if (sol_eta1 > eta_lb)
                eta_lb = sol_eta1;
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
                eta_ub = sol_eta1;
            if (sol_eta2 > eta_lb)
                eta_lb = sol_eta2;
        }
        else //sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
                eta_ub = sol_eta2;
            if (sol_eta1 > eta_lb)
                eta_lb = sol_eta1;
        }
        //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

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
        //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
        //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
        //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
    }

    VectorXd contact_force_redistribution_torque(RobotData &Robot, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta)
    {
        int contact_dof_ = Robot.J_C.rows();

        if (contact_dof_ == 12)
        {
            Vector12d ContactForce_ = Robot.J_C_INV_T.rightCols(MODEL_DOF) * command_torque - Robot.P_C;

            Vector3d P1_ = Robot.ee_[0].xpos_contact - Robot.link_[COM_id].xpos;
            Vector3d P2_ = Robot.ee_[1].xpos_contact - Robot.link_[COM_id].xpos;

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
            ForceRedistribution = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();
            for (int i = 0; i < 6; i++)
            {
                desired_force(i + 6) = -ContactForce_(i + 6) + ForceRedistribution(i + 6);
            }
            Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).bottomRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(6,6);
        }
        else
        {
            Robot.torque_contact.setZero();
        }

        return Robot.torque_contact;
    }

}