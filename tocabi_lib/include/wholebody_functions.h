#ifndef WBF_H
#define WBF_H

#include "tocabi_lib/robot_data.h"
#include "qp.h"
#include <fstream>

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

    bool GravMinMax(VectorQd torque)
    {
        static bool loading = false;
        char gf_[] = "/home/dyros/.tocabi_bootlog/minmax_log";
        char gf_v[] = "/home/dyros/.tocabi_bootlog/minmax_view";
        static double tminmax[66];
        // torque,

        if (!loading)
        {
            std::ifstream ifs(gf_, std::ios::binary);

            if (!ifs.is_open())
            {
                std::cout << "GMM read failed " << std::endl;
            }
            else
            {
                for (int i = 0; i < 66; i++)
                {
                    ifs.read(reinterpret_cast<char *>(&tminmax[i]), sizeof(double));
                }

                ifs.close();
            }
            loading = true;
        }

        bool record = false;
        for (int i = 0; i < 33; i++)
        {
            if (tminmax[i] > torque[i])
            {
                record = true;

                tminmax[i] = torque[i];
            }

            if (tminmax[i + 33] < torque[i])
            {
                record = true;

                tminmax[i + 33] = torque[i];
            }
        }

        if (record)
        {
            std::ofstream ofs(gf_, std::ios::binary);
            std::ofstream ofs_view(gf_v);
            if (!ofs.is_open())
            {
                std::cout << "GMM write failed " << std::endl;
            }
            else
            {
                for (int i = 0; i < 66; i++)
                    ofs.write(reinterpret_cast<char const *>(&tminmax[i]), sizeof(double));
                ofs.close();
            }

            if (!ofs_view.is_open())
            {
                std::cout << "GMM view write failed " << std::endl;
            }
            else
            {
                ofs_view << "MIN VALUE : " << std::endl;
                for (int i = 0; i < 33; i++)
                {
                    ofs_view << tminmax[i] << std::endl;
                }
                ofs_view << "MAX VALUE : " << std::endl;
                for (int i = 0; i < 33; i++)
                {
                    ofs_view << tminmax[i + 33] << std::endl;
                }
            }
        }

        return true;
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

    Vector3d GetFstarPos(LinkData &link_)
    {
        Vector3d fstar;

        for (int i = 0; i < 3; i++)
            fstar(i) = link_.pos_p_gain(i) * (link_.x_traj(i) - link_.xpos(i)) + link_.pos_d_gain(i) * (link_.v_traj(i) - link_.v(i));

        return fstar;
    }

    Vector3d GetFstarRot(LinkData &link_)
    {
        Vector3d fstar;

        Vector3d ad;

        ad = DyrosMath::getPhi(link_.rotm, link_.r_traj);

        for (int i = 0; i < 3; i++)
            fstar(i) = -link_.rot_p_gain(i) * ad(i) + link_.rot_d_gain(i) * (link_.w_traj(i) - link_.w(i));

        //std::cout << ad.transpose() << "\t" << (link_.w_traj - link_.w).transpose() << std::endl;
        //std::cout << DyrosMath::rot2Euler_tf(link_.rotm).transpose() << "\t" << DyrosMath::rot2Euler_tf(link_.r_traj) << std::endl;
        //return link_.rot_p_gain.cwiseProduct(DyrosMath::getPhi(link_.rotm, link_.r_traj)); // + link_.rot_d_gain.cwiseProduct(link_.w_traj - link_.w);

        return fstar;
    }

    Vector6d GetFstar6d(LinkData &link_)
    {
        Vector6d f_star;

        f_star.segment(0, 3) = GetFstarPos(link_);
        f_star.segment(3, 3) = GetFstarRot(link_);

        return f_star;
    }

    // void CalcNonlinear(RobotData &rd_)
    // {
    //     RigidBodyDynamics::Math::VectorNd tau_;
    //     RigidBodyDynamics::NonlinearEffects(rd_.model_, rd_.q_virtual_, VectorVQf::Zero(), tau_);

    //     std::cout<<tau_.transpose();
    // }

    VectorQd GravityCompensationTorque(RobotData &rd_)
    {
        rd_.G.setZero();
        for (int i = 0; i < MODEL_DOF + 1; i++)
            rd_.G -= rd_.link_[i].jac_com.cast<double>().topRows(3).transpose() * rd_.link_[i].mass * rd_.grav_ref;

        rd_.torque_grav = rd_.W_inv * (rd_.A_inv_.bottomRows(MODEL_DOF) * (rd_.N_C * rd_.G));
        rd_.P_C = rd_.J_C_INV_T * rd_.G;

        return rd_.torque_grav;
    }

    VectorQd GravityCompenstationTorque_Isolated(RobotData &rd_, bool contact_left_foot_, bool contact_right_foot_)
    {
    }

    // template <int TaskNum>
    // Eigen::Matrix<double, MODEL_DOF, TaskNum> GetJKT(Matrix<double, TaskNum, MODEL_DOF_VIRTUAL> J_task_)
    // {
    //     Matrix<double, MODEL_DOF, TaskNum> res;
    //     Matrix<double, TaskNum, TaskNum> lambda_ = (J_task_ * rd_.A_inv_ * rd_.N_C * J_task_.transpose()).inverse();
    //     Matrix<double, TaskNum, MODEL_DOF> Q_ = lambda_ * J_task_ * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);

    //     res = rd_.W_inv * DyrosMath::pinv_QR(Q_ * rd_.W_inv * Q_.transpose());

    //     return res;
    // }
    MatrixXd GetJKT1(RobotData &rd_, MatrixXd &J_task)
    {
        MatrixXd lambda_ = (J_task * rd_.A_inv_ * rd_.N_C * J_task.transpose()).inverse();
        MatrixXd Q_ = lambda_ * J_task * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);

        return rd_.W_inv * Q_.transpose() * DyrosMath::pinv_QR_prev(Q_ * rd_.W_inv * Q_.transpose());
    }

    MatrixXd GetJKT2(RobotData &rd_, MatrixXd &J_task)
    {
        int t_d = J_task.rows();
        MatrixXd lambda_ = (J_task * rd_.A_inv_ * rd_.N_C * J_task.transpose()).llt().solve(MatrixXd::Identity(t_d, t_d));
        MatrixXd Q_ = lambda_ * J_task * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);

        return rd_.W_inv * Q_.transpose() * DyrosMath::pinv_QR(Q_ * rd_.W_inv * Q_.transpose());
    }

    VectorQd TaskControlTorque(RobotData &rd_, VectorXd f_star)
    {
        int task_dof = rd_.J_task.rows();
        //rd_.J_task = J_task;
        rd_.J_task_T = rd_.J_task.transpose();

        rd_.lambda_inv = rd_.J_task * rd_.A_inv_ * rd_.N_C * rd_.J_task_T;

        rd_.lambda = rd_.lambda_inv.llt().solve(MatrixXd::Identity(task_dof, task_dof));

        rd_.J_task_inv_T = rd_.lambda * rd_.J_task * rd_.A_inv_ * rd_.N_C;

        rd_.Q = rd_.J_task_inv_T.rightCols(MODEL_DOF);

        rd_.Q_T_ = rd_.Q.transpose();

        //std::cout<<"1"<<std::endl;
        rd_.Q_temp = rd_.Q * rd_.W_inv * rd_.Q_T_;
        //std::cout<<"2"<<std::endl;

        rd_.Q_temp_inv = DyrosMath::pinv_QRs(rd_.Q_temp);

        //DyrosMath::dc_inv_QR(rd_.J_task)

        //std::cout<<"3"<<std::endl;

        return rd_.W_inv * (rd_.Q_T_ * (rd_.Q_temp_inv * (rd_.lambda * f_star)));
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
    VectorQd ContactForceRedistributionTorqueWalking(RobotData &Robot, VectorQd command_torque, double eta = 0.9, double ratio = 1.0, int supportFoot = 0)
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
            Robot.fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();

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

                desired_force.segment(0, 6) = -ContactForce_.segment(0, 6) + ratio * Robot.fc_redist_.segment(0, 6);
                Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(0, 6);
            }
            else
            {
                desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + ratio * Robot.fc_redist_.segment(6, 6);
                Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).bottomRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(6, 6);
            }
        }
        else
        {
            Robot.torque_contact.setZero();
        }

        return Robot.torque_contact + command_torque;
    }

    VectorQd ContactForceRedistributionTorque(RobotData &Robot, VectorQd command_torque, double eta = 0.9)
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
            Robot.fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();

            desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + Robot.fc_redist_.segment(6, 6);
            Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).bottomRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(6, 6);
        }
        else
        {
            Robot.torque_contact.setZero();
        }

        return Robot.torque_contact + command_torque;
    }

}

#endif