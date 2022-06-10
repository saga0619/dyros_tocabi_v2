#ifndef DWBC_H
#define DWBC_H

#include <Eigen/Dense>
using namespace Eigen;

namespace DWBC
{

    MatrixXd GetZMPConstMatrix(double local_contact_plane_x, double local_contact_plane_y)
    {
        MatrixXd zmp_const_mat = MatrixXd::Zero(4, 6);

        zmp_const_mat(0, 2) = -local_contact_plane_x;
        zmp_const_mat(0, 4) = -1;

        zmp_const_mat(1, 2) = -local_contact_plane_x;
        zmp_const_mat(1, 4) = 1;

        zmp_const_mat(2, 2) = -local_contact_plane_y;
        zmp_const_mat(2, 3) = -1;

        zmp_const_mat(3, 2) = -local_contact_plane_y;
        zmp_const_mat(3, 3) = 1;

        return zmp_const_mat;
    }
    MatrixXd GetForceConstMatrix(double friction_ratio, double friction_ratio_z)
    {
        MatrixXd force_const_matrix = MatrixXd::Zero(6, 6);

        force_const_matrix(0, 0) = 1.0;
        force_const_matrix(0, 2) = -friction_ratio;
        force_const_matrix(1, 0) = -1.0;
        force_const_matrix(1, 2) = -friction_ratio;

        force_const_matrix(2, 1) = 1.0;
        force_const_matrix(2, 2) = -friction_ratio;
        force_const_matrix(3, 1) = -1.0;
        force_const_matrix(3, 2) = -friction_ratio;

        force_const_matrix(4, 5) = 1.0;
        force_const_matrix(4, 2) = -friction_ratio_z;
        force_const_matrix(5, 5) = -1.0;
        force_const_matrix(5, 2) = -friction_ratio_z;

        return force_const_matrix;
    }
    void CalculateContactConstraint(
        const MatrixXd &J_contact, const MatrixXd &A_inv,
        MatrixXd &lambda_contact, MatrixXd &J_C_INV_T, MatrixXd &N_C, MatrixXd &W, MatrixXd &NwJw, MatrixXd &Winv, MatrixXd &V2)
    {
        int rows = J_contact.rows();
        int cols = J_contact.cols();

        lambda_contact = (J_contact * A_inv * J_contact.transpose()).inverse();
        J_C_INV_T = lambda_contact * J_contact * A_inv;
        N_C = MatrixXd::Identity(cols, cols) - J_contact.transpose() * J_C_INV_T;
        W = A_inv.bottomRows(cols - 6) * N_C.rightCols(cols - 6);
        PinvCODWB(W, Winv, V2);
        NwJw = V2.transpose() * (J_C_INV_T.rightCols(cols - 6).topRows(6) * V2.transpose()).inverse();
    }

    /*
     * Gravity Compensation Torque Calculation
     * input  : A_inv, W_inv, N_C, J_C_INV_T, G
     * output : torque_grav,
     */
    void CalculateGravityCompensation(
        const MatrixXd &A_inv, const MatrixXd &W_inv, const MatrixXd &N_C, const MatrixXd &J_C_INV_T, const VectorXd &G,
        VectorXd &torque_grav, MatrixXd &P_C)
    {
        torque_grav = W_inv * (A_inv.bottomRows(W_inv.cols()) * (N_C * G));
        P_C = J_C_INV_T * G;
    }

    /*
     * Get Jkt
     * input  : J_task, A_inv, N_C, W_inv,
     * output : J_kt, lambda_task
     */
    void CalculateJKT(const MatrixXd &J_task, const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv,
                      MatrixXd &J_kt, MatrixXd &lambda_task)
    {
        lambda_task = (J_task * A_inv * N_C * J_task.transpose()).inverse();
        MatrixXd Q = (lambda_task * J_task * A_inv * N_C).rightCols(J_task.cols() - 6);
        J_kt = W_inv * Q.transpose() * PinvCODWB(Q * W_inv * Q.transpose());
    }


    /*
     * Calculate Taskspace Null matrix
     * input  : J_kt, lambda_task, J_task, A_inv, N_C, upper_task_null
     * output : Null_task
     */
    void CalculateTaskNullSpace(const MatrixXd &J_kt, const MatrixXd &Lambda_task, const MatrixXd &J_task, const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &prev_null, MatrixXd &Null_task)
    {
        int model_size = J_task.cols() - 6;
        Null_task = prev_null * (MatrixXd::Identity(model_size, model_size) - J_kt * Lambda_task * J_task * A_inv * N_C.rightCols(model_size));
    }

    /*
     * Calculate contact force from command torque
     * input  : command_torque, J_C_INV_T, P_C
     * output : ContactForce
     */
    void CalculateContactForce(const VectorXd &command_torque, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, VectorXd &ContactForce)
    {
        ContactForce = J_C_INV_T.rightCols(command_torque.size()) * command_torque - P_C;
    }

    
};

#endif