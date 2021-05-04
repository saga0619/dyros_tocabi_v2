//urdf reader test

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl_math.h>
//#include <eigen3/Eigen/Dense>

#define MODEL_DOF 1
#define MODEL_DOF_VIRTUAL 7

typedef Eigen::Matrix<double, MODEL_DOF_VIRTUAL + 1, 1> VectorQVQd;
typedef Eigen::Matrix<double, MODEL_DOF_VIRTUAL, 1> VectorVQd;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;


struct mymodel_
{
    VectorQVQd q_;
    VectorVQd q_dot_;
    VectorVQd q_ddot_;
};

Eigen::MatrixXd A_temp_;

void updateKinematics_local(RigidBodyDynamics::Model &model_l, const Eigen::VectorXd &q_virtual_f, const Eigen::VectorXd &q_dot_virtual_f, const Eigen::VectorXd &q_ddot_virtual_f)
{
    std::cout<<model_l.q_size<<std::endl;
    std::cout<<model_l.qdot_size<<std::endl;
    A_temp_.setZero(MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
    //std::cout<<"c1"<<std::endl;
    RigidBodyDynamics::UpdateKinematicsCustom(model_l, &q_virtual_f, &q_dot_virtual_f, &q_ddot_virtual_f);
    //std::cout<<"c1"<<std::endl;

    

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_l, q_virtual_f, A_temp_, false);

    std::cout<<"c1"<<std::endl;

    A_temp_.setZero();
    std::cout<<"c1"<<std::endl;
}


namespace RigidBodyDynamics
{
    struct mytest_
    {
        std::vector<Eigen::MatrixXd> m1_;
        std::vector<Matrix3d> m2_;

        std::vector<Math::SpatialVector> v;
    };

};

int main(void)
{
    mymodel_ m1_;
    RigidBodyDynamics::Model model_;


    m1_.q_.setZero();
    m1_.q_dot_.setZero();
    m1_.q_ddot_.setZero();


    RigidBodyDynamics::Addons::URDFReadFromFile("/home/saga/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/test_robot.urdf", &model_, true, false);
    
    std::cout<<"u1"<<std::endl;

    updateKinematics_local(model_,m1_.q_,m1_.q_dot_,m1_.q_ddot_);
    std::cout<<"done"<<std::endl;


    return 1;
}
