#pragma once

#include <memory>
#include <string>
#include <Eigen/Dense>

struct JacobianResult
{
    Eigen::Matrix2d J_motor2Joint;
    Eigen::Matrix2d J_Joint2motor;
};

struct ForwardMappingResult
{
    int count;
    Eigen::Vector2d ankle_joint_ori;
    JacobianResult Jac;
};

class Decouple
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~Decouple() = default;

    virtual void get_forwardQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel,
                                Eigen::VectorXd &tau, bool is_left) = 0;

    virtual void get_decoupleQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel,
                                 Eigen::VectorXd &tau, bool is_left) = 0;

    static std::shared_ptr<Decouple> create(const std::string &type);
};
