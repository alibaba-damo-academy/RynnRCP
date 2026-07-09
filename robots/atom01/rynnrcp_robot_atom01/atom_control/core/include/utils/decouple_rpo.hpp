#pragma once

#include "utils/close_chain_mapping.hpp"
#include <array>
#include <map>
#include <vector>

struct LinkParamsRPO
{
    double l_rod;
    double l_bar;
    double l_spacing;
    Eigen::Vector3d r_A_0;
    Eigen::Vector3d r_B_0;
    Eigen::Vector3d r_C_0;
    double theta_0;
};

struct IKResultRPO
{
    std::array<Eigen::Vector3d, 2> r_bar;
    std::array<Eigen::Vector3d, 2> r_rod;
    std::array<Eigen::Vector3d, 2> r_C;
    Eigen::Vector2d THETA;
};

class DecoupleRPO : public Decouple
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DecoupleRPO();

    void get_forwardQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel,
                        Eigen::VectorXd &tau, bool is_left) override;

    void get_decoupleQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel,
                         Eigen::VectorXd &tau, bool is_left) override;

private:
    std::vector<LinkParamsRPO> links_left_;
    std::vector<LinkParamsRPO> links_right_;
    std::map<bool, Eigen::Vector2d> last_solution_;

    static std::vector<LinkParamsRPO> get_links(bool is_left);

    const std::vector<LinkParamsRPO> &cached_links(bool is_left) const
    {
        return is_left ? links_left_ : links_right_;
    }

    IKResultRPO inverse_kinematics(double q_roll, double q_pitch, bool is_left);
    JacobianResult jacobian(const IKResultRPO &ik_result, double q_pitch);
    std::pair<Eigen::Vector2d, JacobianResult> get_decouple(double roll, double pitch, bool is_left);
    ForwardMappingResult forward_kinematics(const Eigen::Vector2d &thetaRef, bool is_left);
};
