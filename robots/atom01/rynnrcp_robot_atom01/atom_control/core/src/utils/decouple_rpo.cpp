#include "utils/decouple_rpo.hpp"
#include <iostream>

static const Eigen::Vector3d S_BAR(0, 1, 0);

//////********************link parameters********************//////
std::vector<LinkParamsRPO> DecoupleRPO::get_links(bool is_left)
{
    std::vector<LinkParamsRPO> links(2);

    double l_bar = 20.0;
    double l_spacing = is_left ? 42.35 : -42.35;

    double long_link_angle_0 = 0.0;
    double short_link_angle_0 = 180.0 * M_PI / 180.0;

    double r_B1_0_x = -l_bar * std::cos(long_link_angle_0);
    double r_B1_0_z = 180.0 - l_bar * std::sin(long_link_angle_0);
    double r_B2_0_x = -l_bar * std::cos(short_link_angle_0);
    double r_B2_0_z = 110.0 - l_bar * std::sin(short_link_angle_0);

    // long link
    links[0].l_rod = 180.0;
    links[0].l_bar = l_bar;
    links[0].l_spacing = l_spacing;
    links[0].r_A_0 = Eigen::Vector3d(0, l_spacing, 180);
    links[0].r_B_0 = Eigen::Vector3d(r_B1_0_x, l_spacing, r_B1_0_z);
    links[0].r_C_0 = Eigen::Vector3d(-20, l_spacing, 0);
    links[0].theta_0 = long_link_angle_0;

    // short link
    links[1].l_rod = 110.0;
    links[1].l_bar = l_bar;
    links[1].l_spacing = l_spacing;
    links[1].r_A_0 = Eigen::Vector3d(0, l_spacing, 110);
    links[1].r_B_0 = Eigen::Vector3d(r_B2_0_x, l_spacing, r_B2_0_z);
    links[1].r_C_0 = Eigen::Vector3d(20, l_spacing, 0);
    links[1].theta_0 = short_link_angle_0;

    return links;
}

DecoupleRPO::DecoupleRPO()
    : links_left_(get_links(true)), links_right_(get_links(false))
{
}
//////********************link parameters********************//////

//////********************inverse kinematics*****************//////
IKResultRPO
DecoupleRPO::inverse_kinematics(double q_roll, double q_pitch, bool is_left)
{
    IKResultRPO result;
    result.THETA = Eigen::Vector2d::Zero();

    // Rotation matrices
    Eigen::Matrix3d R_y;
    R_y << std::cos(q_pitch), 0, std::sin(q_pitch),
        0, 1, 0,
        -std::sin(q_pitch), 0, std::cos(q_pitch);

    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
        0, std::cos(q_roll), -std::sin(q_roll),
        0, std::sin(q_roll), std::cos(q_roll);

    Eigen::Matrix3d x_rot = R_y * R_x;

    const auto &links = cached_links(is_left);

    for (int i = 0; i < 2; i++)
    {
        double l_rod = links[i].l_rod;
        double l_bar = links[i].l_bar;
        Eigen::Vector3d r_A_i = links[i].r_A_0;
        Eigen::Vector3d r_C_i = x_rot * links[i].r_C_0;
        Eigen::Vector3d rBA_bar = links[i].r_B_0 - links[i].r_A_0;

        double a = r_C_i[0] - r_A_i[0];
        double b = r_A_i[2] - r_C_i[2];
        double c = (l_rod * l_rod - l_bar * l_bar - (r_C_i - r_A_i).squaredNorm()) / (2 * l_bar);

        double a_sq = a * a;
        double b_sq = b * b;
        double c_sq = c * c;
        double ab_sq_sum = a_sq + b_sq;
        double discriminant = b_sq * c_sq - ab_sq_sum * (c_sq - a_sq);
        if (discriminant < 0)
        {
            std::cerr << "Warning: Negative discriminant in inverse kinematics. Setting theta_i to 0." << std::endl;
            discriminant = 0;
        }

        double theta_i = std::asin((b * c + std::sqrt(discriminant)) / ab_sq_sum);
        theta_i = a < 0 ? theta_i : -theta_i;

        Eigen::Matrix3d R_y_theta;
        R_y_theta << std::cos(theta_i), 0, std::sin(theta_i),
            0, 1, 0,
            -std::sin(theta_i), 0, std::cos(theta_i);

        Eigen::Vector3d r_B_i = r_A_i + R_y_theta * rBA_bar;

        result.r_bar[i] = r_B_i - r_A_i;
        result.r_rod[i] = r_C_i - r_B_i;
        result.r_C[i] = r_C_i;
        result.THETA[i] = theta_i;
    }

    return result;
}
//////********************inverse kinematics*****************//////

//////********************jacobian***************************//////
JacobianResult
DecoupleRPO::jacobian(const IKResultRPO &ik_result, double q_pitch)
{
    const auto &r_C = ik_result.r_C;
    const auto &r_bar = ik_result.r_bar;
    const auto &r_rod = ik_result.r_rod;

    Eigen::Matrix<double, 2, 6> J_x;
    J_x << r_rod[0].transpose(), (r_C[0].cross(r_rod[0])).transpose(),
        r_rod[1].transpose(), (r_C[1].cross(r_rod[1])).transpose();

    Eigen::Matrix2d J_theta;
    J_theta << S_BAR.dot(r_bar[0].cross(r_rod[0])), 0,
        0, S_BAR.dot(r_bar[1].cross(r_rod[1]));

    Eigen::Matrix<double, 6, 2> J_q;
    J_q << 0, 0,
        0, 0,
        0, 0,
        0, std::cos(q_pitch),
        1, 0,
        0, -std::sin(q_pitch);

    Eigen::Matrix2d J_Temp = J_x * J_q;

    Eigen::PartialPivLU<Eigen::Matrix2d> lu_decomp(J_Temp);
    Eigen::PartialPivLU<Eigen::Matrix2d> lu_theta(J_theta);

    JacobianResult result;
    result.J_motor2Joint = lu_decomp.solve(J_theta);
    result.J_Joint2motor = lu_theta.solve(J_Temp);

    return result;
}
//////********************jacobian***************************//////

// from joint [pitch, roll] to motor [theta], from S to P
std::pair<Eigen::Vector2d, JacobianResult>
DecoupleRPO::get_decouple(double roll, double pitch, bool is_left)
{
    IKResultRPO kinematics = inverse_kinematics(roll, pitch, is_left);
    JacobianResult Jac = jacobian(kinematics, pitch);
    return {kinematics.THETA, Jac};
}

//////********************forward kinematics*****************//////
ForwardMappingResult
DecoupleRPO::forward_kinematics(const Eigen::Vector2d &thetaRef, bool is_left)
{
    ForwardMappingResult mapping_result;

    int count = 0;
    Eigen::Vector2d f_error{10, 10};
    Eigen::Vector2d x_k = last_solution_.count(is_left) ? last_solution_[is_left] : Eigen::Vector2d::Zero();

    JacobianResult Jac;
    static constexpr int MAX_ITERATIONS = 100;
    static constexpr double TOLERANCE = 1e-3;
    static constexpr double ALPHA = 0.5;

    while (f_error.norm() > TOLERANCE && count < MAX_ITERATIONS)
    {
        IKResultRPO kinematics = inverse_kinematics(x_k[1], x_k[0], is_left);
        Jac = jacobian(kinematics, x_k[0]);

        if (Jac.J_motor2Joint.hasNaN())
        {
            std::cerr << "DecoupleRPO::forward_kinematics() Jac is nan!!" << std::endl;
            std::cerr << "  pitch=" << x_k[0] << ", roll=" << x_k[1] << std::endl;
            mapping_result.count = -1;
            mapping_result.ankle_joint_ori = Eigen::Vector2d::Zero();
            mapping_result.Jac = Jac;
            return mapping_result;
        }

        f_error = thetaRef - kinematics.THETA;
        x_k = x_k + ALPHA * Jac.J_motor2Joint * f_error;
        count++;
    }

    if (f_error.norm() < TOLERANCE)
    {
        last_solution_[is_left] = x_k;
    }

    mapping_result.count = count;
    mapping_result.ankle_joint_ori = x_k;
    mapping_result.Jac = Jac;

    return mapping_result;
}
//////********************forward kinematics*****************//////

// from joint [pitch, roll] to motor [theta], from Serial to Parallel
void DecoupleRPO::get_decoupleQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel, Eigen::VectorXd &tau, bool is_left)
{
    double pitch = q[0];
    double roll = q[1];

    auto motor = get_decouple(roll, pitch, is_left);
    q = motor.first;
    vel = motor.second.J_Joint2motor * vel;
    tau = motor.second.J_motor2Joint.transpose() * tau;
}

void DecoupleRPO::get_forwardQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel, Eigen::VectorXd &tau, bool is_left)
{
    ForwardMappingResult joint = forward_kinematics(q, is_left);
    q = joint.ankle_joint_ori;
    vel = joint.Jac.J_motor2Joint * vel;
    tau = joint.Jac.J_Joint2motor.transpose() * tau;
}
