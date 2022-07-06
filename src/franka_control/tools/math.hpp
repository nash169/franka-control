#ifndef FRANKACONTROL_TOOLS_MATH_HPP
#define FRANKACONTROL_TOOLS_MATH_HPP

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace franka_control {
    namespace tools {
        inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true)
        {
            double lambda_ = damped ? 0.2 : 0.0;

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
            Eigen::MatrixXd S_ = M_; // copying the dimensions of M_, its content is not needed.
            S_.setZero();

            for (int i = 0; i < sing_vals_.size(); i++)
                S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

            M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
        }
    } // namespace tools
} // namespace franka_control

#endif // FRANKACONTROL_TOOLS_MATH_HPP
