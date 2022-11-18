#include <franka_control/Franka.hpp>

#include <control_lib/controllers/Feedback.hpp>
#include <control_lib/controllers/LinearDynamics.hpp>
#include <control_lib/spatial/SE3.hpp>

// using namespace control_lib;

// struct Params {
//     struct controller : public defaults::controller {
//     };

//     struct feedback : public defaults::feedback {
//     };

//     struct linear_dynamics : public defaults::linear_dynamics {
//     };
// };

// class FrankaController : public control::MultiBodyCtr {
// public:
//     OperationSpaceCtr() : control::MultiBodyCtr(ControlMode::OPERATIONSPACE)
//     {
//         // step
//         _dt = 0.01;

//         // set controlled frame
//         _frame = "lbr_iiwa_link_7";

//         // set controller gains
//         Eigen::MatrixXd D = 1 * Eigen::MatrixXd::Identity(6, 6);
//         _controller.setDamping(D);

//         // set ds gains
//         Eigen::MatrixXd A = 0.1 * Eigen::MatrixXd::Identity(6, 6);
//         _ds.setDynamicsMatrix(A);

//         // goal
//         Eigen::Vector3d xDes(0.365308, -0.0810892, 1.13717);
//         xDes += Eigen::Vector3d(0, -1, 0);
//         Eigen::Matrix3d oDes;
//         oDes << 0.591427, -0.62603, 0.508233,
//             0.689044, 0.719749, 0.0847368,
//             -0.418848, 0.300079, 0.857041;
//         _sDes._trans = xDes;
//         _sDes._rot = oDes;

//         // set reference
//         _ds.setReference(spatial::SE3(oDes, xDes));
//     }

//     Eigen::VectorXd action(bodies::MultiBody& body) override
//     {
//         // current state
//         spatial::SE3 sCurr(body.framePose(_frame));
//         sCurr._vel = body.frameVelocity(_frame);

//         // reference state
//         spatial::SE3 sRef;
//         sRef._vel = _ds.action(sCurr);

//         return _controller.setReference(sRef).action(sCurr);
//     }

// protected:
//     // step
//     double _dt;

//     // reference DS
//     spatial::SE3 _sDes;

//     // ds
//     controllers::LinearDynamics<Params, spatial::SE3> _ds;

//     // controller
//     controllers::Feedback<Params, spatial::SE3> _controller;
// };

int main(int argc, char const* argv[])
{
    return 0;
}
