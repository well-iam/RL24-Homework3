#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    /*
    std::cout << "INSIDE CONTROLLER\n";
    std::cout << ddqd << "\n";
    std::cout << robot_->getJsim() << "\n\n";
    */
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis();// + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    //implementation of Op.Space Controller 4a)
 
// calculate gain matrices (matrici diagonali)
    Eigen::Matrix<double,6,6> Kp, Kd;
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();  
    //Kp.block(2,2,1,1) = 5*Kp.block(2,2,1,1);
    //Kd.block(2,2,1,1) = 2*Kd.block(2,2,1,1);
 
    // read current cartesian state 
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
    Eigen::Matrix<double,7,7> M = robot_->getJsim();
    Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);
 
 
    // position (desired + current)
    Eigen::Vector3d p_d(_desPos.p.data);
    Eigen::Vector3d p_e(robot_->getEEFrame().p.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
    R_d = matrixOrthonormalization(R_d);
    R_e = matrixOrthonormalization(R_e);
 
    // velocity (desired + current)
    Eigen::Vector3d dot_p_d(_desVel.vel.data);                                 
    Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);
    Eigen::Vector3d omega_d(_desVel.rot.data);
    Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);
 
    // acceleration (desired)
    Eigen::Matrix<double,6,1> dot_dot_x_d;
    Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
    Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);
 
    // compute linear errors (position + velocity)
    Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);
    std::cout << "Linear error: \n" << e_p << std::endl;

    // compute orientation errors (position + velocity)                                        
    Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
    Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
                                                                        omega_e,
                                                                        R_d,
                                                                        R_e);
    std::cout << "Orientation error: \n" << e_o << std::endl;

    // calculate errors                                                                   
    Eigen::Matrix<double,6,1> x_tilde;                                                 
    Eigen::Matrix<double,6,1> dot_x_tilde;
    x_tilde << e_p, e_o;
    dot_x_tilde << dot_e_p, dot_e_o;
    dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;
 
    //Define J_dot*q_dot
    Eigen::Matrix<double,6,1> Jdotqdot = robot_->getEEJacDotqDot();
 
    //Control Law
    Eigen::Matrix<double,7,1> y;
    Eigen::Matrix<double,7,1> n;
 
    y << Jpinv * (dot_dot_x_d - Jdotqdot + Kd*dot_x_tilde + Kp*x_tilde);
    n << robot_->getCoriolis();// + robot_->getGravity();
 
    
    return M * y + n;
}

// Function to compute the control law for q_dot
Eigen::VectorXd KDLController::look_at_point_control(
    const Eigen::Vector3d &cPo,                     // Object position in camera frame
    const Eigen::VectorXd &qdot_null,               // Null-space velocity (optional)
    double k                                        // Gain
) {
    k=3;
    // Get EE(=camera) Jacobian
    Eigen::Matrix<double,6,7> Jc = robot_->getEEJacobian().data;

    // Get Camera rotation matrix
    Eigen::Matrix3d Rc = toEigen(robot_->getEEFrame().M);
    
    // Ensure the object is not at the camera origin to avoid division by zero
    double norm_cPo = cPo.norm();
    //std::cout << "NORMA: " << norm_cPo << "\n";
    if (norm_cPo < 1e-6) {
        throw std::runtime_error("cPo is too close to the camera origin!");
    }

    // Compute s = cPo / ||cPo||
    Eigen::Vector3d s = cPo / norm_cPo;

    // Desired s_d = [0, 0, 1]
    Eigen::Vector3d s_d(0, 0, 1);

    // Compute L(s)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S_s = skew(s);
    Eigen::Matrix3d L_linear = -1.0 / norm_cPo * (I - s * s.transpose()) * Rc;

    Eigen::MatrixXd L(3, 6);
    L.block<3, 3>(0, 0) = L_linear;
    L.block<3, 3>(0, 3) = S_s * Rc;

    // Compute the pseudoinverse of L * Jc
    Eigen::MatrixXd LJ = L * Jc;
    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse(); //Eventualmente usare LJ_pinv = pseudoinverse(LJ)
    LJ_pinv = pseudoinverse(LJ);

    // Compute null space projector
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(Jc.cols(), Jc.cols()) - LJ_pinv * LJ;

    // Compute q_dot
    Eigen::VectorXd q_dot = k * LJ_pinv * s_d+ N * qdot_null;
    //std::cout << "Velocity norm command: " << q_dot.norm() << "\n";
    return q_dot;
}
