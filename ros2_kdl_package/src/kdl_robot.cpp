#include "kdl_robot.h"

KDLRobot::KDLRobot(){}

KDLRobot::KDLRobot(KDL::Tree &robot_tree)
{
    createChain(robot_tree);
    n_ = chain_.getNrOfJoints();
    grav_ = KDL::JntArray(n_);
    s_J_ee_ = KDL::Jacobian(n_);
    b_J_ee_ = KDL::Jacobian(n_);
    s_J_dot_ee_ = KDL::Jacobian(n_);
    b_J_dot_ee_ = KDL::Jacobian(n_);
    s_J_ee_.data.setZero();
    b_J_ee_.data.setZero();
    s_J_dot_ee_.data.setZero();
    b_J_dot_ee_.data.setZero();
    jntArray_ = KDL::JntArray(n_);
    jntVel_ = KDL::JntArray(n_);
    coriol_ = KDL::JntArray(n_);
    dynParam_ = new KDL::ChainDynParam(chain_,KDL::Vector(0,0,-9.81));
    jacSol_ = new KDL::ChainJntToJacSolver(chain_);
    jntJacDotSol_ = new KDL::ChainJntToJacDotSolver(chain_);
    fkSol_ = new KDL::ChainFkSolverPos_recursive(chain_);
    fkVelSol_ = new KDL::ChainFkSolverVel_recursive(chain_);
    idSolver_ = new KDL::ChainIdSolver_RNE(chain_,KDL::Vector(0,0,-9.81));
    jsim_.resize(n_);
    grav_.resize(n_);
    q_min_.data.resize(n_);
    q_max_.data.resize(n_);
    // q_min_.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from file
    // q_max_.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from file          
  
    ikVelSol_ = new KDL::ChainIkSolverVel_pinv(chain_); //Inverse velocity solver 
}

void KDLRobot::getInverseKinematics(KDL::Frame &f, KDL::JntArray &q){
    //std::cout << "Initial joint positions: ";
    //printJntArray(jntArray_,this->getNrJnts());
    int ret = ikSol_->CartToJnt(jntArray_,f,q);
    if(ret != 0) {std::cout << ikSol_->strError(ret) << std::endl;};
}

//(3a)
void KDLRobot::getInverseKinematicsVel(KDL::Twist &v, KDL::JntArray &qdot){
    //std::cout << "Initial joint velocities: ";
    //printJntArray(jntVel_,this->getNrJnts());
    int ret = ikVelSol_->CartToJnt(jntArray_,v,qdot);
    if(ret != 0) {std::cout << ikVelSol_->strError(ret) << std::endl;};
}

/*
//(3a)
void KDLRobot::getInverseKinematicsAcc(KDL::Twist &a, KDL::JntArray &qddot, KDL::JntArray &q_in, KDL::JntArray &qdot_in){
    int ret = ikAccSol_->CartToJnt(q_in, qdot_in, a, qddot);
    if(ret != 0) {std::cout << ikAccSol_->strError(ret) << std::endl;};
}

void KDLRobot::getInverseKinematicsAcc(KDL::FrameAcc &inputCartesian, KDL::JntArrayAcc &outputJoint){
    int ret = ikAccSol_->CartTojnt(jntArray_, inputCartesian, outputJoint);
    if(ret != 0) {std::cout << ikAccSol_->strError(ret) << std::endl;};
}
*/

void KDLRobot::getDirectKinematics(const KDL::JntArray &q, KDL::Frame &f) {
    int ret = fkSol_->JntToCart(q,f);
    if(ret != 0) {std::cout << fkSol_->strError(ret) << std::endl;};
}

void KDLRobot::getDirectKinematicsPosVel(const KDL::JntArrayVel &q_pos_vel, KDL::FrameVel &f_pos_vel) {
    int ret = fkVelSol_->JntToCart(q_pos_vel,f_pos_vel);
    if(ret != 0) {std::cout << fkVelSol_->strError(ret) << std::endl;};
}

int KDLRobot::getDirectKinematicsAcc(const KDL::JntArrayAcc &q_in, KDL::FrameAcc &out, int segmentNr) {
    std::cout << "segment nr: " << segmentNr << std::endl;
    // Ensure segmentNr is valid
    if (segmentNr < -1 || segmentNr >= (int)getNrSgmts()) {
        return -1; // Invalid segment number
    }
    std::cout << "No of segments: " << getNrSgmts() << std::endl;
    // Initialize base frame and velocities
    KDL::FrameAcc current_frame = KDL::FrameAcc::Identity();
    KDL::Twist current_vel = KDL::Twist::Zero();
    KDL::Twist current_acc = KDL::Twist::Zero();

    // Loop through segments
    for (size_t i = 0; i < chain_.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain_.getSegment(i);

        // Joint data for this segment
        const KDL::Joint& joint = segment.getJoint();
        double q = q_in.q(i);         // Position
        double q_dot = q_in.qdot(i);  // Velocity
        double q_dotdot = q_in.qdotdot(i); // Acceleration
        std::cout << "q: " << q << " qdot: " << q_dot << " qddot: " << q_dotdot << std::endl;

        // Compute joint pose, velocity, and acceleration
        KDL::Frame joint_frame = joint.pose(q);
        KDL::Twist joint_twist = joint.twist(q_dot);
        KDL::Twist joint_acc = joint.twist(q_dotdot);

        // Compute segment contribution
        current_frame = multiplyFrameAcc(current_frame,KDL::FrameAcc(joint_frame, joint_twist, joint_acc));
        current_vel = current_vel + joint_twist;
        current_acc = current_acc + joint_acc;

        std::cout << "current frame " << i << ":\n";
        std::cout << current_frame.p.p.data[0] << " " << current_frame.p.p.data[1] << " " << current_frame.p.p.data[2] << std::endl;
        std::cout << current_frame.p.v.data[0] << " " << current_frame.p.v.data[1] << " " << current_frame.p.v.data[2] << std::endl;
        std::cout << current_frame.p.dv.data[0] << " " << current_frame.p.dv.data[1] << " " << current_frame.p.dv.data[2] << std::endl;
        (void) getchar();
    }

    // Add this segment's frame to the output
    out = current_frame;
    return 0; // Success
}

KDL::FrameAcc KDLRobot::multiplyFrameAcc(const KDL::FrameAcc& parent, const KDL::FrameAcc& child) {
    // Extract parent components
    const KDL::Frame& F_p = parent.GetFrame();      // Parent frame
    const KDL::Twist& V_p = parent.GetTwist();     // Parent velocity
    const KDL::Twist& A_p = parent.GetAccTwist(); // Parent acceleration

    // Extract child components
    const KDL::Frame& F_c = child.GetFrame();      // Child frame
    const KDL::Twist& V_c = child.GetTwist();     // Child velocity
    const KDL::Twist& A_c = child.GetAccTwist(); // Child acceleration

    // Compute the new frame
    KDL::Frame F_new = F_p * F_c;

    // Transform child velocity into parent frame
    KDL::Twist V_c_in_p = F_p.M * V_c.RefPoint(F_c.p);

    // Compute the combined velocity
    KDL::Twist V_new = V_p + V_c_in_p;

    // Transform child acceleration into parent frame
    KDL::Twist A_c_in_p = F_p.M * A_c.RefPoint(F_c.p) +
                     KDL::Twist(F_p.M * V_c.vel, F_p.M * V_c.rot) * V_p;

    // Compute the combined acceleration
    KDL::Twist A_new = A_p + A_c_in_p;

    // Return the new FrameAcc
    return KDL::FrameAcc(F_new, V_new, A_new);
}


void KDLRobot::setJntLimits(KDL::JntArray &q_low, KDL::JntArray &q_high)
{
    q_min_ = q_low; q_max_ = q_high;
    ikSol_ = new KDL::ChainIkSolverPos_NR_JL(chain_,
                                            *fkSol_,
                                            *ikVelSol_,
                                            300,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
    ikSol_->setJointLimits(q_min_,q_max_);
}

// Updates the internal state of the robot given _jnt_values and _jnt_vel
void KDLRobot::update(std::vector<double> _jnt_values, std::vector<double> _jnt_vel)
{
    int err;
    updateJnts(_jnt_values, _jnt_vel);

    KDL::Twist s_T_f;
    KDL::Frame s_F_f;
    KDL::Jacobian s_J_f(7);
    KDL::Jacobian s_J_dot_f(7);
    KDL::FrameVel s_Fv_f;
    KDL::JntArrayVel jntVel(jntArray_,jntVel_);
    KDL::Twist s_J_dot_q_dot_f;

    // joints space
    err = dynParam_->JntToMass(jntArray_, jsim_); if(err != 0) {std::cout << strError(err);};
    err = dynParam_->JntToCoriolis(jntArray_, jntVel_, coriol_); if(err != 0) {std::cout << strError(err);};
    err = dynParam_->JntToGravity(jntArray_, grav_); if(err != 0) {std::cout << strError(err);};

    // robot flange
    err = fkVelSol_->JntToCart(jntVel, s_Fv_f); if(err != 0) {std::cout << strError(err);};
    s_T_f = s_Fv_f.GetTwist();
    s_F_f = s_Fv_f.GetFrame();
    err = jacSol_->JntToJac(jntArray_, s_J_f); if(err != 0) {std::cout << strError(err);};
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_q_dot_f); if(err != 0) {std::cout << strError(err);};
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_f); if(err != 0) {std::cout << strError(err);};

    // robot end-effector
    s_F_ee_ = s_F_f*f_F_ee_;
    KDL::Vector s_p_f_ee = s_F_ee_.p - s_F_f.p;
    KDL::changeRefPoint(s_J_f, s_p_f_ee, s_J_ee_);
    KDL::changeRefPoint(s_J_dot_f, s_p_f_ee, s_J_dot_ee_);
    KDL::changeBase(s_J_ee_, s_F_ee_.M.Inverse(), b_J_ee_);
    KDL::changeBase(s_J_dot_ee_, s_F_ee_.M.Inverse(), b_J_dot_ee_);
    s_V_ee_ = s_T_f.RefPoint(s_p_f_ee);
}


////////////////////////////////////////////////////////////////////////////////
//                                 CHAIN                                      //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::createChain(KDL::Tree &robot_tree)
{
    //if(!robot_tree.getChain(robot_tree.getRootSegment()->first, "lbr_iiwa_link_7",chain_))
    if(!robot_tree.getChain(robot_tree.getRootSegment()->first, 
        std::prev(std::prev(robot_tree.getSegments().end()))->first, chain_))
    {
        std::cout << "Failed to create KDL robot" << std::endl;
        return;
    }
    std::cout << "KDL robot model created" << std::endl;
    std::cout << "with " << chain_.getNrOfJoints() << " joints" << std::endl;
    std::cout << "and " << chain_.getNrOfSegments() << " segments" << std::endl;
}

unsigned int KDLRobot::getNrJnts()
{
    return n_;
}

unsigned int KDLRobot::getNrSgmts()
{
    return chain_.getNrOfSegments();
}

////////////////////////////////////////////////////////////////////////////////
//                                 JOINTS                                     //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::updateJnts(std::vector<double> _jnt_pos, std::vector<double> _jnt_vel)
{
    for (unsigned int i = 0; i < n_; i++)
    {
        jntArray_(i) = _jnt_pos[i];
        jntVel_(i) = _jnt_vel[i];
    }
}
Eigen::VectorXd KDLRobot::getJntValues()
{
    return jntArray_.data;
}

Eigen::VectorXd KDLRobot::getJntVelocities()
{
    return jntVel_.data;
}

Eigen::MatrixXd KDLRobot::getJntLimits()
{
    Eigen::MatrixXd jntLim;
    jntLim.resize(n_,2);

    jntLim.col(0) = q_min_.data;
    jntLim.col(1) = q_max_.data;

    return jntLim;
}

Eigen::MatrixXd KDLRobot::getJsim()
{
    return jsim_.data;
}

Eigen::VectorXd KDLRobot::getCoriolis()
{
    return coriol_.data;
}

Eigen::VectorXd KDLRobot::getGravity()
{
    return grav_.data;
}

Eigen::VectorXd KDLRobot::getID(const KDL::JntArray &q,
                                const KDL::JntArray &q_dot,
                                const KDL::JntArray &q_dotdot,
                                const KDL::Wrenches &f_ext)
{
    Eigen::VectorXd t;
    t.resize(chain_.getNrOfJoints());
    KDL::JntArray torques(chain_.getNrOfJoints());
    int r = idSolver_->CartToJnt(q,q_dot,q_dotdot,f_ext,torques);
    std::cout << "idSolver result: " << idSolver_->strError(r) << std::endl;
    // std::cout << "torques: " << torques.data.transpose() << std::endl;
    t = torques.data;
    return t;
}

////////////////////////////////////////////////////////////////////////////////
//                              END-EFFECTOR                                  //
////////////////////////////////////////////////////////////////////////////////

KDL::Frame KDLRobot::getEEFrame()
{
    return s_F_ee_;
}


KDL::Twist KDLRobot::getEEVelocity()
{
    return s_V_ee_;
}

KDL::Twist KDLRobot::getEEBodyVelocity()
{
    return s_V_ee_;
}

KDL::Jacobian KDLRobot::getEEJacobian()
{
    return s_J_ee_;
}

KDL::Jacobian KDLRobot::getEEBodyJacobian()
{
    //    KDL::Frame ee_F_s = this->getEEPose().Inverse();
    //    KDL::Vector pkdl = ee_F_s.p;
    //    KDL::Rotation M = ee_F_s.M;
    //    std::cout << adjoint(toEigen(pkdl),toEigen(M))*s_J_ee_.data << std::endl;
    //    s_J_ee_.changeRefFrame(ee_F_s);
    //    std::cout << s_J_ee_.data << std::endl;
    //    return adjoint(toEigen(pkdl),toEigen(M))*s_J_ee_.data;
    return b_J_ee_;
}

Eigen::VectorXd KDLRobot::getEEJacDotqDot()
{
    return s_J_dot_ee_.data*jntVel_.data;
}

void KDLRobot::addEE(const KDL::Frame &_f_F_ee)
{
    f_F_ee_ = _f_F_ee;
    this->update(toStdVector(this->jntArray_.data), toStdVector(this->jntVel_.data));
}

////////////////////////////////////////////////////////////////////////////////
//                              OTHER FUNCTIONS                               //
////////////////////////////////////////////////////////////////////////////////
// Implementation copied from <kdl/isolveri.hpp> because
// KDL::ChainDynSolver inherits *privately* from SolverI ... -.-'
std::string KDLRobot::strError(const int error) {
  
  // clang-format off
  switch(error) {
  case KDL::SolverI::E_NOERROR:                 return "No error \n"; break;
  case KDL::SolverI::E_NO_CONVERGE:             return "[ERROR] Failed to converge \n"; break;
  case KDL::SolverI::E_UNDEFINED:               return "[ERROR] Undefined value \n"; break;
  case KDL::SolverI::E_DEGRADED:                return "[ERROR] Converged but degraded solution \n"; break;

  // These were introduced in melodic
  case KDL::SolverI::E_NOT_UP_TO_DATE:          return "[ERROR] Internal data structures not up to date with Chain \n"; break;
  case KDL::SolverI::E_SIZE_MISMATCH:           return "[ERROR] The size of the input does not match the internal state \n"; break;
  case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED: return "[ERROR] The maximum number of iterations is exceeded \n"; break;
  case KDL::SolverI::E_OUT_OF_RANGE:            return "[ERROR] The requested index is out of range \n"; break;
  case KDL::SolverI::E_NOT_IMPLEMENTED:         return "[ERROR] The requested function is not yet implemented \n"; break;
  case KDL::SolverI::E_SVD_FAILED:              return "[ERROR] SVD failed \n"; break;

  default: return "UNKNOWN ERROR";
  }
  // clang-format on
}
