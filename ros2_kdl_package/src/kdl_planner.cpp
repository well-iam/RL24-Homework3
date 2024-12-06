#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}



KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    //Constructor for Lin and Traps
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
    return traject_;
}

//----Our Func---- vvv

//Constructor for (1b).
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    calculateCoefficients(trajDuration_);
} 


//(1b): Define a function called in constructors where i'm supposed to use a 3rd grade polynomia for s
void KDLPlanner::calculateCoefficients(double _trajDuration){
    a0=0;
    a1=0;
    a2=3.0/std::pow(_trajDuration,2);
    a3= -2.0/std::pow(_trajDuration,3);
}

//Constructor for (2a)
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius){
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    calculateCoefficients(_trajDuration);
}

//Constructor for (2b)
KDLPlanner::KDLPlanner(double _trajDuration,double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius){
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

//(1a)
void KDLPlanner::trapezoidal_vel(double time, double _accDuration, double& s, double& sdot, double& sddot) {
    accDuration_ = _accDuration;
    double ddot_s_c = -1.0 / (std::pow(accDuration_, 2) - trajDuration_ * accDuration_);
    if (time <= accDuration_) {
        s = 0.5 * ddot_s_c * std::pow(time, 2);
        sdot = ddot_s_c * time;
        sddot = ddot_s_c;
    }
    else if (time <= trajDuration_ - accDuration_) {
        s = ddot_s_c * accDuration_ * (time - accDuration_ / 2);
        sdot = ddot_s_c * accDuration_;
        sddot = 0;
    }
    else {
        s = 1 - 0.5 * ddot_s_c * std::pow(trajDuration_ - time, 2);
        sdot = ddot_s_c * (trajDuration_ - time);
        sddot = -ddot_s_c;
    }
}

//(1b)
void KDLPlanner::cubic_polinomial(double time, double& s, double& sdot, double& sddot) {
    s = a3 * std::pow(time, 3) + a2 * std::pow(time, 2) + a1 * time + a0;
    sdot = 3 * a3 * std::pow(time, 2) + 2 * a2 * time + a1;
    sddot = 6 * a3 * time + 2 * a2;
}

//(2b): Version for Cubic and circular
trajectory_point KDLPlanner::compute_trajectory_circular(double time) {
    trajectory_point traj;
    Eigen::Vector3d trajCenter(trajInit_ + Eigen::Vector3d(0, trajRadius_, 0));
    double s, sdot, sddot;
    const double _alpha_ = 1 * 3.14;
    cubic_polinomial(time, s, sdot, sddot);

   // std::cout << "Circuling: " << std::endl;
    traj.pos << trajCenter[0],
        trajCenter[1] - trajRadius_ * cos(_alpha_ * s),
        trajCenter[2] + trajRadius_ * sin(_alpha_ * s);

    traj.vel << 0,
        _alpha_* trajRadius_* sin(_alpha_ * s)* sdot,
        +_alpha_ * trajRadius_ * cos(_alpha_ * s) * sdot;

    traj.acc << 0,
        std::pow(_alpha_, 2)* trajRadius_* cos(_alpha_ * s)* std::pow(sdot, 2) + _alpha_*trajRadius_*sin(_alpha_*s)*sddot,
        -std::pow(_alpha_, 2)* trajRadius_* sin(_alpha_ * s)* std::pow(sdot, 2) + _alpha_*trajRadius_*cos(_alpha_*s)*sddot;

    return traj;
}
//(2b): Version for Traps and circular
trajectory_point KDLPlanner::compute_trajectory_circular(double time, double _accDuration) {
    trajectory_point traj;
    Eigen::Vector3d trajCenter(trajInit_ + Eigen::Vector3d(0, trajRadius_, 0));
    double s, sdot, sddot;
    const double _alpha_ = 1 * 3.14;
    trapezoidal_vel(time, _accDuration, s, sdot, sddot);

    traj.pos << trajCenter[0],
        trajCenter[1] - trajRadius_ * cos(_alpha_ * s),
        trajCenter[2] + trajRadius_ * sin(_alpha_ * s);

    traj.vel << 0,
        _alpha_* trajRadius_* sin(_alpha_ * s)* sdot,
        _alpha_ * trajRadius_ * cos(_alpha_ * s) * sdot;

    traj.acc << 0,
        std::pow(_alpha_, 2)*trajRadius_*cos(_alpha_*s)*std::pow(sdot, 2) + _alpha_*trajRadius_*sin(_alpha_*s)*sddot,
        -std::pow(_alpha_, 2)*trajRadius_*sin(_alpha_*s)*std::pow(sdot, 2) + _alpha_*trajRadius_*cos(_alpha_*s)*sddot;
    return traj;
}

//(2c)
trajectory_point KDLPlanner::compute_trajectory_linear(double time) {
    trajectory_point traj;
    Eigen::Vector3d distance = trajEnd_ - trajInit_;

    double s, sdot, sddot;
    cubic_polinomial(time, s, sdot, sddot);

    traj.pos = trajInit_ + s * distance;
    //std::cout << "Traj pos: " << traj.s[0] << ' ' << traj.s[1] << ' ' << traj.s[2] << std::endl;
    traj.vel = sdot * distance;
    traj.acc = sddot * distance;

    return traj;
}

//----Our Func---- ^^^


//This is for linear path
trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}
