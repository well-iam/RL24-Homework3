#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"
#include <vector>

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

/*
struct s_point{
	double pos;
	double vel;
	double acc;
};
*/
class KDLPlanner
{

public:

    KDLPlanner();
    KDLPlanner(double _maxVel, double _maxAcc);
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    //////////////////////////////////

    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);


    trajectory_point compute_trajectory(double time);


    //------Our func-------- vvvvvvvvv

    // Point (1b): We define a costructor to evalueate offline the coefficient a3,a2,a1,a0
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    void calculateCoefficients(double _trajDuration);


    //Point (2a) : We define a new costructor as requested
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius);

    //Constructor for (2b)
    KDLPlanner(double _trajDuration,double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius);

    //Point (1a)
    void trapezoidal_vel(double time, double _accDuration,double &pos,double &vel ,double &acc);

    //Point (1b)
    void cubic_polinomial(double time,double &pos,double &vel ,double &acc);

    //Point (2b)
    trajectory_point compute_trajectory_circular(double time);
    trajectory_point compute_trajectory_circular(double time,double _accDuration);
    
    //(2c)
    trajectory_point compute_trajectory_linear(double time);

    //------Our func-------- ^^^^^^^^^

private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_, accDuration_;
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;

    //Our member
    double a3,a2,a1,a0;
    double trajRadius_;

};

#endif
