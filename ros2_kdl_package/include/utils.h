#ifndef UTILS_H
#define UTILS_H

#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "Eigen/Dense"
#include <iostream>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

inline KDL::Vector toKDL(const Eigen::Vector3d& v) //override function: converts a lot of stuff in KDL::Vector
{
    return KDL::Vector(v[0],v[1],v[2]);
}

inline KDL::Wrench toKDLWrench(const Vector6d& w)
{
    return KDL::Wrench(KDL::Vector(w[0], w[1], w[2]), KDL::Vector(w[3], w[4], w[5]));
}

inline KDL::Twist toKDLTwist(const Vector6d& t)
{
    return KDL::Twist(KDL::Vector(t[0], t[1], t[2]), KDL::Vector(t[3], t[4], t[5]));
}

inline KDL::Frame toKDL(const std::vector<double>& v) //given an RPY and Position evalueate the tern multiplying the rot matricies and 
{//.at(i) is a method that return the ref of the elem at pos in the vector
    return KDL::Frame(KDL::Rot(KDL::Vector(v.at(3),0,0))*
                      KDL::Rot(KDL::Vector(0,v.at(4),0))*
                      KDL::Rot(KDL::Vector(0,0,v.at(5))),
                      KDL::Vector(v.at(0),v.at(1),v.at(2)));
}

inline Eigen::MatrixXd toEigen(const std::vector<Eigen::Matrix<double,3,7>>& J)
{
    Eigen::Matrix<double,12,7> Ja;
    for (long unsigned int i = 0; i < J.size(); i ++)
    {
        Ja.block(i*3,0,3,7) = J.at(i);
    }
    return Ja;
}

inline Eigen::Vector3d toEigen(const KDL::Vector& v)
{
    return Eigen::Vector3d(v.x(),v.y(),v.z());
}

inline Eigen::Matrix3d toEigen(const KDL::Rotation& M)
{
    Eigen::Matrix3d E;
    for (unsigned int i = 0; i < 9; ++i)
        E(i/3, i%3) = M.data[i];
    return E;
}

inline Vector6d toEigen(const KDL::Wrench& w)
{
    Vector6d e;
    e << toEigen(w.force), toEigen(w.torque);
    return e;
}

inline Vector6d toEigen(const KDL::Twist& t)
{
    Vector6d e;
    e << toEigen(t.vel), toEigen(t.rot);
    return e;
}


inline std::vector<double> toStdVector(const Eigen::VectorXd& V)
{
    std::vector<double> v;
    v.resize(V.size());
    Eigen::VectorXd::Map(&v[0], V.size()) = V;
    return v;
}

inline Matrix6d spatialRotation(const KDL::Rotation& M)
{
    Matrix6d R = Matrix6d::Identity();
    R.block(0,0,3,3) = toEigen(M);
    R.block(3,3,3,3) = toEigen(M);

    return R;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d & t)
{
    Eigen::Matrix3d t_hat;

    t_hat <<     0, -t[2],  t[1],
              t[2],     0, -t[0],
             -t[1],  t[0],     0;

    return t_hat;
}

inline KDL::Jacobian adjoint(const KDL::Frame & F, const KDL::Jacobian &_J)
{
    Matrix6d ad;
    ad.setZero();
    ad.block(0,0,3,3) = toEigen(F.M);
    ad.block(0,3,3,3) = skew(toEigen(F.p))*toEigen(F.M);;
    ad.block(3,3,3,3) = toEigen(F.M);;

    Eigen::MatrixXd J_;
    J_.resize(6, _J.columns());
    J_ = ad*_J.data;

    KDL::Jacobian J(_J.columns());
    J.data = J_;

    return J;
}


inline KDL::Twist adjoint(const KDL::Frame & F, const KDL::Twist &T)
{
    Matrix6d ad;
    ad.setZero();

    Vector6d t;
    t << toEigen(T.vel), toEigen(T.rot);

    ad.block(0,0,3,3) = toEigen(F.M);
    ad.block(0,3,3,3) = skew(toEigen(F.p))*toEigen(F.M);;
    ad.block(3,3,3,3) = toEigen(F.M);;

    Vector6d t_;
    t_ = ad*t;

    KDL::Twist a(KDL::Vector(t_[0], t_[1], t_[2]), KDL::Vector(t_[3], t_[4], t_[5]));

    return a;
}


inline Matrix6d adjoint(const Eigen::Vector3d & p, const Eigen::Matrix3d & R)
{
    Matrix6d ad;
    ad.setZero();

    ad.block(0,0,3,3) = R;
    ad.block(0,3,3,3) = skew(p)*R;
    ad.block(3,3,3,3) = R;

    return ad;
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

inline Eigen::Matrix<double,3,1> computeOrientationError(const Eigen::Matrix<double,3,3> &_R_d, 
                                                         const Eigen::Matrix<double,3,3> &_R_e)
{

    Eigen::Matrix<double,3,1> e_o;
    e_o << 0.5*(skew(_R_e.col(0))*(_R_d.col(0)) + skew(_R_e.col(1))*(_R_d.col(1)) + skew(_R_e.col(2))*(_R_d.col(2)));
//    std::cout << "e_o angle/axis: " << std::endl << e_o.transpose() << std::endl;
//    Eigen::Quaterniond q_e(_R_e);
//    Eigen::Quaterniond q_d(_R_d);
//    Eigen::Quaterniond q = q_d*q_e.inverse();
//    e_o << q.x(), q.y(), q.z();
//    std::cout << "e_o quaternion: " << std::endl << e_o.transpose() << std::endl;
//    std::cout << "e_o norm: " << std::endl << e_o.norm() << std::endl;
//    std::cout << "R_d: " << std::endl << _R_d << std::endl;
//    std::cout << "R_e: " << std::endl << _R_e << std::endl;
    return e_o;
}

inline Eigen::Matrix<double,3,1> computeLinearError(const Eigen::Matrix<double,3,1> &_p_d,
                                                    const Eigen::Matrix<double,3,1> &_p_e)
{
    return _p_d - _p_e;
}

inline Eigen::Matrix<double,3,1> computeOrientationVelocityError(const Eigen::Matrix<double,3,1> &_omega_d,
                                                                 const Eigen::Matrix<double,3,1> &_omega_e,
                                                                 const Eigen::Matrix<double,3,3> &_R_d,
                                                                 const Eigen::Matrix<double,3,3> &_R_e)
{
    Eigen::Matrix3d L = -0.5*(skew(_R_d.col(0))*skew(_R_e.col(0)) +
                              skew(_R_d.col(1))*skew(_R_e.col(1)) +
                              skew(_R_d.col(2))*skew(_R_e.col(2)));
    return L.inverse()*(L.transpose()*_omega_d - L*_omega_e);
}


inline void computeErrors(const KDL::Frame &_Fd,
                          const KDL::Frame &_Fe,
                          const KDL::Twist &_Vd,
                          const KDL::Twist &_Ve,
                          Vector6d &e,
                          Vector6d &edot)
{

    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_Fd.M.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(_Fe.M.data);
    e << computeLinearError(toEigen(_Fd.p), toEigen(_Fe.p)), computeOrientationError(R_d, R_e);
    edot << computeLinearError(toEigen(_Vd.vel), toEigen(_Ve.vel)), -toEigen(_Ve.rot);//computeOrientationVelocityError(toEigen(_Vd.rot), toEigen(_Ve.rot), R_d, R_e);
}

inline Eigen::MatrixXd weightedPseudoInverse(const Eigen::MatrixXd &W,
                                             const Eigen::MatrixXd &Mat)
{
    return W.inverse()*Mat.transpose()*(Mat*W.inverse()*Mat.transpose()).inverse();
}

inline Eigen::MatrixXd gradientJointLimits(const Eigen::VectorXd &q, const Eigen::MatrixXd &jntLimits, double &costValue)
{
    int n = q.size();
    costValue = 0;
    double gamma = 1.0;
    Eigen::VectorXd gradient;
    gradient.resize(n);

    for(int i = 0; i < n; i++)
    {
        if (q(i,0) < jntLimits(i,0) || q(i,0) > jntLimits(i,1))
        {
            std::cout << "joint " << i << " limits violated. Value = "<< q(i,0)*180.0/M_PI << "\n";
        }
        gradient(i,0) = 1.0/gamma * std::pow((jntLimits(i,1) - jntLimits(i,0)),2)* (2*q(i,0) - jntLimits(i,1) - jntLimits(i,0))/(std::pow((jntLimits(i,1)-q(i,0)),2)*std::pow((q(i,0)-jntLimits(i,0)),2));
        costValue = costValue + 1.0/gamma * std::pow((jntLimits(i,1) - jntLimits(i,0)),2)/((jntLimits(i,1)-q(i,0))*(q(i,0)-jntLimits(i,0)));
    }
    return gradient;
}

//Matrix ortonormalization
inline Eigen::MatrixXd matrixOrthonormalization(Eigen::MatrixXd R){

   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(R.transpose()*R);
   Eigen::Vector3d D = es.eigenvalues();
   Eigen::Matrix3d V = es.eigenvectors();
   R = R*((1/sqrt(D(0)))*V.col(0)*V.col(0).transpose() + (1/sqrt(D(1)))*V.col(1)*V.col(1).transpose() + (1/sqrt(D(2)))*V.col(2)*V.col(2).transpose());

   return R;
}

inline void printFrame(const KDL::Frame & F) {
    std::cout << "Position: " << std::endl;
    for(int i=0; i<3; i++)
        std::cout << F.p(i) << " ";
    std::cout << std::endl;

    std::cout << "Rotation: " << std::endl;
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++)
            std::cout << F.M(i,j) << " ";
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

inline void printTwist(const KDL::Twist & T) {
    std::cout << "Linear velocity: " << std::endl;
    for(int i=0; i<3; i++)
        std::cout << T.vel(i) << " ";
    std::cout << std::endl;

    std::cout << "Rotational velocity: " << std::endl;
    for(int i=0; i<3; i++)
        std::cout << T.rot(i) << " ";
    std::cout << std::endl;
}

inline void printEigenVector3d(const Eigen::Vector3d & v) {
    for(int i=0; i<3; i++)
        std::cout << v[i] << " ";
    std::cout << std::endl;
}

inline void printJntArray(const KDL::JntArray & A, const unsigned int l) {
    for(unsigned int i=0; i<l; i++)
        std::cout << A(i) << " ";
    std::cout << std::endl;
}

#endif