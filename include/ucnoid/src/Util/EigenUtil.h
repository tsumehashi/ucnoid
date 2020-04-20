/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_UTIL_H
#define CNOID_UTIL_EIGEN_UTIL_H

#include "EigenTypes.h"
#include <memory>

#if defined(WIN32) && !defined(BOOST_NO_CXX11_ALLOCATOR)
#include <boost/version.hpp>
#if (BOOST_VERSION >= 105900)
#define BOOST_NO_CXX11_ALLOCATOR
#endif
#endif

#include "exportdecl.h"

namespace cnoid {

const double PI = 3.14159265358979323846;
const double PI_2 = 1.57079632679489661923;

const double TO_DEGREE = 180.0 / PI;
const double TO_RADIAN = PI / 180.0;

inline double degree(double rad) { return TO_DEGREE * rad; }
inline double radian(double deg) { return TO_RADIAN * deg; }
inline float degree(float rad) { return (float)TO_DEGREE * rad; }
inline float radian(float deg) { return (float)TO_RADIAN * deg; }
inline double radian(int deg) { return TO_RADIAN * deg; }

/*
  Since version 3.2, the behavior of Eigen's eulerAngles function was slightly modified;
  The returned angles are in the ranges [0:pi]x[0:pi]x[-pi:pi].
  This is not good for using the returned angles to interpolate attitdues.
  Now our own implementation is used for getting R-P-Y angles.
*/
/*
  template<typename Derived>
  inline Eigen::Matrix<typename Eigen::MatrixBase<Derived>::Scalar, 3, 1>
  rpyFromRot(const Eigen::MatrixBase<Derived>& R) {
  Vector3 ea = R.eulerAngles(2, 1, 0);
  return Vector3(ea[2], ea[1], ea[0]); // exchange element order to be our conventional one !
  }
*/

CNOID_EXPORT Vector3 rpyFromRot(const Matrix3& R);

CNOID_EXPORT Matrix3 rotFromRpy(double r, double p, double y);

inline Matrix3 rotFromRpy(const Vector3& rpy) {
    return rotFromRpy(rpy[0], rpy[1], rpy[2]);
}

CNOID_EXPORT Vector3 omegaFromRot(const Matrix3& R);

inline Matrix3 hat(const Vector3& x) {
    Matrix3 M;
    M <<  0.0, -x(2),   x(1),
        x(2),   0.0,  -x(0),
        -x(1),  x(0),   0.0;
    return M;
}

CNOID_EXPORT std::string str(const Vector3& v);
CNOID_EXPORT std::string str(const Vector3f& v);
CNOID_EXPORT std::string str(const Vector2& v);
CNOID_EXPORT std::string str(const AngleAxis& a);
CNOID_EXPORT bool toVector3(const std::string& s, Vector3& out_v);
CNOID_EXPORT bool toVector3(const std::string& s, Vector3f& out_v);


CNOID_EXPORT void normalizeRotation(Matrix3& R);
CNOID_EXPORT void normalizeRotation(Position& T);
CNOID_EXPORT void normalizeRotation(Affine3& T);

template<class T, typename... Args>
std::shared_ptr<T> make_shared_aligned(Args&&... args)
{
  return std::allocate_shared<T>(Eigen::aligned_allocator<T>(), std::forward<Args>(args)...);
}

}

#endif
