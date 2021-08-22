/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_FORWARD_DYNAMICS_CPP_H
#define UCNOID_BODY_FORWARD_DYNAMICS_CPP_H

#include "ForwardDynamics.h"
#include "DyBody.h"

namespace cnoid {
inline namespace ucnoid {

inline ForwardDynamics::ForwardDynamics(DyBody* body)
    : body(body)
{
    g.setZero();
    timeStep = 0.005;

    integrationMode = RUNGEKUTTA_METHOD;
    sensorsEnabled = false;
}


inline ForwardDynamics::~ForwardDynamics()
{

}


inline void ForwardDynamics::setTimeStep(double ts)
{
    timeStep = ts;
}


inline void ForwardDynamics::setGravityAcceleration(const Vector3& g)
{
    this->g = g;
}


inline void ForwardDynamics::setEulerMethod()
{
    integrationMode = EULER_METHOD;
}


inline void ForwardDynamics::setRungeKuttaMethod()
{
    integrationMode = RUNGEKUTTA_METHOD;
}


inline void ForwardDynamics::enableSensors(bool on)
{
    sensorsEnabled = on;
}


inline void ForwardDynamics::setOldAccelSensorCalcMode(bool on)
{
    sensorHelper.setOldAccelSensorCalcMode(on);
}


/// function from Murray, Li and Sastry p.42
inline void ForwardDynamics::SE3exp
(Position& out_T, const Position& T0, const Vector3& w, const Vector3& vo, double dt)
{
    double norm_w = w.norm();
	
    if(norm_w < std::numeric_limits<double>::epsilon()) {
        out_T.linear() = T0.linear();
        out_T.translation() = T0.translation() + vo * dt;
    } else {
        double th = norm_w * dt;
        Vector3 w_n = w / norm_w;
        Vector3 vo_n = vo / norm_w;
        const Matrix3 R(AngleAxisd(th, w_n));
        out_T.translation() =
            R * T0.translation() + (Matrix3::Identity() - R) * w_n.cross(vo_n) + (w_n * w_n.transpose()) * vo_n * th;
        out_T.linear() = R * T0.linear();
    }
}


inline void ForwardDynamics::initializeSensors()
{
    body->initializeDeviceStates();

    if(sensorsEnabled){
        sensorHelper.initialize(body, timeStep, g);
    }
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_FORWARD_DYNAMICS_CPP_H
