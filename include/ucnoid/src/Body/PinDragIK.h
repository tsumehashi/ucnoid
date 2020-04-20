/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_PIN_DRAG_IK_H
#define UCNOID_BODY_PIN_DRAG_IK_H

#include "InverseKinematics.h"
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class Body;
class Link;
class PinDragIKImpl;

class UCNOID_EXPORT PinDragIK : public InverseKinematics
{
public:
    PinDragIK(Body* body);
    ~PinDragIK();

    Body* body() const;
  
    void setBaseLink(Link* baseLink);
    void setFreeRootWeight(double translation, double rotation);
    void setTargetLink(Link* targetLink, bool isAttitudeEnabled = false);
    void setJointWeight(int jointId, double weight);

    void setPin(Link* link, InverseKinematics::AxisSet axes = InverseKinematics::TRANSLATION_3D, double weight = 1.0);
    InverseKinematics::AxisSet pinAxes(Link* link);
    void clearPins();
    int numPinnedLinks();
            
    virtual void setIKErrorThresh(double e);
    virtual bool hasAnalyticalIK();
    virtual InverseKinematics::AxisSet targetAxes() const;
    void setSRInverseParameters(double k0, double w0);
    void enableJointRangeConstraints(bool on);

    /**
       this must be called before the initial calcInverseKinematics() call
       after settings have been changed.
    */
    bool initialize();

    virtual bool calcInverseKinematics(const Position& T) override;

private:
    PinDragIKImpl* impl;
};

typedef std::shared_ptr<PinDragIK> PinDragIKptr;

}   // inline namespace ucnoid
}

#include "PinDragIK.cpp.h"

#endif
