/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_COMPOSITE_IK_CPP_H
#define UCNOID_BODY_COMPOSITE_IK_CPP_H

#include "CompositeIK.h"
#include "Link.h"

namespace cnoid {
inline namespace ucnoid {

inline CompositeIK::CompositeIK()
{
    targetLink_ = 0;
    hasAnalyticalIK_ = false;
}


inline CompositeIK::CompositeIK(Body* body, Link* targetLink)
{
    reset(body, targetLink);
}


inline CompositeIK::~CompositeIK()
{

}


inline void CompositeIK::reset(Body* body, Link* targetLink)
{
    body_ = body;
    targetLink_ = targetLink;
    hasAnalyticalIK_ = false;
    paths.clear();
}
   

inline bool CompositeIK::addBaseLink(Link* baseLink)
{
    if(baseLink && targetLink_){
        JointPathPtr path = getCustomJointPath(body_, targetLink_, baseLink);
        if(path){
            hasAnalyticalIK_ = paths.empty() ? path->hasAnalyticalIK() : (hasAnalyticalIK_ && path->hasAnalyticalIK());
            paths.push_back(path);
            return true;
        }
    }
    return false;
}


inline void CompositeIK::setMaxIKerror(double e)
{
    for(size_t i=0; i < paths.size(); ++i){
        paths[i]->setNumericalIKmaxIKerror(e);
    }
}


inline bool CompositeIK::calcInverseKinematics(const Position& T)
{
    const int n = body_->numJoints();

    Position T0 = targetLink_->T();
    q0.resize(n);
    for(int i=0; i < n; ++i){
        q0[i] = body_->joint(i)->q();
    }

    targetLink_->setPosition(T);

    bool solved = true;
    size_t pathIndex;
    for(pathIndex=0; pathIndex < paths.size(); ++pathIndex){
        JointPath& path = *paths[pathIndex];
        Link* link = path.endLink();
        Position T_end = link->T();
        solved = path.setBaseLinkGoal(T).calcInverseKinematics(T_end);
        if(!solved){
            link->setPosition(T_end);
            break;
        }
    }

    if(!solved){
        targetLink_->setPosition(T0);
        for(int i=0; i < n; ++i){
            body_->joint(i)->q() = q0[i];
        }
        for(size_t i=0; i < pathIndex; ++i){
            paths[i]->calcForwardKinematics();
        }
    }

    return solved;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_COMPOSITE_IK_CPP_H
