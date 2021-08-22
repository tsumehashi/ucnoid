/**
   \file
   \author Shin'ichiro Nakaoka
*/
#ifndef UCNOID_BODY_LINK_CPP_H
#define UCNOID_BODY_LINK_CPP_H

#include "Link.h"
#include "Material.h"
#include <ucnoid/SceneGraph>
#include <ucnoid/ValueTree>

namespace cnoid {
inline namespace ucnoid {

inline Link::Link()
{
    index_ = -1;
    jointId_ = -1;
    parent_ = 0;
    body_ = 0;
    T_.setIdentity();
    Tb_.setIdentity();
    Rs_.setIdentity();
    a_ = Vector3::UnitZ();
    jointType_ = FIXED_JOINT;
    actuationMode_ = NO_ACTUATION;
    q_ = 0.0;
    dq_ = 0.0;
    ddq_ = 0.0;
    u_ = 0.0;
    q_target_ = 0.0;
    dq_target_ = 0.0;
    v_.setZero();
    w_.setZero();
    dv_.setZero();
    dw_.setZero();
    c_.setZero();
    wc_.setZero();
    m_ = 0.0;
    I_.setIdentity();
    Jm2_ = 0.0;
    F_ext_.setZero();
    q_initial_ = 0.0;
    q_upper_ = std::numeric_limits<double>::max();
    q_lower_ = -std::numeric_limits<double>::max();
    dq_upper_ = std::numeric_limits<double>::max();
    dq_lower_ = -std::numeric_limits<double>::max();
    materialId_ = 0;
    info_ = new Mapping;
}


inline Link::Link(const Link& org)
    : name_(org.name_)
{
    index_ = -1; // should be set by a Body object
    jointId_ = org.jointId_;

    parent_ = 0;
    body_ = 0;

    T_ = org.T_;
    Tb_ = org.Tb_;
    Rs_ = org.Rs_;
    
    a_ = org.a_;
    jointType_ = org.jointType_;
    actuationMode_ = org.actuationMode_;

    q_ = org.q_;
    dq_ = org.dq_;
    ddq_ = org.ddq_;
    u_ = org.u_;

    q_target_ = org.q_target_;
    dq_target_ = org.dq_target_;

    v_ = org.v_;
    w_ = org.w_;
    dv_ = org.dv_;
    dw_ = org.dw_;
    
    c_ = org.c_;
    wc_ = org.wc_;
    m_ = org.m_;
    I_ = org.I_;
    Jm2_ = org.Jm2_;

    F_ext_ = org.F_ext_;

    q_initial_ = org.q_initial_;
    q_upper_ = org.q_upper_;
    q_lower_ = org.q_lower_;
    dq_upper_ = org.dq_upper_;
    dq_lower_ = org.dq_lower_;

    materialId_ = org.materialId_;

    //! \todo add the mode for doing deep copy of the following objects
    visualShape_ = org.visualShape_;
    collisionShape_ = org.collisionShape_;
    info_ = org.info_;
}


inline Link* Link::clone() const
{
    return new Link(*this);
}


inline Link::~Link()
{
    LinkPtr link = child_;
    while(link){
        link->parent_ = 0;
        LinkPtr next = link->sibling_;
        link->sibling_ = 0;
        link = next;
    }
}


inline void Link::initializeState()
{
    u_ = 0.0;
    dq_ = 0.0;
    ddq_ = 0.0;
    q_target_ = q_;
    dq_target_ = dq_;
    v_.setZero();
    w_.setZero();
    dv_.setZero();
    dw_.setZero();
    F_ext_.setZero();
}


inline void Link::setBody(Body* newBody)
{
    if(body_ != newBody){
        setBodySub(newBody);
    }
}


inline void Link::setBodySub(Body* newBody)
{
    body_ = newBody;
    for(Link* link = child_; link; link = link->sibling_){
        link->setBodySub(newBody);
    }
}


inline void Link::prependChild(Link* link)
{
    LinkPtr holder;
    if(link->parent_){
        holder = link;
        link->parent_->removeChild(link);
    }
    link->sibling_ = child_;
    child_ = link;
    link->parent_ = this;

    link->setBody(body_);
}


inline void Link::appendChild(Link* link)
{
    LinkPtr holder;
    if(link->parent_){
        holder = link;
        link->parent_->removeChild(link);
    }
    if(!child_){
        child_ = link;
        link->sibling_ = 0;
    } else {
        Link* lastChild = child_;
        while(lastChild->sibling_){
            lastChild = lastChild->sibling_;
        }
        lastChild->sibling_ = link;
        link->sibling_ = 0;
    }
    link->parent_ = this;

    link->setBody(body_);
}


inline bool Link::isOwnerOf(const Link* link) const
{
    if(link == this){
        return true;
    }
    for(const Link* owner = link->parent_; owner; owner = owner->parent_){
        if(owner == link){
            return true;
        }
    }
    return false;
}
    

/**
   A child link is removed from the link.
   If a link given by the parameter is not a child of the link, false is returned.
*/
inline bool Link::removeChild(Link* childToRemove)
{
    Link* link = child_;
    Link* prevSibling = 0;
    while(link){
        if(link == childToRemove){
            childToRemove->parent_ = 0;
            childToRemove->sibling_ = 0;
            if(prevSibling){
                prevSibling->sibling_ = link->sibling_;
            } else {
                child_ = link->sibling_;
            }
            childToRemove->setBody(0);
            return true;
        }
        prevSibling = link;
        link = link->sibling_;
    }
    return false;
}


inline void Link::setName(const std::string& name)
{
    name_ = name;
}


inline std::string Link::jointTypeString() const
{
    switch(jointType_){
    case REVOLUTE_JOINT:    return "revolute";
    case PRISMATIC_JOINT:   return "prismatic";
    case FREE_JOINT:        return "free";
    case FIXED_JOINT:       return "fixed";
    case PSEUDO_CONTINUOUS_TRACK: return "pseudo continuous track";
    default: return "unknown";
    }
}


inline std::string Link::actuationModeString() const
{
    switch(actuationMode_){

    case NO_ACTUATION:
        return "no actuation";

    case JOINT_EFFORT:
        if(isRevoluteJoint()){
            return "joint torque";
        } else if(isPrismaticJoint()){
            return "joint force";
        } else {
            return "joint effort";
        }

    case JOINT_DISPLACEMENT:
        if(isRevoluteJoint()){
            return "joint angle";
        } else {
            return "joint displacement";
        }

    case JOINT_VELOCITY:
        return "joint velocity";

    case JOINT_SURFACE_VELOCITY:
        return "joint surface velocity";

    case LINK_POSITION:
        return "link position";

    default:
        return "unknown";
    }
}

inline std::string Link::materialName() const
{
    return Material::name(materialId_);
}

inline void Link::setMaterial(const std::string& name)
{
    setMaterial(Material::id(name));
}


inline void Link::setShape(SgNode* shape)
{
    visualShape_ = shape;
    collisionShape_ = shape;
}

inline void Link::setVisualShape(SgNode* shape)
{
    visualShape_ = shape;
}


inline void Link::setCollisionShape(SgNode* shape)
{
    collisionShape_ = shape;
}

inline void Link::resetInfo(Mapping* info)
{
    info_ = info;
}


template<> inline double Link::info(const std::string& key) const
{
    return info_->get(key).toDouble();
}


template<> inline double Link::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> inline bool Link::info(const std::string& key, const bool& defaultValue) const
{
    bool value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> inline void Link::setInfo(const std::string& key, const double& value)
{
    info_->write(key, value);
}


template<> inline void Link::setInfo(const std::string& key, const bool& value)
{
    info_->write(key, value);
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_LINK_CPP_H
