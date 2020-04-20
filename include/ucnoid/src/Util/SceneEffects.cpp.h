/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_SCENE_EFFECTS_CPP_H
#define UCNOID_UTIL_SCENE_EFFECTS_CPP_H

#include "SceneEffects.h"

namespace cnoid {
inline namespace ucnoid {

SgFog::SgFog(int polymorhicId)
    : SgPreprocessed(polymorhicId)
{
    color_.setOnes();
    visibilityRange_ = 0.0f;
}


SgFog::SgFog()
    : SgFog(findPolymorphicId<SgFog>())
{

}


SgFog::SgFog(const SgFog& org)
    : SgPreprocessed(org)
{
    color_ = org.color_;
    visibilityRange_ = org.visibilityRange_;
}


SgObject* SgFog::clone(SgCloneMap&) const
{
    return new SgFog(*this);
}


SgOutlineGroup::SgOutlineGroup(int polymorhicId)
    : SgGroup(polymorhicId)
{
    lineWidth_ = 1.0;
    color_ << 1.0, 0.0, 0.0;
}


SgOutlineGroup::SgOutlineGroup()
    : SgOutlineGroup(findPolymorphicId<SgOutlineGroup>())
{

}

namespace detail::scene_effects {

inline struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgFog, SgPreprocessed>();
        SgNode::registerType<SgOutlineGroup, SgGroup>();
    }
} registration;

}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_SCENE_EFFECTS_CPP_H
