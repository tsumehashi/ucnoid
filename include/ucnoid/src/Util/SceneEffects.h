/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_SCENE_EFFECTS_H
#define UCNOID_UTIL_SCENE_EFFECTS_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class UCNOID_EXPORT SgFog : public SgPreprocessed
{
protected:
    SgFog(int polymorhicId);
    
public:
    SgFog();
    SgFog(const SgFog& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    const Vector3f& color() const { return color_; }
    template<typename Derived> void setColor(const Eigen::MatrixBase<Derived>& c) {
        color_ = c.template cast<Vector3f::Scalar>(); }
    void setVisibilityRange(float r) { visibilityRange_ = r; }
    float visibilityRange() const { return visibilityRange_; }
    
private:
    Vector3f color_;
    float  visibilityRange_;
    //int fogType;
};
typedef ref_ptr<SgFog> SgFogPtr;


class UCNOID_EXPORT SgOutlineGroup : public SgGroup
{
protected:
    SgOutlineGroup(int polymorhicId);
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgOutlineGroup();

    const Vector3f& color() const { return color_; }
    void setColor(const Vector3f& color) { color_ = color; }
    void setLineWidth(float width) { lineWidth_ = width; }
    float lineWidth() const { return lineWidth_; }

private:
    Vector3f color_;
    float lineWidth_;
};
typedef ref_ptr<SgOutlineGroup> SgOutlineGroupPtr;

}   // inline namespace ucnoid
}

#include "SceneEffects.cpp.h"

#endif
