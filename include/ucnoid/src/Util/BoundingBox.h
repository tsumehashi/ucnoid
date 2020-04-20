/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_BOUNDING_BOX_H
#define UCNOID_UTIL_BOUNDING_BOX_H

#include "EigenTypes.h"
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class BoundingBoxf;

class UCNOID_EXPORT BoundingBox
{
public:
    BoundingBox();
    BoundingBox(const Vector3& min, const Vector3& max);
    BoundingBox(const BoundingBox& org);
    BoundingBox(const BoundingBoxf& org);

    void set(const Vector3& min, const Vector3& max);
    void clear();

    bool empty() const { return empty_; }
    const Vector3& min() const { return min_; }
    const Vector3& max() const { return max_; }
    Vector3 center() const;
    Vector3 size() const;
    double boundingSphereRadius() const;
        
    void expandBy(const BoundingBox& bbox);
    void expandBy(double x, double y, double z);
    void expandBy(const Vector3& v){ expandBy(v.x(), v.y(), v.z()); }

    void transform(const Affine3& T);

private:
    Vector3 min_;
    Vector3 max_;
    bool empty_;
};

UCNOID_EXPORT std::ostream& operator<<(std::ostream& os, const BoundingBox& bb);

/**
   float type version of the BoundingBox class
*/
class UCNOID_EXPORT BoundingBoxf
{
public:
    BoundingBoxf();
    BoundingBoxf(const Vector3f& min, const Vector3f& max);
    BoundingBoxf(const BoundingBoxf& org);
    BoundingBoxf(const BoundingBox& org);
        
    void set(const Vector3f& min, const Vector3f& max);
    void clear();

    bool empty() const { return empty_; }
    const Vector3f& min() const { return min_; }
    const Vector3f& max() const { return max_; }
    Vector3f center() const;
    float boundingSphereRadius() const;
        
    void expandBy(const BoundingBoxf& bbox);
    void expandBy(float x, float y, float z);
    void expandBy(const Vector3f& v){ expandBy(v.x(), v.y(), v.z()); }

    void transform(const Affine3f& T);

private:
    bool empty_;
    Vector3f min_;
    Vector3f max_;
};

UCNOID_EXPORT std::ostream& operator<<(std::ostream& os, const BoundingBoxf& bb);

}   // inline namespace ucnoid
}    

#include "BoundingBox.cpp.h"

#endif
