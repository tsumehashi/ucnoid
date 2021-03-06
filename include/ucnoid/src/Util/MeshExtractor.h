/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_MESH_EXTRACTOR_H
#define UCNOID_UTIL_MESH_EXTRACTOR_H

#include "EigenTypes.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class SgNode;
class SgMesh;
class SgShape;
class MeshExtractorImpl;

class UCNOID_EXPORT MeshExtractor
{
public:
    MeshExtractor();
    bool extract(SgNode* node, std::function<void()> callback);
    bool extract(SgNode* node, std::function<void(SgMesh* mesh)> callback);
    SgMesh* integrate(SgNode* node);

    SgMesh* currentMesh() const;
    SgShape* currentShape() const;
    const Affine3& currentTransform() const;
    const Affine3& currentTransformWithoutScaling() const;
    bool isCurrentScaled() const;

private:
    MeshExtractorImpl* impl;
};

}   // inline namespace ucnoid
}

#include "MeshExtractor.cpp.h"

#endif
