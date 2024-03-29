/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_MESH_EXTRACTOR_CPP_H
#define UCNOID_UTIL_MESH_EXTRACTOR_CPP_H

#include "MeshExtractor.h"
#include "SceneDrawables.h"
#include "PolymorphicFunctionSet.h"

namespace cnoid {
inline namespace ucnoid {

class MeshExtractorImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PolymorphicFunctionSet<SgNode> functions;
    std::function<void()> callback1;
    std::function<void(SgMesh* mesh)> callback2;
    SgMesh* currentMesh;
    SgShape* currentShape;
    Affine3 currentTransform;
    Affine3 currentTransformWithoutScaling;
    bool isCurrentScaled;
    bool meshFound;
    
    MeshExtractorImpl();
    void visitGroup(SgGroup* group);
    void visitSwitch(SgSwitch* switchNode);    
    void visitTransform(SgTransform* transform);
    void visitPosTransform(SgPosTransform* transform);
    void visitShape(SgShape* shape);
    bool extract(SgNode* node);
};

inline MeshExtractor::MeshExtractor()
{
    impl = new MeshExtractorImpl;
}


inline MeshExtractorImpl::MeshExtractorImpl()
{
    functions.setFunction<SgGroup>(
        [&](SgGroup* node){ visitGroup(node); });
    functions.setFunction<SgSwitch>(
        [&](SgSwitch* node){ visitSwitch(node); });
    functions.setFunction<SgTransform>(
        [&](SgTransform* node){ visitTransform(node); });
    functions.setFunction<SgPosTransform>(
        [&](SgPosTransform* node){ visitPosTransform(node); });
    functions.setFunction<SgShape>(
        [&](SgShape* node){ visitShape(node); });
    functions.updateDispatchTable();
}
    
    
inline void MeshExtractorImpl::visitGroup(SgGroup* group)
{
    for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
        functions.dispatch(*p);
    }
}


inline void MeshExtractorImpl::visitSwitch(SgSwitch* switchNode)
{
    if(switchNode->isTurnedOn()){
        visitGroup(switchNode);
    }
}
    

inline void MeshExtractorImpl::visitTransform(SgTransform* transform)
{
    bool isParentScaled = isCurrentScaled;
    isCurrentScaled = true;
    Affine3 T0 = currentTransform;
    Affine3 T;
    transform->getTransform(T);
    currentTransform = T0 * T;
    visitGroup(transform);
    currentTransform = T0;
    isCurrentScaled = isParentScaled;
}


inline void MeshExtractorImpl::visitPosTransform(SgPosTransform* transform)
{
    const Affine3 T0(currentTransform);
    const Affine3 P0(currentTransformWithoutScaling);
    currentTransform = T0 * transform->T();
    currentTransformWithoutScaling = P0 * transform->T();
    visitGroup(transform);
    currentTransform = T0;
    currentTransformWithoutScaling = P0;
}


inline void MeshExtractorImpl::visitShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->vertices() && !mesh->vertices()->empty() && !mesh->triangleVertices().empty()){
        meshFound = true;
        currentMesh = mesh;
        currentShape = shape;
        if(callback1){
            callback1();
        } else {
            callback2(mesh);
        }
        currentMesh = 0;
        currentShape = 0;
    }
}


inline SgMesh* MeshExtractor::currentMesh() const
{
    return impl->currentMesh;
}


inline SgShape* MeshExtractor::currentShape() const
{
    return impl->currentShape;
}


inline const Affine3& MeshExtractor::currentTransform() const
{
    return impl->currentTransform;
}


inline const Affine3& MeshExtractor::currentTransformWithoutScaling() const
{
    return impl->currentTransformWithoutScaling;
}


inline bool MeshExtractor::isCurrentScaled() const
{
    return impl->isCurrentScaled;
}


inline bool MeshExtractor::extract(SgNode* node, std::function<void()> callback)
{
    impl->callback1 = callback;
    impl->callback2 = nullptr;
    return impl->extract(node);
}


inline bool MeshExtractor::extract(SgNode* node, std::function<void(SgMesh* mesh)> callback)
{
    impl->callback1 = nullptr;
    impl->callback2 = callback;
    return impl->extract(node);
}


inline bool MeshExtractorImpl::extract(SgNode* node)
{
    currentMesh = 0;
    currentShape = 0;
    currentTransform.setIdentity();
    currentTransformWithoutScaling.setIdentity();
    isCurrentScaled = false;
    meshFound = false;
    functions.dispatch(node);
    return meshFound;
}

namespace detail::mesh_extractor {

inline void integrateMesh(MeshExtractor* extractor, SgMesh* mesh)
{
    SgMesh* srcMesh = extractor->currentMesh();
    const Affine3f T = extractor->currentTransform().cast<Affine3f::Scalar>();

    if(srcMesh->hasVertices()){
        SgVertexArray& vertices = *mesh->getOrCreateVertices();
        const int numVertices = vertices.size();
        SgVertexArray& srcVertices = *srcMesh->vertices();
        const int numSrcVertices = srcVertices.size();
        vertices.reserve(numVertices + numSrcVertices);
        for(int i=0; i < numSrcVertices; ++i){
            vertices.push_back(T * srcVertices[i]);
        }

        SgIndexArray& indices = mesh->triangleVertices();
        const int numIndices = indices.size();
        SgIndexArray& srcIndices = srcMesh->triangleVertices();
        const int numSrcIndices = srcIndices.size();
        indices.reserve(numIndices + numSrcIndices);
        for(int i=0; i < numSrcIndices; ++i){
            indices.push_back(srcIndices[i] + numVertices);
        }
        
        if(srcMesh->hasNormals()){
            SgNormalArray& normals = *mesh->getOrCreateNormals();
            const int numNormals = normals.size();
            SgNormalArray& srcNormals = *srcMesh->normals();
            const int numSrcNormals = srcNormals.size();
            normals.reserve(numNormals + numSrcNormals);
            const Affine3f U = extractor->currentTransformWithoutScaling().cast<Affine3f::Scalar>();
            for(int i=0; i < numSrcNormals; ++i){
                normals.push_back(U * srcNormals[i]);
            }
            
            SgIndexArray& indices = mesh->normalIndices();
            const int numIndices = indices.size();
            SgIndexArray& srcIndices = srcMesh->normalIndices();
            const int numSrcIndices = srcIndices.size();
            indices.reserve(numIndices + numSrcIndices);
            for(int i=0; i < numSrcIndices; ++i){
                indices.push_back(srcIndices[i] + numNormals);
            }
        }
    }
}

}   // namespace detail::mesh_extractor

/**
   \todo take into acount the case where some meshes have normals or colors
   and others don't have them.
*/
inline SgMesh* MeshExtractor::integrate(SgNode* node)
{
    SgMesh* integrated = new SgMesh;
    extract(node, [&](){ detail::mesh_extractor::integrateMesh(this, integrated); });
    return integrated;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_MESH_EXTRACTOR_CPP_H
