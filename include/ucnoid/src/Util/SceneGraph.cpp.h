/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_SCENE_GRAPH_CPP_H
#define UCNOID_UTIL_SCENE_GRAPH_CPP_H

#include "SceneGraph.h"
#include "Exception.h"
#include <unordered_map>
#include <typeindex>
#include <mutex>

namespace cnoid {
inline namespace ucnoid {

inline SgUpdate::~SgUpdate()
{

}

namespace detail::scene_graph {
typedef std::unordered_map<const SgObject*, SgObjectPtr> CloneMap;
}

class SgCloneMapImpl : public detail::scene_graph::CloneMap { };


inline SgCloneMap::SgCloneMap()
{
    cloneMap = new SgCloneMapImpl;
    isNonNodeCloningEnabled_ = true;
}


inline SgCloneMap::SgCloneMap(const SgCloneMap& org)
{
    cloneMap = new SgCloneMapImpl(*org.cloneMap);
    isNonNodeCloningEnabled_ = org.isNonNodeCloningEnabled_;
}


inline void SgCloneMap::clear()
{
    cloneMap->clear();
}


inline SgObject* SgCloneMap::findOrCreateClone(const SgObject* org)
{
    detail::scene_graph::CloneMap::iterator p = cloneMap->find(org);
    if(p == cloneMap->end()){
        SgObject* clone = org->clone(*this);
        (*cloneMap)[org] = clone;
        return clone;
    } else {
        return p->second.get();
    }
}


inline SgCloneMap::~SgCloneMap()
{
    delete cloneMap;
}


inline SgObject::SgObject()
{

}


inline SgObject::SgObject(const SgObject& org)
    : name_(org.name_)
{

}


inline SgObject* SgObject::clone(SgCloneMap&) const
{
    return new SgObject(*this);
}


inline int SgObject::numChildObjects() const
{
    return 0;
}


inline SgObject* SgObject::childObject(int /* index */)
{
    return 0;
}


inline void SgObject::onUpdated(SgUpdate& update)
{
    update.push(this);
    sigUpdated_(update);
    for(const_parentIter p = parents.begin(); p != parents.end(); ++p){
        (*p)->onUpdated(update);
    }
    update.pop();
}


inline void SgObject::addParent(SgObject* parent, bool doNotify)
{
    parents.insert(parent);
    if(doNotify){
        SgUpdate update(SgUpdate::ADDED);
        update.push(this);
        parent->onUpdated(update);
    }
    if(parents.size() == 1){
        sigGraphConnection_(true);
    }
}


inline void SgObject::removeParent(SgObject* parent)
{
    parents.erase(parent);
    if(parents.empty()){
        sigGraphConnection_(false);
    }
}


namespace detail::scene_graph {
inline std::mutex polymorphicIdMutex;
typedef std::unordered_map<std::type_index, int> PolymorphicIdMap;
inline PolymorphicIdMap polymorphicIdMap;
inline std::vector<int> superTypePolymorphicIdMap;
}

inline int SgNode::registerNodeType(const std::type_info& nodeType, const std::type_info& superType)
{
    using namespace detail::scene_graph;
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);

    int superTypeId;
    PolymorphicIdMap::iterator iter = polymorphicIdMap.find(superType);
    if(iter == polymorphicIdMap.end()){
        superTypeId = polymorphicIdMap.size();
        polymorphicIdMap[superType] = superTypeId;
    } else {
        superTypeId = iter->second;
    }
    int id;
    if(nodeType == superType){
        id = superTypeId;
    } else {
        id = polymorphicIdMap.size();
        polymorphicIdMap[nodeType] = id;
        if(id >= static_cast<int>(superTypePolymorphicIdMap.size())){
            superTypePolymorphicIdMap.resize(id + 1, -1);
        }
        superTypePolymorphicIdMap[id] = superTypeId;
    }

    return id;
}

inline int SgNode::findPolymorphicId(const std::type_info& nodeType)
{
    using namespace detail::scene_graph;
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);

    auto iter = polymorphicIdMap.find(nodeType);
    if(iter != polymorphicIdMap.end()){
        return iter->second;
    }
    return -1;
}


inline int SgNode::findSuperTypePolymorphicId(int polymorhicId)
{
    using namespace detail::scene_graph;
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);
    return superTypePolymorphicIdMap[polymorhicId];
}


inline int SgNode::numPolymorphicTypes()
{
    using namespace detail::scene_graph;
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);
    return polymorphicIdMap.size();
}


inline SgNode::SgNode()
{
    polymorhicId_ = findPolymorphicId<SgNode>();
}


inline SgNode::SgNode(int polymorhicId)
    : polymorhicId_(polymorhicId)
{

}


inline SgNode::SgNode(const SgNode& org)
    : SgObject(org),
      polymorhicId_(org.polymorhicId_)
{

}


inline SgNode::~SgNode()
{

}


inline SgObject* SgNode::clone(SgCloneMap&) const
{
    return new SgNode(*this);
}


inline const BoundingBox& SgNode::boundingBox() const
{
    static const BoundingBox bbox; // empty one
    return bbox;
}


inline bool SgNode::isGroup() const
{
    return false;
}

namespace detail::scene_graph {
/**
   \note The current implementation of this function does not seem to return the correct T value
*/
inline bool findNodeSub(SgNode* node, const std::string& name, SgNodePath& path, Affine3 T, Affine3& out_T)
{
    path.push_back(node);

    if(auto group = dynamic_cast<SgGroup*>(node)){
        if(auto transform = dynamic_cast<SgTransform*>(group)){
            Affine3 T0;
            transform->getTransform(T0);
            T = T * T0;
        }
        if(node->name() == name){
            out_T = T;
            return true;
        }
        for(auto& child : *group){
            if(findNodeSub(child, name, path, T, out_T)){
                return true;
            }
        }
    } else {
        if(node->name() == name){
            out_T = T;
            return true;
        }
    }
    
    path.pop_back();

    return false;
}

}   // namespace detail::scene_graph

inline SgNodePath SgNode::findNode(const std::string& name, Affine3& out_T)
{
    SgNodePath path;
    out_T.setIdentity();
    detail::scene_graph::findNodeSub(this, name, path, out_T, out_T);
    return path;
}


inline SgGroup::SgGroup()
    : SgNode(findPolymorphicId<SgGroup>())
{
    isBboxCacheValid = false;
}


inline SgGroup::SgGroup(int polymorhicId)
    : SgNode(polymorhicId)
{
    isBboxCacheValid = false;
}


inline SgGroup::SgGroup(const SgGroup& org)
    : SgNode(org)
{
    children.reserve(org.numChildren());

    // shallow copy
    for(const_iterator p = org.begin(); p != org.end(); ++p){
        addChild(*p, false);
    }

    isBboxCacheValid = true;
    bboxCache = org.bboxCache;
}


inline SgGroup::SgGroup(const SgGroup& org, SgCloneMap& cloneMap)
    : SgNode(org)
{
    children.reserve(org.numChildren());

    for(const_iterator p = org.begin(); p != org.end(); ++p){
        addChild(cloneMap.getClone<SgNode>(p->get()), false);
    }

    isBboxCacheValid = true;
    bboxCache = org.bboxCache;
}


inline SgGroup::~SgGroup()
{
    for(const_iterator p = begin(); p != end(); ++p){
        (*p)->removeParent(this);
    }
}


inline SgObject* SgGroup::clone(SgCloneMap& cloneMap) const
{
    return new SgGroup(*this, cloneMap);
}


inline int SgGroup::numChildObjects() const
{
    return children.size();
}


inline SgObject* SgGroup::childObject(int index)
{
    return children[index].get();
}


inline void SgGroup::onUpdated(SgUpdate& update)
{
    //if(update.action() & SgUpdate::BBOX_UPDATED){
    invalidateBoundingBox();
    SgNode::onUpdated(update);
    //}
}


inline const BoundingBox& SgGroup::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    isBboxCacheValid = true;

    return bboxCache;
}


inline bool SgGroup::isGroup() const
{
    return true;
}


inline bool SgGroup::contains(SgNode* node) const
{
    for(const_iterator p = begin(); p != end(); ++p){
        if((*p) == node){
            return true;
        }
    }
    return false;
}


inline void SgGroup::addChild(SgNode* node, bool doNotify)
{
    if(node){
        children.push_back(node);
        node->addParent(this, doNotify);
    }
}


inline bool SgGroup::addChildOnce(SgNode* node, bool doNotify)
{
    if(!contains(node)){
        addChild(node, doNotify);
        return true;
    }
    return false;
}


inline void SgGroup::insertChild(SgNode* node, int index, bool doNotify)
{
    if(node){
        if(index > static_cast<int>(children.size())){
            index = children.size();
        }
        children.insert(children.begin() + index, node);
        node->addParent(this, doNotify);
    }
}


inline SgGroup::iterator SgGroup::removeChild(iterator childIter, bool doNotify)
{
    iterator next;
    SgNode* child = *childIter;
    child->removeParent(this);
    
    if(!doNotify){
        next = children.erase(childIter);
    } else {
        SgNodePtr childHolder = child;
        next = children.erase(childIter);
        SgUpdate update(SgUpdate::REMOVED);
        update.push(child);
        onUpdated(update);
    }
    return next;
}


inline bool SgGroup::removeChild(SgNode* node, bool doNotify)
{
    bool removed = false;
    if(node){
        iterator p = children.begin();
        while(p != children.end()){
            if((*p) == node){
                p = removeChild(p, doNotify);
                removed = true;
            } else {
                ++p;
            }
        }
    }
    return removed;
}


inline void SgGroup::removeChildAt(int index, bool doNotify)
{
    removeChild(children.begin() + index, doNotify);
}


inline void SgGroup::clearChildren(bool doNotify)
{
    iterator p = children.begin();
    while(p != children.end()){
        p = removeChild(p, doNotify);
    }
}


inline void SgGroup::copyChildrenTo(SgGroup* group, bool doNotify)
{
    for(size_t i=0; i < children.size(); ++i){
        group->addChild(child(i), doNotify);
    }
}


inline void SgGroup::moveChildrenTo(SgGroup* group, bool doNotify)
{
    const int destTop = group->children.size();
    
    for(size_t i=0; i < children.size(); ++i){
        group->addChild(child(i));
    }
    clearChildren(doNotify);
    if(doNotify){
        SgUpdate update(SgUpdate::ADDED);
        for(int i=destTop; i < group->numChildren(); ++i){
            group->child(i)->notifyUpdate(update);
        }
    }
}


inline void SgGroup::throwTypeMismatchError()
{
    throw type_mismatch_error();
}


inline SgInvariantGroup::SgInvariantGroup()
    : SgGroup(findPolymorphicId<SgInvariantGroup>())
{

}


inline SgInvariantGroup::SgInvariantGroup(const SgInvariantGroup& org)
    : SgGroup(org)
{

}


inline SgInvariantGroup::SgInvariantGroup(const SgInvariantGroup& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{

}


inline SgObject* SgInvariantGroup::clone(SgCloneMap& cloneMap) const
{
    return new SgInvariantGroup(*this, cloneMap);
}


inline SgTransform::SgTransform(int polymorhicId)
    : SgGroup(polymorhicId)
{

}


inline SgTransform::SgTransform(const SgTransform& org)
    : SgGroup(org)
{
    untransformedBboxCache = org.untransformedBboxCache;
}
    

inline SgTransform::SgTransform(const SgTransform& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{
    untransformedBboxCache = org.untransformedBboxCache;
}


inline const BoundingBox& SgTransform::untransformedBoundingBox() const
{
    if(!isBboxCacheValid){
        boundingBox();
    }
    return untransformedBboxCache;
}


inline SgPosTransform::SgPosTransform(int polymorhicId)
    : SgTransform(polymorhicId),
      T_(Affine3::Identity())
{

}


inline SgPosTransform::SgPosTransform()
    : SgPosTransform(findPolymorphicId<SgPosTransform>())
{

}


inline SgPosTransform::SgPosTransform(const Affine3& T)
    : SgTransform(findPolymorphicId<SgPosTransform>()),
      T_(T)
{

}


inline SgPosTransform::SgPosTransform(const SgPosTransform& org)
    : SgTransform(org),
      T_(org.T_)
{

}


inline SgPosTransform::SgPosTransform(const SgPosTransform& org, SgCloneMap& cloneMap)
    : SgTransform(org, cloneMap),
      T_(org.T_)
{

}


inline SgObject* SgPosTransform::clone(SgCloneMap& cloneMap) const
{
    return new SgPosTransform(*this, cloneMap);
}


inline const BoundingBox& SgPosTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(T_);
    isBboxCacheValid = true;
    return bboxCache;
}


inline void SgPosTransform::getTransform(Affine3& out_T) const
{
    out_T = T_;
}


inline SgScaleTransform::SgScaleTransform(int polymorhicId)
    : SgTransform(polymorhicId)
{
    scale_.setOnes();
}


inline SgScaleTransform::SgScaleTransform()
    : SgScaleTransform(findPolymorphicId<SgScaleTransform>())
{

}


inline SgScaleTransform::SgScaleTransform(const Vector3& scale)
    : SgTransform(findPolymorphicId<SgScaleTransform>()),
      scale_(scale)
{

}


inline SgScaleTransform::SgScaleTransform(const SgScaleTransform& org)
    : SgTransform(org),
      scale_(org.scale_)
{

}
      
    
inline SgScaleTransform::SgScaleTransform(const SgScaleTransform& org, SgCloneMap& cloneMap)
    : SgTransform(org, cloneMap),
      scale_(org.scale_)
{

}


inline SgAffineTransform::SgAffineTransform(int polymorhicId)
    : SgTransform(polymorhicId),
      T_(Affine3::Identity())
{

}


inline SgAffineTransform::SgAffineTransform()
    : SgAffineTransform(findPolymorphicId<SgAffineTransform>())
{

}


inline SgAffineTransform::SgAffineTransform(const Affine3& T)
    : SgTransform(findPolymorphicId<SgAffineTransform>()),
      T_(T)
{

}


inline SgAffineTransform::SgAffineTransform(const SgAffineTransform& org)
    : SgTransform(org),
      T_(org.T_)
{

}


inline SgAffineTransform::SgAffineTransform(const SgAffineTransform& org, SgCloneMap& cloneMap)
    : SgTransform(org, cloneMap),
      T_(org.T_)
{

}


inline SgObject* SgAffineTransform::clone(SgCloneMap& cloneMap) const
{
    return new SgAffineTransform(*this, cloneMap);
}


inline const BoundingBox& SgAffineTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(T_);
    isBboxCacheValid = true;
    return bboxCache;
}


inline void SgAffineTransform::getTransform(Affine3& out_T) const
{
    out_T = T_;
}


inline SgObject* SgScaleTransform::clone(SgCloneMap& cloneMap) const
{
    return new SgScaleTransform(*this, cloneMap);
}


inline const BoundingBox& SgScaleTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(Affine3(scale_.asDiagonal()));
    isBboxCacheValid = true;
    return bboxCache;
}


inline void SgScaleTransform::getTransform(Affine3& out_T) const
{
    out_T = scale_.asDiagonal();
}


inline SgSwitch::SgSwitch()
    : SgGroup(findPolymorphicId<SgSwitch>())
{
    isTurnedOn_ = true;
}


inline SgSwitch::SgSwitch(const SgSwitch& org)
    : SgGroup(org)
{
    isTurnedOn_ = org.isTurnedOn_;
}


inline SgSwitch::SgSwitch(const SgSwitch& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{
    isTurnedOn_ = org.isTurnedOn_;
}


inline SgObject* SgSwitch::clone(SgCloneMap& cloneMap) const
{
    return new SgSwitch(*this, cloneMap);
}


inline void SgSwitch::setTurnedOn(bool on, bool doNotify)
{
    isTurnedOn_ = on;
    if(doNotify){
        notifyUpdate();
    }
}


inline SgUnpickableGroup::SgUnpickableGroup()
    : SgGroup(findPolymorphicId<SgUnpickableGroup>())
{

}


inline SgUnpickableGroup::SgUnpickableGroup(const SgUnpickableGroup& org)
    : SgGroup(org)
{

}


inline SgUnpickableGroup::SgUnpickableGroup(const SgUnpickableGroup& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{

}


inline SgObject* SgUnpickableGroup::clone(SgCloneMap& cloneMap) const
{
    return new SgUnpickableGroup(*this, cloneMap);
}


inline SgPreprocessed::SgPreprocessed(int polymorhicId)
    : SgNode(polymorhicId)
{

}


inline SgPreprocessed::SgPreprocessed(const SgPreprocessed& org)
    : SgNode(org)
{

}


inline SgObject* SgPreprocessed::clone(SgCloneMap&) const
{
    return new SgPreprocessed(*this);
}

namespace detail::scene_graph {

inline struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgNode, SgNode>();
        SgNode::registerType<SgGroup, SgNode>();
        SgNode::registerType<SgInvariantGroup, SgGroup>();
        SgNode::registerType<SgTransform, SgGroup>();
        SgNode::registerType<SgAffineTransform, SgTransform>();
        SgNode::registerType<SgPosTransform, SgTransform>();
        SgNode::registerType<SgScaleTransform, SgTransform>();
        SgNode::registerType<SgSwitch, SgGroup>();
        SgNode::registerType<SgUnpickableGroup, SgGroup>();
        SgNode::registerType<SgPreprocessed, SgNode>();
    }
} registration;

}   // detail::scene_graph

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_SCENE_GRAPH_CPP_H
