/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_YAML_BODY_LOADER_CPP_H
#define UCNOID_BODY_YAML_BODY_LOADER_CPP_H

#include "YAMLBodyLoader.h"
#include "BodyLoader.h"
#include "Body.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include "Camera.h"
#include "RangeCamera.h"
#include "RangeSensor.h"
#include "PointLight.h"
#include "SpotLight.h"
#include <ucnoid/YAMLSceneReader>
#include <ucnoid/EigenArchive>
#include <ucnoid/FileUtil>
#include <ucnoid/Exception>
#include <ucnoid/YAMLReader>
#include <ucnoid/NullOut>
#include <Eigen/StdVector>
#if UCNOID_NOT_SUPPORTED
#include <fmt/format.h>
#endif   // UCNOID_NOT_SUPPORTED
#include <optional>
#include <unordered_map>
#if UCNOID_NOT_SUPPORTED
#include <mutex>
#endif   // UCNOID_NOT_SUPPORTED
#include <cstdlib>
#include "gettext.h"
#include <iostream>

namespace cnoid {
inline namespace ucnoid {
namespace detail::yaml_body_loader {

#if UCNOID_NOT_SUPPORTED
std::mutex customNodeFunctionMutex;
#endif   // UCNOID_NOT_SUPPORTED
typedef std::function<bool(YAMLBodyLoader& loader, Mapping& node)> CustomNodeFunction;
typedef std::map<std::string, CustomNodeFunction> CustomNodeFunctionMap;
static inline CustomNodeFunctionMap customNodeFunctions;

class ROSPackageSchemeHandler
{
    std::vector<std::string> packagePaths;
    
public:
    ROSPackageSchemeHandler()
    {
        const char* str = getenv("ROS_PACKAGE_PATH");
        if(str){
            do {
                const char* begin = str;
                while(*str != ':' && *str) str++;
                packagePaths.push_back(std::string(begin, str));
            } while (0 != *str++);
        }
    }

    std::string operator()(const std::string& path, std::ostream& os)
    {
        std::filesystem::path filepath(path);
        auto iter = filepath.begin();
        if(iter == filepath.end()){
            return std::string();
        }
        
        std::filesystem::path directory = *iter++;
        std::filesystem::path relativePath;
        while(iter != filepath.end()){
            relativePath /= *iter++;
        }

        bool found = false;
        std::filesystem::path combined;
        
        for(auto element : packagePaths){
            std::filesystem::path packagePath(element);
            combined = packagePath / filepath;
            if(std::filesystem::exists(combined)){
                found = true;
                break;
            }
            combined = packagePath / relativePath;
            if(std::filesystem::exists(combined)){
                found = true;
                break;
            }
        }

        if(found){
            return combined.string();
        } else {
#if UCNOID_NOT_SUPPORTED
            os << format(_("\"{}\" is not found in the ROS package directories."), path) << endl;
#else   // UCNOID_NOT_SUPPORTED
            os << ssformat("\"", path, "\" is not found in the ROS package directories.") << std::endl;
#endif  // UCNOID_NOT_SUPPORTED
            return std::string();
        }
    }
};

#if UCNOID_NOT_SUPPORTED
struct ROSPackageSchemeHandlerRegistration {
    ROSPackageSchemeHandlerRegistration(){
        YAMLSceneReader::registerUriSchemeHandler("package", ROSPackageSchemeHandler());
    }
} rosPackageSchemeHandlerRegistration;
#endif  // UCNOID_NOT_SUPPORTED

}   // namespace detail::yaml_body_loader

inline void YAMLBodyLoader::addNodeType
(const std::string& typeName, std::function<bool(YAMLBodyLoader& loader, Mapping& node)> readFunction)
{
#if UCNOID_NOT_SUPPORTED
    std::lock_guard<std::mutex> guard(customNodeFunctionMutex);
#endif  // UCNOID_NOT_SUPPORTED
    detail::yaml_body_loader::customNodeFunctions[typeName] = readFunction;
}

class YAMLBodyLoaderImpl
{
public:
    YAMLBodyLoader* self;

    std::unique_ptr<BodyLoader> bodyLoader;
    
    YAMLReader reader;
    YAMLSceneReader sceneReader;
    std::filesystem::path mainFilePath;

    typedef std::function<bool(Mapping& node)> NodeFunction;

    struct NodeFunctionInfo {
        NodeFunction function;
        bool isTransformDerived;
        bool hasElements;
        void set(NodeFunction f) { function = f; isTransformDerived = false; hasElements = false; }
        void setT(NodeFunction f) { function = f; isTransformDerived = true; hasElements = false; }
        void setTE(NodeFunction f) { function = f; isTransformDerived = true; hasElements = true; }
    };
    typedef std::unordered_map<std::string, NodeFunctionInfo> NodeFunctionMap;
    
    NodeFunctionMap nodeFunctions;
    size_t numCustomNodeFunctions;
    
    Body* body;
    Link* rootLink;

    struct LinkInfo : public Referenced
    {
        LinkPtr link;
        MappingPtr node;
        std::string parent;
    };
    typedef ref_ptr<LinkInfo> LinkInfoPtr;
    
    std::vector<LinkInfoPtr> linkInfos;

    typedef std::map<std::string, LinkPtr> LinkMap;
    LinkMap linkMap;

    LinkPtr currentLink;
    std::vector<std::string> nameStack;
    typedef std::vector<Affine3, Eigen::aligned_allocator<Affine3>> Affine3Vector;
    Affine3Vector transformStack;

    struct RigidBody
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Vector3 c;
        double m;
        Matrix3 I;
    };
    typedef std::vector<RigidBody, Eigen::aligned_allocator<RigidBody>> RigidBodyVector;
    RigidBodyVector rigidBodies;

    struct SceneGroupSet
    {
        SgGroupPtr visual;
        SgGroupPtr collision;
        bool hasVisualChild;
        bool hasCollisionChild;

        SceneGroupSet(){
            hasVisualChild = false;
            hasCollisionChild = false;
        }
        template<class NodeType> void newGroup(){
            visual = new NodeType;
            collision = new NodeType;
        }
        template<class NodeType, class ParameterType> void newGroup(const ParameterType& param){
            visual = new NodeType(param);
            collision = new NodeType(param);
        }
        void setName(const std::string& name){
            visual->setName(name);
            collision->setName(name);
            // Group node that has name is kept even if it does not have contents
            hasVisualChild = true;
        }
    };

    std::vector<SceneGroupSet> sceneGroupSetStack;

    enum ModelType { VISUAL_AND_COLLISION, VISUAL, COLLISION } currentModelType;
    bool hasVisualOrCollisionNodes;

    SceneGroupSet& currentSceneGroupSet(){ return sceneGroupSetStack.back(); }
    
    void addScene(SgNode* node){
        auto& current = sceneGroupSetStack.back();
        switch(currentModelType){
        case VISUAL_AND_COLLISION:
            current.visual->addChild(node);
            current.collision->addChild(node);
            current.hasVisualChild = true;
            current.hasCollisionChild = true;
            break;
        case VISUAL:
            current.visual->addChild(node);
            current.hasVisualChild = true;
            break;
        case COLLISION:
            current.collision->addChild(node);
            current.hasCollisionChild = true;
            break;
        }
    }
    void addCurrentSceneGroupToParentSceneGroup(){
        auto& parent = sceneGroupSetStack[sceneGroupSetStack.size() - 2];
        auto& current = sceneGroupSetStack.back();
        switch(currentModelType){
        case VISUAL_AND_COLLISION:
            if(current.hasVisualChild){
                parent.visual->addChild(current.visual);
                parent.hasVisualChild = true;
            }
            if(current.hasCollisionChild){
                parent.collision->addChild(current.collision);
                parent.hasCollisionChild = true;
            }
            break;
        case VISUAL:
            if(current.hasVisualChild){
                parent.visual->addChild(current.visual);
                parent.hasVisualChild = true;
            }
            break;
        case COLLISION:
            if(current.hasCollisionChild){
                parent.collision->addChild(current.collision);
                parent.hasCollisionChild = true;
            }
            break;
        }
    }

    // temporary variables for reading values
    int id;
    double value;
    std::string symbol;
    bool on;
    Vector3f color;
    Vector3 v;
    Matrix3 M;

    std::vector<bool> validJointIdSet;
    size_t numValidJointIds;

    std::unique_ptr<YAMLBodyLoader> subLoader;
    std::map<std::string, BodyPtr> subBodyMap;
    std::vector<BodyPtr> subBodies;
    bool isSubLoader;

    std::ostream* os_;
    std::ostream& os() { return *os_; }
    int defaultDivisionNumber;
    double defaultCreaseAngle;
    bool isVerbose;
    bool isShapeLoadingEnabled;

    YAMLBodyLoaderImpl(YAMLBodyLoader* self);
    ~YAMLBodyLoaderImpl();
    void updateCustomNodeFunctions();
    bool clear();
    bool load(Body* body, const std::string& filename);
    bool readTopNode(Body* body, Mapping* topNode);
    bool checkFormat(Mapping* topNode);
    bool loadAnotherFormatBodyFile(Mapping* topNode);
    bool readBody(Mapping* topNode);
    void readNodeInLinks(Mapping* linkNode, const std::string& nodeType);
    void readLinkNode(Mapping* linkNode);
    void setLinkName(Link* link, const std::string& name, ValueNode* node);
    LinkPtr readLinkContents(Mapping* linkNode, LinkPtr link = nullptr);
    void setJointId(Link* link, int id);
    void readJointContents(Link* link, Mapping* node);
    bool extractAxis(Mapping* node, const char* key, Vector3& out_axis);
    bool readAxis(Mapping* node, const char* key, Vector3& out_axis);
    void readAxis(ValueNode* node, Vector3& out_axis);
    void setJointParameters(Link* link, std::string jointType, ValueNode* node);
    void setMassParameters(Link* link);

    /*
      The following functions return true if any scene nodes other than Group and Transform
      are added in the sub tree.
    */
    bool readElements(Mapping& node);
    bool readElementContents(ValueNode& elements);
    bool readNode(Mapping& node, const std::string& type);
    bool readSkipNode(Mapping& node);
    bool readContainerNode(Mapping& node, NodeFunction nodeFunction);
    bool readTransformContents(Mapping& node, NodeFunction nodeFunction, bool hasElements);
    bool readGroup(Mapping& node);
    bool readTransform(Mapping& node);
    bool readRigidBody(Mapping& node);
    bool readVisualOrCollision(Mapping& node, bool isVisual);
    bool readResource(Mapping& node);
    bool readDevice(Device* device, Mapping& node);
    bool readForceSensor(Mapping& node);
    bool readRateGyroSensor(Mapping& node);
    bool readAccelerationSensor(Mapping& node);
    bool readCamera(Mapping& node);
    bool readRangeSensor(Mapping& node);
    bool readSpotLight(Mapping& node);
    void readContinuousTrackNode(Mapping* linkNode);
    void addTrackLink(int index, LinkPtr link, Mapping* node, std::string& io_parent, double initialAngle);
    void readSubBodyNode(Mapping* linkNode);
    void addSubBodyLinks(BodyPtr subBody, Mapping* node);
    void readExtraJoints(Mapping* topNode);
    void readExtraJoint(Mapping* node);

    bool isDegreeMode() const {
        return sceneReader.isDegreeMode();
    }
    
    double toRadian(double angle) const {
        return sceneReader.toRadian(angle);
    }

    double readLimitValue(ValueNode& node, bool isUpper){
        double value;
        if(node.read(value)){
            return value;
        }
        std::string symbol;
        if(node.read(symbol)){
            if(symbol == "unlimited"){
                if(isUpper){
                    return std::numeric_limits<double>::max();
                } else {
                    return -std::numeric_limits<double>::max();
                }
            } else {
                node.throwException(_("Unknown symbol is used as a jointRange value"));
            }
        }
        node.throwException(_("Invalid type value is used as a jointRange value"));
        return 0.0;
    }

    bool readAngle(const Mapping& node, const char* key, double& angle) const {
        return sceneReader.readAngle(node, key, angle);
    }
    bool readRotation(const Mapping& node, Matrix3& out_R) const {
        return sceneReader.readRotation(node, out_R);
    }
    bool readRotation(const Mapping& node, const char* key, Matrix3& out_R) const {
        return sceneReader.readRotation(node, key, out_R);
    }
    bool extractRotation(Mapping& node, Matrix3& out_R) const {
        return sceneReader.extractRotation(node, out_R);
    }
    bool readTranslation(const Mapping& node, Vector3& out_p) const {
        return sceneReader.readTranslation(node, out_p);
    }
    bool readTranslation(const Mapping& node, const char* key, Vector3& out_p) const {
        return sceneReader.readTranslation(node, key, out_p);
    }
    bool extractTranslation(Mapping& node, Vector3& out_p) const {
        return sceneReader.extractTranslation(node, out_p);
    }
};

namespace detail::yaml_body_loader {

template<typename ValueType>
bool extract(Mapping* mapping, const char* key, ValueType& out_value)
{
    ValueNodePtr node = mapping->extract(key);
    if(node){
        out_value = node->to<ValueType>();
        return true;
    }
    return false;
}


template<typename Derived>
bool extractEigen(Mapping* mapping, const char* key, Eigen::MatrixBase<Derived>& x)
{
    ListingPtr listing = dynamic_pointer_cast<Listing>(mapping->extract(key));
    if(listing){
        read(*listing, x);
        return true;
    }
    return false;
}


inline void readInertia(Listing& inertia, Matrix3& I)
{
    if(inertia.size() == 9){
        for(int i=0; i < 3; ++i){
            for(int j=0; j < 3; ++j){
                I(i, j) = inertia[i * 3 + j].toDouble();
            }
        }
    } else if(inertia.size() == 6){
        I(0, 0) = inertia[0].toDouble();
        I(0, 1) = inertia[1].toDouble();
        I(0, 2) = inertia[2].toDouble();
        I(1, 0) = I(0, 1);
        I(1, 1) = inertia[3].toDouble();
        I(1, 2) = inertia[4].toDouble();
        I(2, 0) = I(0, 2);
        I(2, 1) = I(1, 2);
        I(2, 2) = inertia[5].toDouble();
    } else {
        inertia.throwException(_("The number of elements specified as an inertia value must be six or nine"));
    }
}

    
inline bool extractInertia(Mapping* mapping, const char* key, Matrix3& I)
{
    ListingPtr listing = dynamic_pointer_cast<Listing>(mapping->extract(key));
    if(listing){
        detail::yaml_body_loader::readInertia(*listing, I);
        return true;
    }
    return false;
}


inline bool readInertia(Mapping& node, const char* key, Matrix3& I)
{
    Listing* inertia = node.findListing(key);
    if(inertia->isValid()){
        detail::yaml_body_loader::readInertia(*inertia, I);
        return true;
    }
    return false;
}


// for debug
inline void putLinkInfoValues(Body* body, std::ostream& os)
{
    for(int i=0; i < body->numLinks(); ++i){
        Link* link = body->link(i);
        os << "link \"" << link->name() << "\"\n";
        Mapping* info = link->info();
        Mapping::iterator p = info->begin();
        while(p != info->end()){
            if(p->second->isScalar()){
                os << " " << p->first << ": " << p->second->toString() << "\n";
            }
            ++p;
        }
        os.flush();
    }
}

}   // detail::yaml_body_loader


inline YAMLBodyLoader::YAMLBodyLoader()
{
    impl = new YAMLBodyLoaderImpl(this);
}


inline YAMLBodyLoaderImpl::YAMLBodyLoaderImpl(YAMLBodyLoader* self)
    : self(self)
{
    sceneReader.setYAMLReader(&reader);
    
    nodeFunctions["Skip"].set([&](Mapping& node){ return readSkipNode(node); });
    nodeFunctions["Group"].set([&](Mapping& node){ return readGroup(node); });
    nodeFunctions["Transform"].set([&](Mapping& node){ return readTransform(node); });
    nodeFunctions["RigidBody"].setTE([&](Mapping& node){ return readRigidBody(node); });
    nodeFunctions["Visual"].setT([&](Mapping& node){ return readVisualOrCollision(node, true); });
    nodeFunctions["Collision"].setT([&](Mapping& node){ return readVisualOrCollision(node, false); });
    nodeFunctions["Resource"].set([&](Mapping& node){ return readResource(node); });
    nodeFunctions["ForceSensor"].setTE([&](Mapping& node){ return readForceSensor(node); });
    nodeFunctions["RateGyroSensor"].setTE([&](Mapping& node){ return readRateGyroSensor(node); });
    nodeFunctions["AccelerationSensor"].setTE([&](Mapping& node){ return readAccelerationSensor(node); });
    nodeFunctions["Camera"].setTE([&](Mapping& node){ return readCamera(node); });
    nodeFunctions["CameraDevice"].setTE([&](Mapping& node){ return readCamera(node); });
    nodeFunctions["RangeSensor"].setTE([&](Mapping& node){ return readRangeSensor(node); });
    nodeFunctions["SpotLight"].setTE([&](Mapping& node){ return readSpotLight(node); });

    numCustomNodeFunctions = 0;

    body = 0;
    isSubLoader = false;
    os_ = &nullout();
    isVerbose = false;
    isShapeLoadingEnabled = true;
    defaultDivisionNumber = -1;
    defaultCreaseAngle = -1.0;
}


inline YAMLBodyLoader::~YAMLBodyLoader()
{
    delete impl;
}


inline YAMLBodyLoaderImpl::~YAMLBodyLoaderImpl()
{

}


inline void YAMLBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneReader.setMessageSink(os);
}


inline void YAMLBodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


inline void YAMLBodyLoader::setShapeLoadingEnabled(bool on)
{
    impl->isShapeLoadingEnabled = on;
}


inline void YAMLBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->defaultDivisionNumber = n;
}


inline void YAMLBodyLoader::setDefaultCreaseAngle(double theta)
{
    impl->defaultCreaseAngle = theta;
}


inline void YAMLBodyLoaderImpl::updateCustomNodeFunctions()
{
#if UCNOID_NOT_SUPPORTED
    std::lock_guard<std::mutex> guard(customNodeFunctionMutex);
#endif  // UCNOID_NOT_SUPPORTED
    if(detail::yaml_body_loader::customNodeFunctions.size() > numCustomNodeFunctions){
        for(auto& p : detail::yaml_body_loader::customNodeFunctions){
            detail::yaml_body_loader::CustomNodeFunction& func = p.second;
            nodeFunctions[p.first].setTE([&, func](Mapping& node){ return func(*self, node); });
        }
        numCustomNodeFunctions = detail::yaml_body_loader::customNodeFunctions.size();
    }
}


inline YAMLSceneReader& YAMLBodyLoader::sceneReader()
{
    return impl->sceneReader;
}


inline const YAMLSceneReader& YAMLBodyLoader::sceneReader() const
{
    return impl->sceneReader;
}


inline bool YAMLBodyLoader::isDegreeMode() const
{
    return impl->isDegreeMode();
}


inline double YAMLBodyLoader::toRadian(double angle) const
{
    return impl->toRadian(angle);
}


inline bool YAMLBodyLoader::readAngle(const Mapping& node, const char* key, double& angle) const
{
    return impl->readAngle(node, key, angle);
}


inline bool YAMLBodyLoader::readRotation(const Mapping& node, Matrix3& out_R) const
{
    return impl->readRotation(node, out_R);
}


inline bool YAMLBodyLoader::readRotation(const Mapping& node, const char* key, Matrix3& out_R) const
{
    return impl->readRotation(node, key, out_R);
}


inline bool YAMLBodyLoaderImpl::clear()
{
    rootLink = nullptr;
    linkInfos.clear();
    linkMap.clear();
    nameStack.clear();
    transformStack.clear();
    rigidBodies.clear();
    sceneGroupSetStack.clear();
    sceneReader.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;
    subBodyMap.clear();
    subBodies.clear();
    return true;
}    


inline bool YAMLBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


inline bool YAMLBodyLoaderImpl::load(Body* body, const std::string& filename)
{
    mainFilePath = std::filesystem::absolute(filename);
    
    bool result = false;

    try {
        MappingPtr data = reader.loadDocument(filename)->toMapping();
        if(data){
            result = readTopNode(body, data);
            if(result){
                if(body->modelName().empty()){
                    body->setModelName(getBasename(filename));
                }
            }
        }
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    }

    os().flush();

    reader.clearDocuments();
    
    return result;
}


inline bool YAMLBodyLoader::read(Body* body, Mapping* topNode)
{
    return impl->readTopNode(body, topNode);
}


inline bool YAMLBodyLoaderImpl::readTopNode(Body* body, Mapping* topNode)
{
    clear();

    updateCustomNodeFunctions();
    
    bool result = false;
    this->body = body;
    body->clearDevices();
    body->clearExtraJoints();

    try {
        if(checkFormat(topNode)){
            result = readBody(topNode);
        } else {
            result = loadAnotherFormatBodyFile(topNode);
        }
        if(result){
            for(auto& subBody : subBodies){
                topNode->insert(subBody->info());
            }
            body->resetInfo(topNode);
        }
        
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    } catch(const nonexistent_key_error& error){
#if UCNOID_NOT_SUPPORTED
        if(const std::string* message = boost::get_error_info<error_info_message>(error)){
            os() << *message << endl;
        }
#else   // UCNOID_NOT_SUPPORTED
        {
            os() << error.message() << std::endl;
        }
#endif  // UCNOID_NOT_SUPPORTED
    } catch(...){
        clear();
        throw;
    }

    clear();

    os().flush();

    if(false){ // for debug
        detail::yaml_body_loader::putLinkInfoValues(body, os());
    }

    return result;
}


inline bool YAMLBodyLoaderImpl::checkFormat(Mapping* topNode)
{
    auto formatNode = topNode->extract("format");
    if(formatNode){
        if(formatNode->toString() == "ChoreonoidBody"){
            return true;
        }
        formatNode->throwException(
            _("The file format cannot be loaded as a Choreonoid body model"));
    }

    return false;
}


inline bool YAMLBodyLoaderImpl::loadAnotherFormatBodyFile(Mapping* topNode)
{
    auto modelFileNode = topNode->extract("modelFile");
    if(!modelFileNode){
        topNode->throwException(_("Neither format nor modelFile are specified"));
    }

    std::filesystem::path path(modelFileNode->toString());
    if(!path.has_root_path()){
        path = mainFilePath.parent_path() / path;
    }
    if(!bodyLoader){
        bodyLoader.reset(new BodyLoader);
    }

    bodyLoader->setMessageSink(os());
    bodyLoader->setVerbose(isVerbose);
    bodyLoader->setShapeLoadingEnabled(isShapeLoadingEnabled);
    bodyLoader->setDefaultCreaseAngle(defaultCreaseAngle);

    int dn = defaultDivisionNumber;
    auto geometryNode = topNode->extract("geometry");
    if(geometryNode){
        geometryNode->toMapping()->read("divisionNumber", dn);
    }
    bodyLoader->setDefaultDivisionNumber(dn);

    bool loaded = bodyLoader->load(body, path.string());

    if(loaded){
        auto linkInfo = topNode->findMapping("linkInfo");
        if(linkInfo->isValid()){
            auto p = linkInfo->begin();
            while(p != linkInfo->end()){
                auto& linkName = p->first;
                auto node = p->second;
                if(node->isMapping()){
                    auto link = body->link(linkName);
                    if(link){
                        link->info()->insert(node->toMapping());
                    }
                }
                ++p;
            }
        }
    }
    
    return loaded;
}
            

inline bool YAMLBodyLoaderImpl::readBody(Mapping* topNode)
{
    double version = 1.0;
    
    auto versionNode = topNode->extract("formatVersion");
    if(versionNode){
        version = versionNode->toDouble();
    }
    if(version >= 2.0){
        topNode->throwException(_("This version of the Choreonoid body format is not supported"));
    }

    sceneReader.setBaseDirectory(mainFilePath.parent_path().string());
    sceneReader.setDefaultDivisionNumber(defaultDivisionNumber);
    sceneReader.readHeader(*topNode);

    if(detail::yaml_body_loader::extract(topNode, "name", symbol)){
        body->setModelName(symbol);
    }

    transformStack.clear();
    transformStack.push_back(Affine3::Identity());
    auto links = topNode->extract("links");
    std::string unspecifiedType;
    if(!links){
        topNode->throwException(_("There is no \"links\" values for defining the links in the body"));
    } else {
        if(links->isListing()){
            Listing& linkList = *links->toListing();
            if(linkList.empty()){
                linkList.throwException(_("No link is contained in the \"links\" listing"));
            }
            for(int i=0; i < linkList.size(); ++i){
                readNodeInLinks(linkList[i].toMapping(), unspecifiedType);
            }
        } else if(links->isMapping()){
            auto linksMap = links->toMapping();
            auto p = linksMap->begin();
            while(p != linksMap->end()){
                const std::string& type = p->first;
                auto node = p->second->toMapping();
                readNodeInLinks(node, type);
                ++p;
            }
        } else {
            links->throwException(_("Invalid value specified in the \"links\" key"));
        }
    }

    if(linkInfos.empty()){
        topNode->throwException(_("There is no link defined"));
    }

    auto rootLinkNode = topNode->extract("rootLink");
    if(rootLinkNode){
        std::string rootLinkName = rootLinkNode->toString();
        auto p = linkMap.find(rootLinkName);
        if(p == linkMap.end()){
#if UCNOID_NOT_SUPPORTED
            rootLinkNode->throwException(
                format(_("Link \"{}\" specified in \"rootLink\" is not defined"), rootLinkName));
#else   // UCNOID_NOT_SUPPORTED
            rootLinkNode->throwException(
                ssformat("Link \"", rootLinkName, "\" specified in \"rootLink\" is not defined"));
#endif  // UCNOID_NOT_SUPPORTED
        }
        rootLink = p->second;

    } else {
        rootLink = linkInfos[0]->link;
    }

    // construct a link tree
    for(size_t i=0; i < linkInfos.size(); ++i){
        LinkInfo* info = linkInfos[i];
        Link* link = info->link;
        const std::string& parent = info->parent;
        if(parent.empty()){
            if(info->link != rootLink){
#if UCNOID_NOT_SUPPORTED
                info->node->throwException(
                    format(_("The parent of {} is not specified"), link->name()));
#else   // UCNOID_NOT_SUPPORTED
                info->node->throwException(
                    ssformat("The parent of ", link->name(), " is not specified"));
#endif  // UCNOID_NOT_SUPPORTED
            }
        } else {
            auto p = linkMap.find(parent);
            if(p == linkMap.end()){
#if UCNOID_NOT_SUPPORTED
                info->node->throwException(
                    format(_("Parent link \"{0}\" of {1} is not defined"), parent, link->name()));
#else   // UCNOID_NOT_SUPPORTED
                info->node->throwException(
                    ssformat("Parent link \"", parent, "\" of ", link->name(), " is not defined"));
#endif  // UCNOID_NOT_SUPPORTED
            } else {
                Link* parentLink = p->second;
                if(link->isOwnerOf(parentLink)){
#if UCNOID_NOT_SUPPORTED
                    info->node->throwException(
                        format(_("Adding \"{0}\" to link \"{1}\" will result in a cyclic reference"),
                                link->name(), parent));
#else   // UCNOID_NOT_SUPPORTED
                    info->node->throwException(
                        ssformat("Adding \"", link->name(), "\" to link \"", parent, "\" will result in a cyclic reference"));
#endif  // UCNOID_NOT_SUPPORTED
                }
                parentLink->appendChild(link);
            }
        }
    }        

    body->setRootLink(rootLink);

    if(!isSubLoader){
        body->expandLinkOffsetRotations();
    }

    if(!isSubLoader){
        // Warn empty joint ids
        if(numValidJointIds < validJointIdSet.size()){
            for(size_t i=0; i < validJointIdSet.size(); ++i){
                if(!validJointIdSet[i]){
#if UCNOID_NOT_SUPPORTED
                    os() << format(_("Warning: Joint ID {} is not specified."), i) << endl;
#else   // UCNOID_NOT_SUPPORTED
                    os() << ssformat("Warning: Joint ID ", i, " is not specified.") << std::endl;
#endif  // UCNOID_NOT_SUPPORTED
                }
            }
        }
    }

    readExtraJoints(topNode);

    body->installCustomizer();

    return true;
}


inline void YAMLBodyLoaderImpl::readNodeInLinks(Mapping* node, const std::string& nodeType)
{
    std::string type;
    if(detail::yaml_body_loader::extract(node, "type", type)){
        if(!nodeType.empty() && type != nodeType){
#if UCNOID_NOT_SUPPORTED
            node->throwException(
                format(_("The node type \"{0}\" is different from the type \"{1}\" specified in the parent node"),
                        type, nodeType));
#else   // UCNOID_NOT_SUPPORTED
            node->throwException(
                ssformat("The node type \"", type, "\" is different from the type \"", nodeType, "\" specified in the parent node"));
#endif  // UCNOID_NOT_SUPPORTED
        }
    } else if(!nodeType.empty()){
        type = nodeType;
    }

    if(type.empty() || type == "Link"){
        readLinkNode(node);
    } else if(type == "ContinuousTrack"){
        readContinuousTrackNode(node);
    } else if(type == "SubBody"){
        readSubBodyNode(node);
    } else if(type == "Skip"){

    } else {
#if UCNOID_NOT_SUPPORTED
        node->throwException(
            format(_("A {} node cannot be specified in links"), type));
#else   // UCNOID_NOT_SUPPORTED
        node->throwException(
            ssformat("A ", type, " node cannot be specified in links"));
#endif  // UCNOID_NOT_SUPPORTED
    }
}


inline void YAMLBodyLoaderImpl::readLinkNode(Mapping* linkNode)
{
    LinkInfoPtr info = new LinkInfo;
    detail::yaml_body_loader::extract(linkNode, "parent", info->parent);
    info->link = readLinkContents(linkNode);
    info->node = linkNode;
    linkInfos.push_back(info);
}


inline void YAMLBodyLoaderImpl::setLinkName(Link* link, const std::string& name, ValueNode* node)
{
    link->setName(name);
    
    if(!linkMap.insert(make_pair(link->name(), link)).second){
#if UCNOID_NOT_SUPPORTED
        node->throwException(format(_("Duplicated link name \"{}\""), link->name()));
#else   // UCNOID_NOT_SUPPORTED
        node->throwException(ssformat("Duplicated link name \"", link->name(), "\""));
#endif  // UCNOID_NOT_SUPPORTED
    }
}


inline LinkPtr YAMLBodyLoaderImpl::readLinkContents(Mapping* node, LinkPtr link)
{
    bool isSubBodyNode = (link != nullptr);
    
    if(!isSubBodyNode){
        link = body->createLink();
        auto nameNode = node->extract("name");
        if(nameNode){
            setLinkName(link, nameNode->toString(), nameNode);
        }
    }

    if(extractTranslation(*node, v)){
        link->setOffsetTranslation(v);
    }
    Matrix3 R;
    if(extractRotation(*node, R)){
        link->setOffsetRotation(R);
    }

    readJointContents(link, node);

    if(node->read("material", symbol)){
        link->setMaterial(symbol);
    }
    
    currentLink = link;
    rigidBodies.clear();

    ValueNodePtr elements = node->extract("elements");
    if(elements){
        currentModelType = VISUAL_AND_COLLISION;
        hasVisualOrCollisionNodes = false;

        sceneGroupSetStack.push_back(SceneGroupSet());
        currentSceneGroupSet().newGroup<SgInvariantGroup>();
        
        if(readElementContents(*elements) && !isSubBodyNode){
            SceneGroupSet& sgs = currentSceneGroupSet();
            sgs.setName(link->name());
            if(hasVisualOrCollisionNodes){
                link->setVisualShape(sgs.visual);
                link->setCollisionShape(sgs.collision);
            } else {
                link->setShape(sgs.visual);
            }
        }

        sceneGroupSetStack.pop_back();
    }

    if(!isSubBodyNode){
        RigidBody rbody;
        if(!detail::yaml_body_loader::extractEigen(node, "centerOfMass", rbody.c)){
            rbody.c.setZero();
        }
        if(!detail::yaml_body_loader::extract(node, "mass", rbody.m)){
            rbody.m = 0.0;
        }
        if(!detail::yaml_body_loader::extractInertia(node, "inertia", rbody.I)){
            rbody.I.setZero();
        }
        rigidBodies.push_back(rbody);
        setMassParameters(link);

        ValueNode* import = node->find("import");
        if(import->isValid()){
            if(import->isMapping()){
                node->insert(import->toMapping());
            } else if(import->isListing()){
                Listing& importList = *import->toListing();
                for(int i=importList.size() - 1; i >= 0; --i){
                    node->insert(importList[i].toMapping());
                }
            }
        }
        
        link->resetInfo(node);
    }

    currentLink = nullptr;

    return link;
}


inline void YAMLBodyLoaderImpl::setJointId(Link* link, int id)
{
    link->setJointId(id);
    if(id >= 0){
        if(id >= static_cast<int>(validJointIdSet.size())){
            validJointIdSet.resize(id + 1);
        }
        if(!validJointIdSet[id]){
            ++numValidJointIds;
            validJointIdSet[id] = true;
        } else {
#if UCNOID_NOT_SUPPORTED
            os() << format(_("Warning: Joint ID {0} of {1} is duplicated."),
                    id, link->name()) << endl;
#else   // UCNOID_NOT_SUPPORTED
            os() << ssformat("Warning: Joint ID ", id, " of ", link->name(), " is duplicated.") << std::endl;
#endif  // UCNOID_NOT_SUPPORTED
        }
    }
}


inline void YAMLBodyLoaderImpl::readJointContents(Link* link, Mapping* node)
{
    if(detail::yaml_body_loader::extract(node, "jointId", id)){
        setJointId(link, id);
    }

    auto jointTypeNode = node->extract("jointType");
    if(jointTypeNode){
        std::string jointType = jointTypeNode->toString();
        if(jointType == "revolute"){
            link->setJointType(Link::REVOLUTE_JOINT);
        } else if(jointType == "prismatic"){
            link->setJointType(Link::PRISMATIC_JOINT);
        } else if(jointType == "slide"){
            link->setJointType(Link::PRISMATIC_JOINT);
        } else if(jointType == "free"){
            link->setJointType(Link::FREE_JOINT);
        } else if(jointType == "fixed"){
            link->setJointType(Link::FIXED_JOINT);
        } else if(jointType == "pseudoContinuousTrack"){ // deprecated
            link->setJointType(Link::PSEUDO_CONTINUOUS_TRACK);
            link->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        } else {
            jointTypeNode->throwException("Illegal jointType value");
        }
    }

    if(extractAxis(node, "jointAxis", v)){
       link->setJointAxis(v);
    }

    auto actuationModeNode = node->extract("actuationMode");
    if(actuationModeNode){
        std::string mode = actuationModeNode->toString();
        if(mode == "jointEffort" || mode == "jointTorque" || mode == "jointForce"){
            link->setActuationMode(Link::JOINT_EFFORT);
        } else if(mode == "jointDisplacement" || mode == "jointAngle"){
            link->setActuationMode(Link::JOINT_DISPLACEMENT);
        } else if(mode == "jointVelocity"){
            link->setActuationMode(Link::JOINT_VELOCITY);
        } else if(mode == "jointSurfaceVelocity"){
            link->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        } else if(mode == "linkPosition"){
            link->setActuationMode(Link::LINK_POSITION);
        } else {
            actuationModeNode->throwException("Illegal actuationMode value");
        }
    }

    auto jointAngleNode = node->extract("jointAngle");
    if(jointAngleNode){
        link->setInitialJointDisplacement(toRadian(jointAngleNode->toDouble()));
    }
    auto jointDisplacementNode = node->extract("jointDisplacement");
    if(jointDisplacementNode){
        if(link->isRevoluteJoint()){
            link->setInitialJointDisplacement(toRadian(jointDisplacementNode->toDouble()));
        } else {
            link->setInitialJointDisplacement(jointDisplacementNode->toDouble());
        }
    }

    double lower = -std::numeric_limits<double>::max();
    double upper =  std::numeric_limits<double>::max();
    
    auto jointRangeNode = node->extract("jointRange");
    if(jointRangeNode){
        if(jointRangeNode->isScalar()){
            upper = readLimitValue(*jointRangeNode, true);
            lower = -upper;
        } else if(jointRangeNode->isListing()){
            Listing& jointRange = *jointRangeNode->toListing();
            if(jointRange.size() != 2){
                jointRangeNode->throwException(_("jointRange must have two elements"));
            }
            lower = readLimitValue(jointRange[0], false);
            upper = readLimitValue(jointRange[1], true);
        } else {
            jointRangeNode->throwException(_("Invalid type value is specefied as a jointRange"));
        }
    }
    if(link->jointType() == Link::REVOLUTE_JOINT && isDegreeMode()){
        link->setJointRange(
            lower == -std::numeric_limits<double>::max() ? lower : radian(lower),
            upper ==  std::numeric_limits<double>::max() ? upper : radian(upper));
    } else {
        link->setJointRange(lower, upper);
    }
    
    auto maxVelocityNode = node->extract("maxJointVelocity");
    if(maxVelocityNode){
        double maxVelocity = maxVelocityNode->toDouble();
        if(link->jointType() == Link::REVOLUTE_JOINT){
            link->setJointVelocityRange(toRadian(-maxVelocity), toRadian(maxVelocity));
        } else {
            link->setJointVelocityRange(-maxVelocity, maxVelocity);
        }
    }

    auto velocityRangeNode = node->extract("jointVelocityRange");
    if(velocityRangeNode){
        Listing& velocityRange = *velocityRangeNode->toListing();
        if(velocityRange.size() != 2){
            velocityRangeNode->throwException(_("jointVelocityRange must have two elements"));
        }
        if(link->jointType() == Link::REVOLUTE_JOINT){
            link->setJointVelocityRange(toRadian(velocityRange[0].toDouble()), toRadian(velocityRange[1].toDouble()));
        } else {
            link->setJointVelocityRange(velocityRange[0].toDouble(), velocityRange[1].toDouble());
        }
    }

    double Ir = node->get("rotorInertia", 0.0);
    double r = node->get("gearRatio", 1.0);
    link->setEquivalentRotorInertia(r * r * Ir);
}


inline bool YAMLBodyLoaderImpl::extractAxis(Mapping* node, const char* key, Vector3& out_axis)
{
    auto axisNode = node->extract(key);
    if(axisNode){
        readAxis(axisNode, out_axis);
        return true;
    }
    return false;
}


inline bool YAMLBodyLoaderImpl::readAxis(Mapping* node, const char* key, Vector3& out_axis)
{
    auto axisNode = node->find(key);
    if(axisNode->isValid()){
        readAxis(axisNode, out_axis);
        return true;
    }
    return false;
}


inline void YAMLBodyLoaderImpl::readAxis(ValueNode* node, Vector3& out_axis)
{
    if(node->isListing()){
        read(*node->toListing(), out_axis);

    } else if(node->isString()){
        std::string symbol = node->toString();
        std::transform(symbol.cbegin(), symbol.cend(), symbol.begin(), ::toupper);
        if(symbol == "X"){
            out_axis = Vector3::UnitX();
        } else if(symbol == "-X"){
            out_axis = -Vector3::UnitX();
        } else if(symbol == "Y"){
            out_axis = Vector3::UnitY();
        } else if(symbol == "-Y"){
            out_axis = -Vector3::UnitY();
        } else if(symbol == "Z"){
            out_axis = Vector3::UnitZ();
        } else if(symbol == "-Z"){
            out_axis = -Vector3::UnitZ();
        } else {
            node->throwException("Illegal axis value");
        }
    }
}


inline void YAMLBodyLoaderImpl::setMassParameters(Link* link)
{
    /*
      Mass = Sigma mass 
      C = (Sigma mass * T * c) / Mass 
      I = Sigma(R * I * Rt + m * G)       
      R = Rotation matrix part of T   
      G = y*y+z*z, -x*y, -x*z, -y*x, z*z+x*x, -y*z, -z*x, -z*y, x*x+y*y    
      (x, y, z ) = T * c - C
    */
    
    Vector3 c = Vector3::Zero();
    double m = 0.0;
    Matrix3 I = Matrix3::Zero();

    for(size_t i=0; i < rigidBodies.size(); ++i){
        const RigidBody& r = rigidBodies[i];
        m += r.m;
        c += r.m * r.c;
    }
    if(m > 0.0){
        c = c / m;

        for(size_t i=0; i < rigidBodies.size(); ++i){
            const RigidBody& r = rigidBodies[i];
            I += r.I;
            const Vector3 o = c - r.c;
            const double x = o.x();
            const double y = o.y();
            const double z = o.z();
            const double m = r.m;
            I(0,0) +=  m * (y * y + z * z);
            I(0,1) += -m * (x * y);
            I(0,2) += -m * (x * z);
            I(1,0) += -m * (y * x);
            I(1,1) +=  m * (z * z + x * x);
            I(1,2) += -m * (y * z);
            I(2,0) += -m * (z * x);
            I(2,1) += -m * (z * y);
            I(2,2) +=  m * (x * x + y * y);
        }
    }

    link->setCenterOfMass(c);
    link->setMass(m);
    link->setInertia(I);
}


inline bool YAMLBodyLoaderImpl::readElements(Mapping& node)
{
    bool isSceneNodeAdded = false;
    ValueNode& elements = *node.find("elements");
    if(elements.isValid()){
        isSceneNodeAdded = readElementContents(elements);
    }
    return isSceneNodeAdded;
}


inline bool YAMLBodyLoaderImpl::readElementContents(ValueNode& elements)
{
    bool isSceneNodeAdded = false;

    if(elements.isListing()){
        /*
          Process the case like
          elements:
            -
              type: Transform
              translation: [ 1, 0, 0 ]
        */
        Listing& listing = *elements.toListing();
        for(int i=0; i < listing.size(); ++i){
            Mapping& element = *listing[i].toMapping();
            const std::string type = element["type"].toString();
            if(readNode(element, type)){
                isSceneNodeAdded = true;
            }
        }
    } else if(elements.isMapping()){
        Mapping& mapping = *elements.toMapping();

        /* Check the case like
           elements:
             type: Transform
             translation: [ 1, 0, 0 ]
        */
        ValueNode* typeNode = mapping.find("type");
        if(typeNode->isValid()){
            const std::string type = typeNode->toString();
            if(readNode(mapping, type)){
                isSceneNodeAdded = true;
            }
        } else {
            /*
               Process the case like
               elements:
                 Transform:
                   translation: [ 1, 0, 0 ]
            */
            Mapping::iterator p = mapping.begin();
            while(p != mapping.end()){
                const std::string& type = p->first;
                Mapping& element = *p->second->toMapping();
                ValueNode* typeNode = element.find("type");
                if(typeNode->isValid()){
                    std::string type2 = typeNode->toString();
                    if(type2 != type){
#if UCNOID_NOT_SUPPORTED
                        element.throwException(
                            format(_("The node type \"{0}\" is different from the type \"{1}\" specified in the parent node"),
                                    type2, type));
#else   // UCNOID_NOT_SUPPORTED
                        element.throwException(
                            ssformat("The node type \"", type2, "\" is different from the type \"", type, "\" specified in the parent node"));
#endif  // UCNOID_NOT_SUPPORTED
                    }
                }
                if(readNode(element, type)){
                    isSceneNodeAdded = true;
                }
                ++p;
            }
        }
    } else {
        elements.throwException(_("A value of the \"elements\" key must be a sequence or a mapping"));
    }

    return isSceneNodeAdded;
}


inline bool YAMLBodyLoaderImpl::readNode(Mapping& node, const std::string& type)
{
    bool isSceneNodeAdded = false;
    
    auto p = nodeFunctions.find(type);
    if(p != nodeFunctions.end()){
        NodeFunctionInfo& info = p->second;

        nameStack.push_back(std::string());
        node.read("name", nameStack.back());
        
        if(info.isTransformDerived){
            if(readTransformContents(node, info.function, info.hasElements)){
                isSceneNodeAdded = true;
            }
        } else {
            if(info.function(node)){
                isSceneNodeAdded = true;
            }
        }

        nameStack.pop_back();
        
    } else if(isShapeLoadingEnabled){
        SgNode* scene = sceneReader.readNode(node, type);
        if(scene){
            addScene(scene);
            isSceneNodeAdded = true;
        }
    }

    return isSceneNodeAdded;
}


inline bool YAMLBodyLoaderImpl::readSkipNode(Mapping& node)
{
    return false;
}


inline bool YAMLBodyLoaderImpl::readContainerNode(Mapping& node, NodeFunction nodeFunction)
{
    bool isSceneNodeAdded = false;

    if(nodeFunction){
        if(nodeFunction(node)){
            isSceneNodeAdded = true;
        }
    }

    ValueNode& elements = *node.find("elements");
    if(elements.isValid()){
        if(readElementContents(elements)){
            isSceneNodeAdded = true;
        }
    }

    return isSceneNodeAdded;
}


inline bool YAMLBodyLoaderImpl::readTransformContents(Mapping& node, NodeFunction nodeFunction, bool hasElements)
{
    Affine3 T = Affine3::Identity();
    bool hasPosTransform = false;
    bool hasScale = false;
    bool isSceneNodeAdded = false;
    
    if(readTranslation(node, v)){
        T.translation() = v;
        hasPosTransform = true;
    }
    if(readRotation(node, M)){
        T.linear() = M;
        hasPosTransform = true;
    }

    Affine3 Ts(T);
    Vector3 scale;
    if(read(node, "scale", scale)){
        Ts.linear() *= scale.asDiagonal();
        hasScale = true;
    }
    
    if(hasPosTransform || hasScale){
        transformStack.push_back(transformStack.back() * Ts);
    }

    sceneGroupSetStack.push_back(SceneGroupSet());
    if(hasPosTransform){
        currentSceneGroupSet().newGroup<SgPosTransform>(T);
    } else if(hasScale){
        currentSceneGroupSet().newGroup<SgScaleTransform>(scale);
    } else {
        currentSceneGroupSet().newGroup<SgGroup>();
    }
    currentSceneGroupSet().setName(nameStack.back());
    
    if(hasPosTransform && hasScale){
        sceneGroupSetStack.push_back(SceneGroupSet());
        currentSceneGroupSet().newGroup<SgScaleTransform>(scale);
        currentSceneGroupSet().setName(nameStack.back());
    }
    
    if(hasElements){
        isSceneNodeAdded = readContainerNode(node, nodeFunction);
    } else {
        isSceneNodeAdded = nodeFunction(node);
    }

    if(isSceneNodeAdded){
        addCurrentSceneGroupToParentSceneGroup();
    }

    if(hasPosTransform && hasScale){
        sceneGroupSetStack.pop_back();
        if(isSceneNodeAdded){
            addCurrentSceneGroupToParentSceneGroup();
        }
    }
    sceneGroupSetStack.pop_back();

    if(hasPosTransform || hasScale){
        transformStack.pop_back();
    }

    return isSceneNodeAdded;
}


inline bool YAMLBodyLoaderImpl::readGroup(Mapping& node)
{
    sceneGroupSetStack.push_back(SceneGroupSet());
    currentSceneGroupSet().newGroup<SgGroup>();
    currentSceneGroupSet().setName(nameStack.back());

    bool isSceneNodeAdded = readContainerNode(node, 0);
    if(isSceneNodeAdded){
        addCurrentSceneGroupToParentSceneGroup();
    }
    sceneGroupSetStack.pop_back();
    return isSceneNodeAdded;
}


inline bool YAMLBodyLoaderImpl::readTransform(Mapping& node)
{
    return readTransformContents(node, 0, true);
}


inline bool YAMLBodyLoaderImpl::readRigidBody(Mapping& node)
{
    RigidBody rbody;
    const Affine3& T = transformStack.back();

    if(!read(node, "centerOfMass", v)){
        v.setZero();
    }
    rbody.c = T.linear() * v + T.translation();

    if(!node.read("mass", rbody.m)){
        rbody.m = 0.0;
    }
    if(detail::yaml_body_loader::readInertia(node, "inertia", M)){
        rbody.I = T.linear() * M * T.linear().transpose();
    } else {
        rbody.I.setZero();
    }
    rigidBodies.push_back(rbody);

    return false;
}


inline bool YAMLBodyLoaderImpl::readVisualOrCollision(Mapping& node, bool isVisual)
{
    ModelType prevModelType = currentModelType;

    if(isVisual){
        if(currentModelType == COLLISION){
            node.throwException(
                _("The visual node is conflicting with the Collision node defined at the higher level"));
        }
        currentModelType = VISUAL;
    } else {
        if(currentModelType == VISUAL){
            node.throwException(
                _("The collision node is conflicting with the Visual node defined at the higher level"));
        }
        currentModelType = COLLISION;
    }

    bool isSceneNodeAdded = readElements(node);

    if(isShapeLoadingEnabled){
        auto resourceNode = node.findMapping("resource");
        if(resourceNode->isValid()){
            isSceneNodeAdded = readResource(*resourceNode);
        }
        auto shape = node.findMapping("shape");
        if(shape->isValid()){
            if(auto scene = sceneReader.readNode(*shape, "Shape")){
                addScene(scene);
                isSceneNodeAdded = true;
            }
        }
    }
 
    if(isSceneNodeAdded){
        hasVisualOrCollisionNodes = true;
    }

    currentModelType = prevModelType;

    return isSceneNodeAdded;
}


inline bool YAMLBodyLoaderImpl::readResource(Mapping& node)
{
    bool isSceneNodeAdded = false;
    
    auto resource = sceneReader.readResourceNode(node);

    if(resource.scene){
        addScene(resource.scene);
        isSceneNodeAdded = true;

    } else if(resource.info){
        std::string orgBaseDirectory = sceneReader.baseDirectory();
        sceneReader.setBaseDirectory(resource.directory);

        //isSceneNodeAdded = readElementContents(*resource.node);
        ValueNodePtr resourceNode = resource.info;
        isSceneNodeAdded = readTransformContents(
            node,
            [this, resourceNode](Mapping&){
                return readElementContents(*resourceNode); },
            false);

        sceneReader.setBaseDirectory(orgBaseDirectory);
    }

    return isSceneNodeAdded;
}
        

inline bool YAMLBodyLoader::readDevice(Device* device, Mapping& node)
{
    return impl->readDevice(device, node);
}


inline bool YAMLBodyLoaderImpl::readDevice(Device* device, Mapping& node)
{
    device->setName(nameStack.back());

    if(node.read("id", id)) device->setId(id);
    if(node.read("on", on)) device->on(on);

    const Affine3& T = transformStack.back();
    device->setLocalTranslation(T.translation());
    device->setLocalRotation(T.linear());
    device->setLink(currentLink);
    body->addDevice(device);

    return false;
}


inline bool YAMLBodyLoaderImpl::readForceSensor(Mapping& node)
{
    ForceSensorPtr sensor = new ForceSensor;
    if(read(node, "maxForce",  v)) sensor->F_max().head<3>() = v;
    if(read(node, "maxTorque", v)) sensor->F_max().tail<3>() = v;
    return readDevice(sensor, node);
}


inline bool YAMLBodyLoaderImpl::readRateGyroSensor(Mapping& node)
{
    RateGyroSensorPtr sensor = new RateGyroSensor;
    if(read(node, "maxAngularVelocity", v)){
        if(isDegreeMode()){
            for(int i=0; i < 3; ++i){
                v[i] = radian(v[i]);
            }
        }
        sensor->w_max() = v;
    }
    return readDevice(sensor, node);
}


inline bool YAMLBodyLoaderImpl::readAccelerationSensor(Mapping& node)
{
    AccelerationSensorPtr sensor = new AccelerationSensor();
    if(read(node, "maxAcceleration", v)) sensor->dv_max() = v;
    return readDevice(sensor, node);
}


inline bool YAMLBodyLoaderImpl::readCamera(Mapping& node)
{
    CameraPtr camera;
    RangeCamera* range = 0;

    std::string format;
    if(node.read("format", format)){
        if(format == "COLOR"){
            camera = new Camera;
            camera->setImageType(Camera::COLOR_IMAGE);
        } else if(format == "DEPTH"){
            range = new RangeCamera;
            range->setOrganized(true);
            range->setImageType(Camera::NO_IMAGE);
        } else if(format == "COLOR_DEPTH"){
            range = new RangeCamera;
            range->setOrganized(true);
            range->setImageType(Camera::COLOR_IMAGE);
        } else if(format == "POINT_CLOUD"){
            range = new RangeCamera;
            range->setOrganized(false);
            range->setImageType(Camera::NO_IMAGE);
        } else if(format == "COLOR_POINT_CLOUD"){
            range = new RangeCamera;
            range->setOrganized(false);
            range->setImageType(Camera::COLOR_IMAGE);
        }
    }

    if(!camera){
        if(range){
            camera = range;
        } else {
            camera = new Camera;
        }
    }

    if(node.read("lensType", symbol)){
        if(symbol == "NORMAL"){
            camera->setLensType(Camera::NORMAL_LENS);
        } else if(symbol == "FISHEYE"){
            camera->setLensType(Camera::FISHEYE_LENS);
            //  throwException    ImageType must be COLOR
        } else if(symbol == "DUAL_FISHEYE"){
            camera->setLensType(Camera::DUAL_FISHEYE_LENS);
            //  throwException    ImageType must be COLOR
        }
    }

    if(node.read("width", value)) camera->setResolutionX(value);
    if(node.read("height", value)) camera->setResolutionY(value);
    if(readAngle(node, "fieldOfView", value)) camera->setFieldOfView(value);
    if(node.read("nearClipDistance", value)) camera->setNearClipDistance(value);
    if(node.read("farClipDistance", value)) camera->setFarClipDistance(value);
    if(node.read("frameRate", value)) camera->setFrameRate(value);
    
    return readDevice(camera, node);
}


inline bool YAMLBodyLoaderImpl::readRangeSensor(Mapping& node)
{
    RangeSensorPtr rangeSensor = new RangeSensor;
    
    if(readAngle(node, "yawRange", value)){
        rangeSensor->setYawRange(value);
    } else if(readAngle(node, "scanAngle", value)){ // backward compatibility
        rangeSensor->setYawRange(value);
    }
    if(readAngle(node, "yawStep", value)){
        rangeSensor->setYawStep(value);
    } else if(readAngle(node, "scanStep", value)){ // backward compatibility
        rangeSensor->setYawStep(value);
    }

    if(readAngle(node, "pitchRange", value)) rangeSensor->setPitchRange(value);
    if(readAngle(node, "pitchStep", value)) rangeSensor->setPitchStep(value);
    if(node.read("minDistance", value)) rangeSensor->setMinDistance(value);
    if(node.read("maxDistance", value)) rangeSensor->setMaxDistance(value);
    if(node.read("scanRate", value)) rangeSensor->setScanRate(value);
    
    return readDevice(rangeSensor, node);
}


inline bool YAMLBodyLoaderImpl::readSpotLight(Mapping& node)
{
    SpotLightPtr light = new SpotLight();

    if(read(node, "color", color)) light->setColor(color);
    if(node.read("intensity", value)) light->setIntensity(value);
    if(read(node, "direction", v)) light->setDirection(v);
    if(readAngle(node, "beamWidth", value)) light->setBeamWidth(value);
    if(readAngle(node, "cutOffAngle", value)) light->setCutOffAngle(value);
    if(node.read("cutOffExponent", value)) light->setCutOffExponent(value);
    if(read(node, "attenuation", color)){
        light->setConstantAttenuation(color[0]);
        light->setLinearAttenuation(color[1]);
        light->setQuadraticAttenuation(color[2]);
    }

    return readDevice(light, node);
}


inline void YAMLBodyLoaderImpl::readContinuousTrackNode(Mapping* node)
{
    std::string parent;
    if(!detail::yaml_body_loader::extract(node, "parent", parent)){
        node->throwException("parent must be specified");
    }

    int numJoints = 0;
    if(!detail::yaml_body_loader::extract(node, "numJoints", numJoints)){
        node->throwException("numJoints must be specified");
    }
    if(numJoints < 3){
        node->throwException("numJoints must be more than 2");
    }

    Vector3 jointOffset;
    if(!detail::yaml_body_loader::extractEigen(node, "jointOffset", jointOffset)){
        node->throwException("jointOffset must be specified");
    }

    const int numOpenJoints = numJoints - 1;
    std::vector<double> initialAngles(numOpenJoints, 0.0);
    ValueNodePtr initAnglesNode = node->extract("initialJointAngles");
    if(initAnglesNode){
        Listing& initAngles = *initAnglesNode->toListing();
        const int n = std::min(initAngles.size(), numOpenJoints);
        for(int i=0; i < n; i++){
            initialAngles[i] = toRadian(initAngles[i].toDouble());
        }
    }

    LinkPtr firstLink = readLinkContents(node);
    LinkPtr subsequentLink = firstLink->clone();
    subsequentLink->resetInfo(subsequentLink->info()->cloneMapping());
    
    firstLink->setJointType(Link::FREE_JOINT);
    firstLink->setInfo("isContinuousTrack", true);
    addTrackLink(0, firstLink, node, parent, 0.0);

    subsequentLink->setJointType(Link::REVOLUTE_JOINT);
    subsequentLink->setOffsetTranslation(jointOffset);
    for(int i=0; i < numOpenJoints; ++i){
        addTrackLink(i + 1, subsequentLink->clone(), node, parent, initialAngles[i]);
    }
}


inline void YAMLBodyLoaderImpl::addTrackLink(int index, LinkPtr link, Mapping* node, std::string& io_parent, double initialAngle)
{
#if UCNOID_NOT_SUPPORTED
    setLinkName(link, format("{0}{1}", link->name(), index), node);
#else   // UCNOID_NOT_SUPPORTED
    setLinkName(link, ssformat(link->name(), index), node);
#endif  // UCNOID_NOT_SUPPORTED
    link->setInitialJointAngle(initialAngle);

    LinkInfoPtr info = new LinkInfo;
    info->link = link;
    info->node = node;
    info->parent = io_parent;
    linkInfos.push_back(info);
    
    io_parent = link->name();
}


inline void YAMLBodyLoaderImpl::readSubBodyNode(Mapping* node)
{
    std::string uri;
    if(!node->read("uri", uri)){
        node->throwException("uri must be specified");
    }
    
    std::filesystem::path filepath(uri);
    if(filepath.is_relative()){
        filepath = getCompactPath(std::filesystem::path(sceneReader.baseDirectory()) / filepath);
    }
    if(std::filesystem::equivalent(mainFilePath, filepath)){
        node->throwException("recursive sub-body is prohibited");
    }

    BodyPtr subBody;
    std::string filename = filepath.string();
    auto iter = subBodyMap.find(filename);
    if(iter != subBodyMap.end()){
        subBody = iter->second;
    } else {
        try {
            if(!subLoader){
                subLoader.reset(new YAMLBodyLoader);
                subLoader->setMessageSink(*os_);
                subLoader->impl->isSubLoader = true;
            }
            subLoader->setDefaultDivisionNumber(sceneReader.defaultDivisionNumber());
                
            subBody = new Body;
            if(subLoader->load(subBody, filename)){
                subBodyMap[filename] = subBody;
            } else {
#if UCNOID_NOT_SUPPORTED
                os() << format(_("SubBody specified by uri \"{}\" cannot be loaded."), uri) << endl;
#else   // UCNOID_NOT_SUPPORTED
                os() << ssformat("SubBody specified by uri \"", uri, "\" cannot be loaded.") << std::endl;
#endif  // UCNOID_NOT_SUPPORTED
                subBody.reset();
            }
        } catch(const ValueNode::Exception& ex){
            os() << ex.message();
        }
    }

    if(subBody){
        addSubBodyLinks(subBody->clone(), node);
        subBodies.push_back(subBody);
    }
}


inline void YAMLBodyLoaderImpl::addSubBodyLinks(BodyPtr subBody, Mapping* node)
{
    std::string prefix;
    node->read("prefix", prefix);
    std::string devicePrefix;
    if(!node->read("devicePrefix", devicePrefix)){
        devicePrefix = prefix;
    }

    int jointIdOffset = node->get("jointIdOffset", 0);

    for(int i=0; i < subBody->numLinks(); ++i){
        Link* link = subBody->link(i);
        setLinkName(link, prefix + link->name(), node);
        if(link->jointId() >= 0){
            setJointId(link, link->jointId() + jointIdOffset);
        }
    }

    const int numExtraJoints = subBody->numExtraJoints();
    for(int i=0; i < numExtraJoints; ++i){
        body->addExtraJoint(subBody->extraJoint(i));
    }
    
    for(int i=0; i < subBody->numDevices(); ++i){
        Device* device = subBody->device(i);
        device->setName(devicePrefix + device->name());
        body->addDevice(device);
    }

    LinkPtr rootLink = subBody->rootLink();

    readLinkContents(node, rootLink);
    
    LinkInfoPtr linkInfo = new LinkInfo;
    linkInfo->link = rootLink;
    linkInfo->node = node;
    node->read("parent", linkInfo->parent);
    linkInfos.push_back(linkInfo);
}


inline void YAMLBodyLoaderImpl::readExtraJoints(Mapping* topNode)
{
    auto node = topNode->extract("extraJoints");
    if(node){
        if(node->isListing()){
            Listing& extraJoints = *node->toListing();
            for(int i=0; i < extraJoints.size(); ++i){
                readExtraJoint(extraJoints[i].toMapping());
            }
        }
    }
}


inline void YAMLBodyLoaderImpl::readExtraJoint(Mapping* node)
{
    ExtraJoint joint;

    joint.body[0] = joint.body[1] = body;
    
    joint.link[0] = body->link(node->get("link1Name").toString());
    joint.link[1] = body->link(node->get("link2Name").toString());

    for(int i=0; i < 2; ++i){
        if(!joint.link[i]){
#if UCNOID_NOT_SUPPORTED
            node->throwException(
                format(_("The link specified in \"link{}Name\" is not found"), (i + 1)));
#else   // UCNOID_NOT_SUPPORTED
            node->throwException(
                ssformat("The link specified in \"link", (i + 1), "Name\" is not found"));
#endif  // UCNOID_NOT_SUPPORTED
        }
    }

    std::string jointType = node->get("jointType").toString();
    if(jointType == "piston"){
        joint.type = ExtraJoint::EJ_PISTON;
        if(!readAxis(node, "jointAxis", joint.axis)){
            node->throwException(_("The jointAxis value must be specified for the pistion type"));
        }
    } else if(jointType == "ball"){
        joint.type = ExtraJoint::EJ_BALL;
    } else {
#if UCNOID_NOT_SUPPORTED
        node->throwException(format(_("Joint type \"{}\" is not available"), jointType));
#else   // UCNOID_NOT_SUPPORTED
        node->throwException(ssformat("Joint type \"", jointType, "\" is not available"));
#endif  // UCNOID_NOT_SUPPORTED
    }

    readEx(*node, "link1LocalPos", joint.point[0]);
    readEx(*node, "link2LocalPos", joint.point[1]);

    body->addExtraJoint(joint);
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_YAML_BODY_LOADER_CPP_H
