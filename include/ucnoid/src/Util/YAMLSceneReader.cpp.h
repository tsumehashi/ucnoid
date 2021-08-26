/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_YAML_SCENE_READER_CPP_H
#define UCNOID_UTIL_YAML_SCENE_READER_CPP_H

#include "YAMLSceneReader.h"
#include <ucnoid/SceneDrawables>
#include <ucnoid/SceneLights>
#include <ucnoid/MeshGenerator>
#include <ucnoid/PolygonMeshTriangulator>
#include <ucnoid/MeshFilter>
#include <ucnoid/SceneLoader>
#include <ucnoid/EigenArchive>
#include <ucnoid/YAMLReader>
#include <ucnoid/FileUtil>
#include <ucnoid/NullOut>
#include <ucnoid/Exception>
#include <ucnoid/ImageIO>
#include <ucnoid/Config>
#include <unordered_map>
#if UCNOID_NOT_SUPPORTED
#include <fmt/format.h>
#include <mutex>
#endif  // UCNOID_NOT_SUPPORTED

#ifdef CNOID_USE_BOOST_REGEX
#include <boost/regex.hpp>
using boost::regex;
using boost::smatch;
using boost::regex_match;
#else
#include <regex>
#endif

#include "gettext.h"

namespace cnoid {
inline namespace ucnoid {
namespace detail::yaml_scene_reader {

typedef SgNode* (YAMLSceneReaderImpl::*NodeFunction)(Mapping& info);
typedef std::unordered_map<std::string, NodeFunction> NodeFunctionMap;
static inline NodeFunctionMap nodeFunctionMap;

struct SceneNodeInfo
{
    SgGroupPtr parent;
    SgNodePtr node;
    Matrix3 R;
    bool isScaled;
};

typedef std::unordered_map<std::string, SceneNodeInfo> SceneNodeMap;
    
struct ResourceInfo : public Referenced
{
    SgNodePtr scene;
    std::unique_ptr<SceneNodeMap> sceneNodeMap;
    std::unique_ptr<YAMLReader> yamlReader;
    std::string directory;
};
typedef ref_ptr<ResourceInfo> ResourceInfoPtr;


static inline std::unordered_map<std::string, YAMLSceneReader::UriSchemeHandler> uriSchemeHandlerMap;
#if UCNOID_NOT_SUPPORTED
std::mutex uriSchemeHandlerMutex;
#endif  // UCNOID_NOT_SUPPORTED

}   // namespace detail::yaml_scene_reader

class YAMLSceneReaderImpl
{
public:
    YAMLSceneReader* self;

    std::ostream* os_;
    std::ostream& os() { return *os_; }

    YAMLReader* mainYamlReader;

    // temporary variables for reading values
    double value;
    std::string symbol;
    Vector3f color;
    Vector3 v;
    Vector2 v2;
    bool on;
    
    MeshGenerator meshGenerator;
    PolygonMeshTriangulator polygonMeshTriangulator;
    MeshFilter meshFilter;
    SgMaterialPtr defaultMaterial;
    ImageIO imageIO;

    std::map<std::string, detail::yaml_scene_reader::ResourceInfoPtr> resourceInfoMap;
    SceneLoader sceneLoader;
    std::filesystem::path baseDirectory;
    std::regex uriSchemeRegex;
    bool isUriSchemeRegexReady;
    typedef std::map<std::string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;
    bool generateTexCoord;

    YAMLSceneReaderImpl(YAMLSceneReader* self);
    ~YAMLSceneReaderImpl();

    bool readAngle(const Mapping& info, const char* key, double& angle) const{
        return self->readAngle(info, key, angle);
    }

    SgNode* readNode(Mapping& info);
    SgNode* readNode(Mapping& info, const std::string& type);
    SgNode* readNodeNode(Mapping& info);
    SgNode* readGroup(Mapping& info);
    void readElements(Mapping& info, SgGroup* group);
    void readNodeList(ValueNode& elements, SgGroup* group);
    SgNode* readTransform(Mapping& info);
    SgNode* readTransformParameters(Mapping& info, SgNode* scene);
    SgNode* readShape(Mapping& info);
    SgMesh* readGeometry(Mapping& info);
    SgMesh* readBox(Mapping& info);
    SgMesh* readSphere(Mapping& info);
    SgMesh* readCylinder(Mapping& info);
    SgMesh* readCone(Mapping& info);
    SgMesh* readCapsule(Mapping& info);
    SgMesh* readExtrusion(Mapping& info);
    SgMesh* readElevationGrid(Mapping& info);
    SgMesh* readIndexedFaceSet(Mapping& info);
    SgMesh* readResourceAsGeometry(Mapping& info);
    void readAppearance(SgShape* shape, Mapping& info);
    void readMaterial(SgShape* shape, Mapping& info);
    void setDefaultMaterial(SgShape* shape);
    void readTexture(SgShape* shape, Mapping& info);
    void readTextureTransform(SgTexture* texture, Mapping& info);
    void readLightCommon(Mapping& info, SgLight* light);
    SgNode* readDirectionalLight(Mapping& info);
    SgNode* readSpotLight(Mapping& info);
    SgNode* readResource(Mapping& info);
    YAMLSceneReader::Resource readResourceNode(Mapping& info);
    YAMLSceneReader::Resource loadResource(Mapping& resourceNode, const std::string& uri);
    void extractNamedYamlNodes(
        Mapping& resourceNode, detail::yaml_scene_reader::ResourceInfo* info, std::vector<std::string>& names, const std::string& uri, YAMLSceneReader::Resource& resource);
    void extractNamedSceneNodes(
        Mapping& resourceNode, detail::yaml_scene_reader::ResourceInfo* info, std::vector<std::string>& names, const std::string& uri, YAMLSceneReader::Resource& resource);
    void decoupleResourceNode(Mapping& resourceNode, const std::string& uri, const std::string& nodeName);
    detail::yaml_scene_reader::ResourceInfo* getOrCreateResourceInfo(Mapping& resourceNode, const std::string& uri);
    std::filesystem::path findFileInPackage(const std::string& file);
    void adjustNodeCoordinate(detail::yaml_scene_reader::SceneNodeInfo& info);
    void makeSceneNodeMap(detail::yaml_scene_reader::ResourceInfo* info);
    void makeSceneNodeMapSub(const detail::yaml_scene_reader::SceneNodeInfo& nodeInfo, detail::yaml_scene_reader::SceneNodeMap& nodeMap);
};

namespace detail::yaml_scene_reader {

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

}   // namespace detail::yaml_scene_reader


inline void YAMLSceneReader::registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler)
{
#if UCNOID_NOT_SUPPORTED
    std::lock_guard<std::mutex> guard(uriSchemeHandlerMutex);
#endif  // UCNOID_NOT_SUPPORTED
    detail::yaml_scene_reader::uriSchemeHandlerMap[scheme] = handler;
}
                                                 

inline YAMLSceneReader::YAMLSceneReader()
{
    impl = new YAMLSceneReaderImpl(this);
    clear();
}


inline YAMLSceneReaderImpl::YAMLSceneReaderImpl(YAMLSceneReader* self)
    : self(self)
{
    if(detail::yaml_scene_reader::nodeFunctionMap.empty()){
        detail::yaml_scene_reader::nodeFunctionMap["Node"] = &YAMLSceneReaderImpl::readNodeNode;
        detail::yaml_scene_reader::nodeFunctionMap["Group"] = &YAMLSceneReaderImpl::readGroup;
        detail::yaml_scene_reader::nodeFunctionMap["Transform"] = &YAMLSceneReaderImpl::readTransform;
        detail::yaml_scene_reader::nodeFunctionMap["Shape"] = &YAMLSceneReaderImpl::readShape;
        detail::yaml_scene_reader::nodeFunctionMap["DirectionalLight"] = &YAMLSceneReaderImpl::readDirectionalLight;
        detail::yaml_scene_reader::nodeFunctionMap["SpotLight"] = &YAMLSceneReaderImpl::readSpotLight;
        detail::yaml_scene_reader::nodeFunctionMap["Resource"] = &YAMLSceneReaderImpl::readResource;
    }
    os_ = &nullout();
    isUriSchemeRegexReady = false;
    imageIO.setUpsideDown(true);
}


inline YAMLSceneReader::~YAMLSceneReader()
{
    delete impl;
}


inline YAMLSceneReaderImpl::~YAMLSceneReaderImpl()
{

}


inline void YAMLSceneReader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneLoader.setMessageSink(os);
}


inline void YAMLSceneReader::setDefaultDivisionNumber(int n)
{
    impl->meshGenerator.setDivisionNumber(n);
    impl->sceneLoader.setDefaultDivisionNumber(n);
}


inline int YAMLSceneReader::defaultDivisionNumber() const
{
    return impl->meshGenerator.divisionNumber();
}


inline void YAMLSceneReader::setBaseDirectory(const std::string& directory)
{
    impl->baseDirectory = directory;
}


inline std::string YAMLSceneReader::baseDirectory()
{
    return impl->baseDirectory.string();
}


inline void YAMLSceneReader::setYAMLReader(YAMLReader* reader)
{
    impl->mainYamlReader = reader;
}


inline void YAMLSceneReader::clear()
{
    isDegreeMode_ = true;
    impl->defaultMaterial = 0;
    impl->resourceInfoMap.clear();
    impl->imagePathToSgImageMap.clear();
}


inline void YAMLSceneReader::readHeader(Mapping& info)
{
    auto angleUnitNode = info.extract("angleUnit");
    if(angleUnitNode){
        std::string unit = angleUnitNode->toString();
        if(unit == "radian"){
            setAngleUnit(RADIAN);
        } else if(unit == "degree"){
            setAngleUnit(DEGREE);
        } else {
            angleUnitNode->throwException(_("The \"angleUnit\" value must be either \"radian\" or \"degree\""));
        }
    }
}


inline void YAMLSceneReader::setAngleUnit(AngleUnit unit)
{
    isDegreeMode_ = (unit == DEGREE);
}


inline bool YAMLSceneReader::readAngle(const Mapping& info, const char* key, double& angle) const
{
    if(info.read(key, angle)){
        angle = toRadian(angle);
        return true;
    }
    return false;
}


inline bool YAMLSceneReader::readAngle(const Mapping& info, const char* key, float& angle) const
{
    if(info.read(key, angle)){
        angle = toRadian(angle);
        return true;
    }
    return false;
}


inline AngleAxis YAMLSceneReader::readAngleAxis(const Listing& rotation) const
{
    Vector4 r;
    cnoid::read(rotation, r);
    Vector3 axis(r[0], r[1], r[2]);
    double size = axis.norm();
    if(size < 1.0e-6){
        rotation.throwException("Rotation axis is the zero vector");
    }
    axis /= size; // normalize
    return AngleAxis(toRadian(r[3]), axis);
}


inline bool YAMLSceneReader::readRotation(const ValueNode* info, Matrix3& out_R) const
{
    if(!info || !info->isValid()){
        return false;
    }
    const Listing& rotations = *info->toListing();
    if(!rotations.empty()){
        if(!rotations[0].isListing()){
            out_R = readAngleAxis(rotations);
        } else {
            out_R = Matrix3::Identity();
            for(int i=0; i < rotations.size(); ++i){
                out_R = out_R * readAngleAxis(*rotations[i].toListing());
            }
        }
    }
    return true;
}

        
inline bool YAMLSceneReader::readRotation(const Mapping& info, Matrix3& out_R) const
{
    return readRotation(info.find("rotation"), out_R);
}


inline bool YAMLSceneReader::readRotation(const Mapping& info, const char* key, Matrix3& out_R) const
{
    return readRotation(info.find(key), out_R);
}


inline bool YAMLSceneReader::extractRotation(Mapping& info, Matrix3& out_R) const
{
    ValueNodePtr value = info.extract("rotation");
    return readRotation(value, out_R);
}


inline bool YAMLSceneReader::readTranslation(const ValueNode* info, Vector3& out_p) const
{
    if(!info || !info->isValid()){
        return false;
    }
    const Listing& translations = *info->toListing();
    if(!translations.empty()){
        if(!translations[0].isListing()){
            read(translations, out_p);
        } else {
            out_p.setZero();
            Vector3 v;
            for(int i=0; i < translations.size(); ++i){
                if(readTranslation(&translations[i], v)){
                    out_p += v;
                }
            }
        }
    }
    return true;
}


inline bool YAMLSceneReader::readTranslation(const Mapping& info, Vector3& out_p) const
{
    return readTranslation(info.find("translation"), out_p);
}


inline bool YAMLSceneReader::readTranslation(const Mapping& info, const char* key, Vector3& out_p) const
{
    return readTranslation(info.find(key), out_p);
}


inline bool YAMLSceneReader::extractTranslation(Mapping& info, Vector3& out_p) const
{
    ValueNodePtr value = info.extract("translation");
    return readTranslation(value, out_p);
}


inline SgNode* YAMLSceneReader::readNode(Mapping& info)
{
    return impl->readNode(info);
}


inline SgNode* YAMLSceneReader::readNode(Mapping& info, const std::string& type)
{
    return impl->readNode(info, type);
}

namespace detail::yaml_scene_reader {

inline SgNodePtr removeRedundantGroup(SgGroupPtr& group)
{
    SgNodePtr node;
    if(!group->empty()){
        if(group->numChildren() == 1 && typeid(group) == typeid(SgGroup)){
            node = group->child(0);
        } else {
            node = group;
        }
    }
    group.reset();
    return node;
}

}   // namespace detail::yaml_scene_reader

inline SgNode* YAMLSceneReader::readNodeList(ValueNode& info)
{
    SgGroupPtr group = new SgGroup;
    impl->readNodeList(info, group);
    return detail::yaml_scene_reader::removeRedundantGroup(group).retn();
}    


inline SgNode* YAMLSceneReaderImpl::readNode(Mapping& info)
{
    const std::string type = info["type"].toString();
    return readNode(info, type);
}
    

inline SgNode* YAMLSceneReaderImpl::readNode(Mapping& info, const std::string& type)
{
    detail::yaml_scene_reader::NodeFunctionMap::iterator q = detail::yaml_scene_reader::nodeFunctionMap.find(type);
    if(q == detail::yaml_scene_reader::nodeFunctionMap.end()){
        if(info.get("isOptional", false)){
#if UCNOID_NOT_SUPPORTED
            os() << format(_("Warning: the node type \"{}\" is not defined. Reading this node has been skipped."), type) << std::endl;
#else   // UCNOID_NOT_SUPPORTED
            os() << ssformat("Warning: the node type \"", type, "\" is not defined. Reading this node has been skipped.") << std::endl;
#endif  // UCNOID_NOT_SUPPORTED
            return nullptr;
        }
#if UCNOID_NOT_SUPPORTED
        info.throwException(format(_("The node type \"{}\" is not defined."), type));
#else   // UCNOID_NOT_SUPPORTED
        info.throwException(ssformat("The node type \"", type, "\" is not defined."));
#endif  // UCNOID_NOT_SUPPORTED
    }

    detail::yaml_scene_reader::NodeFunction funcToReadNode = q->second;
    SgNodePtr scene = (this->*funcToReadNode)(info);
    if(scene){
        if(info.read("name", symbol)){
            scene->setName(symbol);
        } else {
            // remove a nameless, redundant group node
            if(SgGroupPtr group = dynamic_pointer_cast<SgGroup>(scene)){
                scene = detail::yaml_scene_reader::removeRedundantGroup(group);
            }
        }
    }
    return scene.retn();
}


inline SgNode* YAMLSceneReaderImpl::readNodeNode(Mapping& info)
{
    SgNodePtr node = new SgNode;
    return node.retn();
}


inline SgNode* YAMLSceneReaderImpl::readGroup(Mapping& info)
{
    SgGroupPtr group = new SgGroup;
    readElements(info, group);
    return group.retn();
}


inline void YAMLSceneReaderImpl::readElements(Mapping& info, SgGroup* group)
{
    ValueNode& elements = *info.find("elements");
    if(elements.isValid()){
        readNodeList(elements, group);
    }
}


inline void YAMLSceneReaderImpl::readNodeList(ValueNode& elements, SgGroup* group)
{
    if(elements.isListing()){
        Listing& listing = *elements.toListing();
        for(int i=0; i < listing.size(); ++i){
            Mapping& element = *listing[i].toMapping();
            const std::string type = element["type"].toString();
            SgNodePtr scene = readNode(element, type);
            if(scene){
                group->addChild(scene);
            }
        }
    } else if(elements.isMapping()){
        Mapping& mapping = *elements.toMapping();
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
            SgNodePtr scene = readNode(element, type);
            if(scene){
                group->addChild(scene);
            }
            ++p;
        }
    }
}


inline SgNode* YAMLSceneReaderImpl::readTransform(Mapping& info)
{
    SgGroupPtr group;

    SgPosTransformPtr posTransform = new SgPosTransform;
    if(self->readTranslation(info, v)){
        posTransform->setTranslation(v);
        group = posTransform;
    }        
    Matrix3 R;
    if(self->readRotation(info, R)){
        posTransform->setRotation(R);
        if(!group){
            group = posTransform;
        }
    }

    if(!read(info, "scale", v)){
        if(!group){
            group = new SgGroup;
        }
        readElements(info, group);

    } else {
        SgScaleTransformPtr scale = new SgScaleTransform;
        scale->setScale(v);
        if(group){
            group->addChild(scale);
        } else {
            group = scale;
        }
        readElements(info, scale);
    }

    // Necessary to prevent group from being deleted when exiting the function
    posTransform.reset();
    
    return group.retn();
}


inline SgNode* YAMLSceneReaderImpl::readTransformParameters(Mapping& info, SgNode* scene)
{
    Matrix3 R;
    bool isRotated = self->readRotation(info, R);
    Vector3 p;
    bool isTranslated = self->readTranslation(info, p);
    if(isRotated || isTranslated){
        SgPosTransform* transform = new SgPosTransform;
        if(isRotated){
            transform->setRotation(R);
        }
        if(isTranslated){
            transform->setTranslation(p);
        }
        transform->addChild(scene);
        return transform;
    }
    return scene;
}


inline SgNode* YAMLSceneReaderImpl::readShape(Mapping& info)
{
    SgNode* scene = 0;

    Mapping& geometry = *info.findMapping("geometry");
    if(geometry.isValid()){
        SgShapePtr shape = new SgShape;

        Mapping& appearance = *info.findMapping("appearance");
        if(appearance.isValid()){
            readAppearance(shape, appearance);
        } else {
            setDefaultMaterial(shape);
        }

        if(shape->texture()){
            generateTexCoord = true;
        }else{
            generateTexCoord = false;
        }
        shape->setMesh(readGeometry(geometry));

        scene = readTransformParameters(info, shape);

        if(scene == shape){
            return shape.retn();
        }
    }

    return scene;
}


inline SgMesh* YAMLSceneReaderImpl::readGeometry(Mapping& info)
{
    SgMesh* mesh = 0;
    ValueNode& typeNode = info["type"];
    std::string type = typeNode.toString();
    if(type == "Box"){
        mesh = readBox(info);
    } else if(type == "Sphere"){
        mesh = readSphere(info);
    } else if(type == "Cylinder"){
        mesh = readCylinder(info);
    } else if(type == "Cone"){
        mesh = readCone(info);
    } else if(type == "Capsule"){
        mesh = readCapsule(info);
    } else if(type == "Extrusion"){
        mesh = readExtrusion(info);
    } else if(type == "ElevationGrid"){
        mesh = readElevationGrid(info);
    } else if(type == "IndexedFaceSet"){
        mesh = readIndexedFaceSet(info);
    } else if(type == "Resource"){
        mesh = readResourceAsGeometry(info);
    } else {
#if UCNOID_NOT_SUPPORTED
        typeNode.throwException(
            format(_("Unknown geometry \"{}\""), type));
#else   // UCNOID_NOT_SUPPORTED
        typeNode.throwException(
            ssformat("Unknown geometry \"", type, "\""));
#endif  // UCNOID_NOT_SUPPORTED
    }
    return mesh;
}


inline SgMesh* YAMLSceneReaderImpl::readBox(Mapping& info)
{
    Vector3 size;
    if(!read(info, "size", size)){
        size.setOnes(1.0);
    }
    return meshGenerator.generateBox(size, generateTexCoord);
}


inline SgMesh* YAMLSceneReaderImpl::readSphere(Mapping& info)
{
    return meshGenerator.generateSphere(info.get("radius", 1.0), generateTexCoord);
}


inline SgMesh* YAMLSceneReaderImpl::readCylinder(Mapping& info)
{
    double radius = info.get("radius", 1.0);
    double height = info.get("height", 1.0);
    bool bottom = info.get("bottom", true);
    bool top = info.get("top", true);
    return meshGenerator.generateCylinder(radius, height, bottom, top, true, generateTexCoord);

}


inline SgMesh* YAMLSceneReaderImpl::readCone(Mapping& info)
{
    double radius = info.get("radius", 1.0);
    double height = info.get("height", 1.0);
    bool bottom = info.get("bottom", true);
    return meshGenerator.generateCone(radius, height, bottom, true, generateTexCoord);
}


inline SgMesh* YAMLSceneReaderImpl::readCapsule(Mapping& info)
{
    double radius = info.get("radius", 1.0);
    double height = info.get("height", 1.0);
    return meshGenerator.generateCapsule(radius, height);
}


inline SgMesh* YAMLSceneReaderImpl::readExtrusion(Mapping& info)
{
    MeshGenerator::Extrusion extrusion;

    Listing& crossSectionNode = *info.findListing("crossSection");
    if(crossSectionNode.isValid()){
        const int n = crossSectionNode.size() / 2;
        MeshGenerator::Vector2Array& crossSection = extrusion.crossSection;
        crossSection.resize(n);
        for(int i=0; i < n; ++i){
            Vector2& s = crossSection[i];
            for(int j=0; j < 2; ++j){
                s[j] = crossSectionNode[i*2+j].toDouble();
            }
        }
    }

    Listing& spineNode = *info.findListing("spine");
    if(spineNode.isValid()){
        const int n = spineNode.size() / 3;
        MeshGenerator::Vector3Array& spine = extrusion.spine;
        spine.resize(n);
        for(int i=0; i < n; ++i){
            Vector3& s = spine[i];
            for(int j=0; j < 3; ++j){
                s[j] = spineNode[i*3+j].toDouble();
            }
        }
    }

    Listing& orientationNode = *info.findListing("orientation");
    if(orientationNode.isValid()){
        const int n = orientationNode.size() / 4;
        MeshGenerator::AngleAxisArray& orientation = extrusion.orientation;
        orientation.resize(n);
        for(int i=0; i < n; ++i){
            AngleAxis& aa = orientation[i];
            Vector3& axis = aa.axis();
            for(int j=0; j < 4; ++j){
                axis[j] = orientationNode[i*4+j].toDouble();
            }
            aa.angle() = self->toRadian(orientationNode[i*4+3].toDouble());
        }
    }
    
    Listing& scaleNode = *info.findListing("scale");
    if(scaleNode.isValid()){
        const int n = scaleNode.size() / 2;
        MeshGenerator::Vector2Array& scale = extrusion.scale;
        scale.resize(n);
        for(int i=0; i < n; ++i){
            Vector2& s = scale[i];
            for(int j=0; j < 2; ++j){
                s[j] = scaleNode[i*2+j].toDouble();
            }
        }
    }

    self->readAngle(info, "creaseAngle", extrusion.creaseAngle);
    info.read("beginCap", extrusion.beginCap);
    info.read("endCap", extrusion.endCap);

    SgMesh* mesh = meshGenerator.generateExtrusion(extrusion, generateTexCoord);

    mesh->setSolid(info.get("solid", mesh->isSolid()));
    
    return mesh;
}


inline SgMesh* YAMLSceneReaderImpl::readElevationGrid(Mapping& info)
{
    MeshGenerator::ElevationGrid grid;

    info.read("xDimension", grid.xDimension);
    info.read("zDimension", grid.zDimension);
    info.read("xSpacing", grid.xSpacing);
    info.read("zSpacing", grid.zSpacing);
    info.read("ccw", grid.ccw);
    self->readAngle(info, "creaseAngle", grid.creaseAngle);

    Listing& heightNode = *info.findListing("height");
    if(heightNode.isValid()){
        for(int i=0; i < heightNode.size(); i++){
            grid.height.push_back(heightNode[i].toDouble());
        }
    }

    SgTexCoordArray* texCoord = 0;
    Listing& texCoordNode = *info.findListing("texCoord");
    if(texCoordNode.isValid()){
        const int size = texCoordNode.size() / 2;
        texCoord = new SgTexCoordArray();
        texCoord->resize(size);
        for(int i=0; i < size; ++i){
            Vector2f& s = texCoord->at(i);
            for(int j=0; j < 2; ++j){
                s[j] = texCoordNode[i*2+j].toDouble();
            }
        }
        generateTexCoord = false;
    }

    SgMesh* mesh = meshGenerator.generateElevationGrid(grid, generateTexCoord);
    if(texCoord){
        mesh->setTexCoords(texCoord);
        mesh->texCoordIndices() = mesh->triangleVertices();
    }

    mesh->setSolid(info.get("solid", mesh->isSolid()));

    return mesh;
}


inline SgMesh* YAMLSceneReaderImpl::readIndexedFaceSet(Mapping& info)
{
    SgPolygonMeshPtr polygonMesh = new SgPolygonMesh;

    Listing& coordinateNode = *info.findListing("coordinate");
    if(coordinateNode.isValid()){
        const int size = coordinateNode.size() / 3;
        SgVertexArray& vertices = *polygonMesh->setVertices(new SgVertexArray());
        vertices.resize(size);
        for(int i=0; i < size; ++i){
            Vector3f& s = vertices[i];
            for(int j=0; j < 3; ++j){
                s[j] = coordinateNode[i*3+j].toDouble();
            }
        }
    }

    Listing& coordIndexNode = *info.findListing("coordIndex");
    if(coordIndexNode.isValid()){
        SgIndexArray& polygonVertices = polygonMesh->polygonVertices();
        const int size = coordIndexNode.size();
        polygonVertices.reserve(size);
        for(int i=0; i<size; i++){
            polygonVertices.push_back(coordIndexNode[i].toInt());
        }
    }

    Listing& texCoordNode = *info.findListing("texCoord");
    if(texCoordNode.isValid()){
        const int size = texCoordNode.size() / 2;
        SgTexCoordArray& texCoord = *polygonMesh->setTexCoords(new SgTexCoordArray());
        texCoord.resize(size);
        for(int i=0; i < size; ++i){
            Vector2f& s = texCoord[i];
            for(int j=0; j < 2; ++j){
                s[j] = texCoordNode[i*2+j].toDouble();
            }
        }
    }

    Listing& texCoordIndexNode = *info.findListing("texCoordIndex");
    if(texCoordIndexNode.isValid()){
        SgIndexArray& texCoordIndices = polygonMesh->texCoordIndices();
        const int size = texCoordIndexNode.size();
        texCoordIndices.reserve(size);
        for(int i=0; i<size; i++){
            texCoordIndices.push_back(texCoordIndexNode[i].toInt());
        }
    }

    //polygonMeshTriangulator.setDeepCopyEnabled(true);
    SgMesh* mesh = polygonMeshTriangulator.triangulate(polygonMesh);
    const std::string& errorMessage = polygonMeshTriangulator.errorMessage();
    if(!errorMessage.empty()){
        info.throwException("Error of an IndexedFaceSet node: \n" + errorMessage);
    }

    if(generateTexCoord){
        if(mesh && !mesh->hasTexCoords()){
            meshGenerator.generateTextureCoordinateForIndexedFaceSet(mesh);
        }
    }

    double creaseAngle = 0.0;
    self->readAngle(info, "creaseAngle", creaseAngle);
    meshFilter.generateNormals(mesh, creaseAngle);

    mesh->setSolid(info.get("solid", mesh->isSolid()));

    return mesh;
}


inline SgMesh* YAMLSceneReaderImpl::readResourceAsGeometry(Mapping& info)
{
    SgNode* resource = readResource(info);
    if(resource){
        SgShape* shape = dynamic_cast<SgShape*>(resource);
        if(!shape){
            info.throwException(_("A resouce specified as a geometry must be a single mesh"));
        }
        double creaseAngle;
        if(readAngle(info, "creaseAngle", creaseAngle)){
            meshFilter.setNormalOverwritingEnabled(true);
            bool removeRedundantVertices = info.get("removeRedundantVertices", false);
            meshFilter.generateNormals(shape->mesh(), creaseAngle, removeRedundantVertices);
            meshFilter.setNormalOverwritingEnabled(false);
        }
        if(!generateTexCoord){
            return shape->mesh();
        } else {
            SgMesh* mesh = shape->mesh();
            if(mesh && !mesh->hasTexCoords()){
                meshGenerator.generateTextureCoordinateForIndexedFaceSet(mesh);
            }
            return mesh;
        }
    }
    return 0;
}


inline void YAMLSceneReaderImpl::readAppearance(SgShape* shape, Mapping& info)
{
    Mapping& material = *info.findMapping("material");
    if(material.isValid()){
        readMaterial(shape, material);
    } else {
        setDefaultMaterial(shape);
    }

    Mapping& texture = *info.findMapping("texture");
    if(texture.isValid()){
        readTexture(shape, texture);

        Mapping& textureTransform = *info.findMapping("textureTransform");
        if(textureTransform.isValid() && shape->texture()){
            readTextureTransform(shape->texture(), textureTransform);
        }
    }
}


inline void YAMLSceneReaderImpl::readMaterial(SgShape* shape, Mapping& info)
{
    SgMaterialPtr material = new SgMaterial;

    material->setAmbientIntensity(info.get("ambientIntensity", 0.2));
    if(read(info, "diffuseColor", color)){
        material->setDiffuseColor(color);
    } else {
        material->setDiffuseColor(Vector3f(0.8f, 0.8f, 0.8f));
    }
    if(read(info, "emissiveColor", color)) material->setEmissiveColor(color);
    material->setShininess(info.get("shininess", 0.2));
    if(read(info, "specularColor", color)) material->setSpecularColor(color);
    if(info.read("transparency", value)) material->setTransparency(value);

    shape->setMaterial(material);
}


inline void YAMLSceneReaderImpl::setDefaultMaterial(SgShape* shape)
{
    if(!defaultMaterial){
        defaultMaterial = new SgMaterial;
        defaultMaterial->setDiffuseColor(Vector3f(0.8f, 0.8f, 0.8f));
        defaultMaterial->setAmbientIntensity(0.2f);
        defaultMaterial->setShininess(0.2f);
    }
    shape->setMaterial(defaultMaterial);
}


inline void YAMLSceneReaderImpl::readTexture(SgShape* shape, Mapping& info)
{
    std::string& url = symbol;
    if(info.read("url", url)){
        if(!url.empty()){
            SgImagePtr image=0;
            ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(url);
            if(p != imagePathToSgImageMap.end()){
                image = p->second;
            }else{
                try{
                    image = new SgImage;
                    std::filesystem::path filepath(url);
                    if(!checkAbsolute(filepath)){
                        filepath = baseDirectory / filepath;
#if UCNOID_NOT_SUPPORTED
                        filepath.normalize();
#else   // UCNOID_NOT_SUPPORTED
                        filepath = std::filesystem::canonical(filepath);
#endif  // UCNOID_NOT_SUPPORTED
                    }
                    imageIO.load(image->image(), getAbsolutePathString(filepath));
                    imagePathToSgImageMap[url] = image;
                }catch(const exception_base& ex){
#if UCNOID_NOT_SUPPORTED
                    info.throwException(*boost::get_error_info<error_info_message>(ex));
#else   // UCNOID_NOT_SUPPORTED
                    info.throwException(ex.message());
#endif  // UCNOID_NOT_SUPPORTED
                }
            }
            if(image){
                SgTexturePtr texture = new SgTexture;
                texture->setImage(image);
                bool repeatS = true;
                bool repeatT = true;
                info.read("repeatS", repeatS);
                info.read("repeatT", repeatT);
                texture->setRepeat(repeatS, repeatT);
                texture->setTextureTransform(new SgTextureTransform);
                shape->setTexture(texture);
            }
        }
    }
}


inline void YAMLSceneReaderImpl::readTextureTransform(SgTexture* texture, Mapping& info)
{
    SgTextureTransform* textureTransform = texture->textureTransform();
    if(read(info, "center", v2)) textureTransform->setCenter(v2);
    if(read(info, "scale", v2)) textureTransform->setScale(v2);
    if(read(info, "translation", v2)) textureTransform->setTranslation(v2);
    if(self->readAngle(info, "rotation", value)) textureTransform->setRotation(value);
}


inline void YAMLSceneReaderImpl::readLightCommon(Mapping& info, SgLight* light)
{
    if(info.read("on", on)) light->on(on);
    if(read(info, "color", color)) light->setColor(color);
    if(info.read("intensity", value)) light->setIntensity(value);
    if(info.read("ambientIntensity", value)) light->setAmbientIntensity(value);
}


inline SgNode* YAMLSceneReaderImpl::readDirectionalLight(Mapping& info)
{
    SgDirectionalLightPtr light = new SgDirectionalLight;
    readLightCommon(info, light);
    if(read(info, "direction", v)) light->setDirection(v);
    return light.retn();
}


inline SgNode* YAMLSceneReaderImpl::readSpotLight(Mapping& info)
{
    SgSpotLightPtr light = new SgSpotLight;

    readLightCommon(info, light);

    if(read(info, "direction", v)) light->setDirection(v);
    if(readAngle(info, "beamWidth", value)) light->setBeamWidth(value);
    if(readAngle(info, "cutOffAngle", value)) light->setCutOffAngle(value);
    if(info.read("cutOffExponent", value)) light->setCutOffExponent(value);
    if(read(info, "attenuation", color)){
        light->setConstantAttenuation(color[0]);
        light->setLinearAttenuation(color[1]);
        light->setQuadraticAttenuation(color[2]);
    }
    
    return light.retn();
}


inline SgNode* YAMLSceneReaderImpl::readResource(Mapping& info)
{
    auto resource = readResourceNode(info);
    if(resource.scene){
        return resource.scene;
    } else if(resource.info){
        return readNode(*resource.info->toMapping());
    }
    return nullptr;
}


inline YAMLSceneReader::Resource YAMLSceneReader::readResourceNode(Mapping& info)
{
    return impl->readResourceNode(info);
}


inline YAMLSceneReader::Resource YAMLSceneReaderImpl::readResourceNode(Mapping& info)
{
    std::string uri = info["uri"].toString();

    ValueNode& exclude = *info.find("exclude");
    if(exclude.isValid()){
        if(exclude.isString()){
            decoupleResourceNode(info, uri, exclude.toString());
        } else if(exclude.isListing()){
            Listing& excludes = *exclude.toListing();
            for(auto& nodeToExclude : excludes){
                decoupleResourceNode(info, uri, nodeToExclude->toString());
            }
        } else {
            exclude.throwException(_("The value of \"exclude\" must be string or sequence."));
        }
    }
        
    auto resource = loadResource(info, uri);

    if(resource.scene){
        resource.scene = readTransformParameters(info, resource.scene);
    }

    return resource;
}


inline YAMLSceneReader::Resource YAMLSceneReaderImpl::loadResource(Mapping& resourceNode, const std::string& uri)
{
    std::vector<std::string> names;
    auto node = resourceNode.find("node");
    if(node->isValid()){
        if(node->isString()){
            names.push_back(node->toString());
        } else if(node->isListing()){
            auto nodes = node->toListing();
            for(auto& nodei : *nodes){
                names.push_back(nodei->toString());
            }
        }
    }
    
    YAMLSceneReader::Resource resource;
    detail::yaml_scene_reader::ResourceInfo* resourceInfo = getOrCreateResourceInfo(resourceNode, uri);
    if(resourceInfo){
        resource.directory = resourceInfo->directory;
        bool isYamlResouce = (resourceInfo->yamlReader != nullptr);
        if(names.empty()){
            if(isYamlResouce){
                resource.info = resourceInfo->yamlReader->document();
            } else {
                resource.scene = resourceInfo->scene;
            }
        } else {
            if(isYamlResouce){
                extractNamedYamlNodes(resourceNode, resourceInfo, names, uri, resource);
            } else {
                extractNamedSceneNodes(resourceNode, resourceInfo, names, uri, resource);
            }
        }
    }
    
    return resource;
}


inline void YAMLSceneReaderImpl::extractNamedYamlNodes
(Mapping& resourceNode, detail::yaml_scene_reader::ResourceInfo* info, std::vector<std::string>& names, const std::string& uri, YAMLSceneReader::Resource& resource)
{
    Listing* group = nullptr;
    if(names.size() >= 2){
        group = new Listing;
        resource.info = group;
    }
    for(auto& name : names){
        auto node = info->yamlReader->findAnchoredNode(name);
        if(!node){
#if UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                format(_("Node \"{0}\" is not found in \"{1}\"."), name, uri));
#else   // UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                ssformat("Node \"", name, "\" is not found in \"", uri, "\"."));
#endif  // UCNOID_NOT_SUPPORTED
        }
        if(group){
            group->append(node);
        } else {
            resource.info = node;
        }
    }
}


inline void YAMLSceneReaderImpl::extractNamedSceneNodes
(Mapping& resourceNode, detail::yaml_scene_reader::ResourceInfo* info, std::vector<std::string>& names, const std::string& uri, YAMLSceneReader::Resource& resource)
{
    std::unique_ptr<detail::yaml_scene_reader::SceneNodeMap>& nodeMap = info->sceneNodeMap;
    if(!nodeMap){
        makeSceneNodeMap(info);
    }

    SgGroupPtr group;
    if(names.size() >= 2){
        group = new SgGroup;
        resource.scene = group;
    }

    for(auto& name : names){
        auto iter = nodeMap->find(name);
        if(iter == nodeMap->end()){
#if UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                format(_("Node \"{0}\" is not found in \"{1}\"."), name, uri));
#else   // UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                ssformat("Node \"", name, "\" is not found in \"", uri, "\"."));
#endif  // UCNOID_NOT_SUPPORTED
        } else {
            detail::yaml_scene_reader::SceneNodeInfo& nodeInfo = iter->second;
            if(nodeInfo.parent){
                nodeInfo.parent->removeChild(nodeInfo.node);
                nodeInfo.parent = 0;
                adjustNodeCoordinate(nodeInfo);
            }
            if(group){
                group->addChild(nodeInfo.node);
            } else {
                resource.scene = nodeInfo.node;
            }
        }
    }
}


inline void YAMLSceneReaderImpl::decoupleResourceNode(Mapping& resourceNode, const std::string& uri, const std::string& nodeName)
{
    detail::yaml_scene_reader::ResourceInfo* resourceInfo = getOrCreateResourceInfo(resourceNode, uri);
    if(resourceInfo){
        std::unique_ptr<detail::yaml_scene_reader::SceneNodeMap>& nodeMap = resourceInfo->sceneNodeMap;
        if(!nodeMap){
            makeSceneNodeMap(resourceInfo);
        }
        auto iter = nodeMap->find(nodeName);
        if(iter != nodeMap->end()){
            detail::yaml_scene_reader::SceneNodeInfo& nodeInfo = iter->second;
            if(nodeInfo.parent){
                nodeInfo.parent->removeChild(nodeInfo.node);
                nodeInfo.parent = 0;
                adjustNodeCoordinate(nodeInfo);
            }
        }
    }
}


inline detail::yaml_scene_reader::ResourceInfo* YAMLSceneReaderImpl::getOrCreateResourceInfo(Mapping& resourceNode, const std::string& uri)
{
    auto iter = resourceInfoMap.find(uri);

    if(iter != resourceInfoMap.end()){
        return iter->second;
    }

    std::filesystem::path filepath;
        
    if(!isUriSchemeRegexReady){
        uriSchemeRegex.assign("^(.+)://(.+)$");
        isUriSchemeRegexReady = true;
    }
    std::smatch match;
    bool hasScheme = false;
        
    if(regex_match(uri, match, uriSchemeRegex)){
        hasScheme = true;
        if(match.size() == 3){
            const std::string scheme = match.str(1);
            if(scheme == "file"){
                filepath = match.str(2);
            } else {
#if UCNOID_NOT_SUPPORTED
                std::lock_guard<std::mutex> guard(uriSchemeHandlerMutex);
#endif  // UCNOID_NOT_SUPPORTED
                auto iter = detail::yaml_scene_reader::uriSchemeHandlerMap.find(scheme);
                if(iter == detail::yaml_scene_reader::uriSchemeHandlerMap.end()){
#if UCNOID_NOT_SUPPORTED
                    resourceNode.throwException(
                        format(_("The \"{0}\" scheme of \"{1}\" is not available"), scheme, uri));
#else   // UCNOID_NOT_SUPPORTED
                    resourceNode.throwException(
                        ssformat("The \"", scheme, "\" scheme of \"", uri, "\" is not available"));
#endif  // UCNOID_NOT_SUPPORTED
                } else {
                    auto& handler = iter->second;
                    filepath = handler(match.str(2), os());
                }
            }
        }
    }

    if(!hasScheme){
        filepath = uri;
        if(!checkAbsolute(filepath)){
            filepath = baseDirectory / filepath;
#if UCNOID_NOT_SUPPORTED
            filepath.normalize();
#else   // UCNOID_NOT_SUPPORTED
            filepath = std::filesystem::canonical(filepath);
#endif  // UCNOID_NOT_SUPPORTED
        }
    }
    
    if(filepath.empty()){
#if UCNOID_NOT_SUPPORTED
        resourceNode.throwException(
            format(_("The resource URI \"{}\" is not valid"), ));
#else   // UCNOID_NOT_SUPPORTED
        resourceNode.throwException(
            ssformat("The resource URI \"", uri, "\" is not valid"));
#endif  // UCNOID_NOT_SUPPORTED
    }

    detail::yaml_scene_reader::ResourceInfoPtr info = new detail::yaml_scene_reader::ResourceInfo;

    std::string filename = std::filesystem::absolute(filepath).string();
    std::string ext = std::filesystem::path(filepath).extension();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if(ext == ".yaml" || ext == ".yml"){
        auto reader = new YAMLReader;
        reader->importAnchors(*mainYamlReader);
        if(!reader->load(filename)){
#if UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                format(_("YAML resource \"{0}\" cannot be loaded ({1})"),
                 uri, reader->errorMessage()));
#else   // UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                ssformat("YAML resource \"", uri, "\" cannot be loaded (", reader->errorMessage(), ")"));
#endif  // UCNOID_NOT_SUPPORTED
        }
        info->yamlReader.reset(reader);

    } else {
        SgNodePtr scene = sceneLoader.load(filename);
        if(!scene){
#if UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                format(_("The resource is not found at URI \"{}\""), uri));
#else   // UCNOID_NOT_SUPPORTED
            resourceNode.throwException(
                ssformat("The resource is not found at URI \"", uri, "\""));
#endif  // UCNOID_NOT_SUPPORTED
        }
        info->scene = scene;
    }

    info->directory = filepath.parent_path().string();
    
    resourceInfoMap[uri] = info;

    return info;
}


inline void YAMLSceneReaderImpl::adjustNodeCoordinate(detail::yaml_scene_reader::SceneNodeInfo& info)
{
    if(auto pos = dynamic_cast<SgPosTransform*>(info.node.get())){
        if(info.isScaled){
            auto affine = new SgAffineTransform;
            affine->setLinear(info.R * pos->rotation());
            affine->translation().setZero();
            pos->moveChildrenTo(affine);
            info.node = affine;
        } else {
            pos->setRotation(info.R * pos->rotation());
            pos->translation().setZero();
        }
            
    } else if(auto affine = dynamic_cast<SgAffineTransform*>(info.node.get())){
        affine->setLinear(info.R * affine->linear());
        affine->translation().setZero();
            
    } else {
        if(info.isScaled){
            auto transform = new SgAffineTransform;
            transform->setLinear(info.R);
            transform->translation().setZero();
            transform->addChild(info.node);
            info.node = transform;
        } else if(!info.R.isApprox(Matrix3::Identity())){
            auto transform = new SgPosTransform;
            transform->setRotation(info.R);
            transform->translation().setZero();
            transform->addChild(info.node);
            info.node = transform;
        }
    }
}
        

inline void YAMLSceneReaderImpl::makeSceneNodeMap(detail::yaml_scene_reader::ResourceInfo* info)
{
    info->sceneNodeMap.reset(new detail::yaml_scene_reader::SceneNodeMap);
    detail::yaml_scene_reader::SceneNodeInfo nodeInfo;
    nodeInfo.parent = 0;
    nodeInfo.node = info->scene;
    nodeInfo.R = Matrix3::Identity();
    nodeInfo.isScaled = false;
    makeSceneNodeMapSub(nodeInfo, *info->sceneNodeMap);
}


inline void YAMLSceneReaderImpl::makeSceneNodeMapSub(const detail::yaml_scene_reader::SceneNodeInfo& nodeInfo, detail::yaml_scene_reader::SceneNodeMap& nodeMap)
{
    const std::string& name = nodeInfo.node->name();
    bool wasProcessed = false;
    if(!name.empty()){
        wasProcessed = !nodeMap.insert(make_pair(name, nodeInfo)).second;
    }
    if(!wasProcessed){
        SgGroup* group = dynamic_cast<SgGroup*>(nodeInfo.node.get());
        if(group){
            detail::yaml_scene_reader::SceneNodeInfo childInfo;
            childInfo.parent = group;
            childInfo.isScaled = nodeInfo.isScaled;
            SgTransform* transform = dynamic_cast<SgTransform*>(group);
            if(!transform){
                childInfo.R = nodeInfo.R;
            } else if(auto pos = dynamic_cast<SgPosTransform*>(transform)){
                childInfo.R = nodeInfo.R * pos->rotation();
            } else {
                Affine3 T;
                transform->getTransform(T);
                childInfo.R = nodeInfo.R * T.linear();
                childInfo.isScaled = true;
            }
            for(SgNode* child : *group){
                childInfo.node = child;
                makeSceneNodeMapSub(childInfo, nodeMap);
            }
        }
    }
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_YAML_SCENE_READER_CPP_H
