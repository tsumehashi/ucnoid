/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_VRML_CPP_H
#define UCNOID_UTIL_VRML_CPP_H

#include "VRML.h"

namespace cnoid {
inline namespace ucnoid {

inline const char* labelOfVRMLfieldTypeId(const std::type_info& fieldType)
{
    if(fieldType == typeid(SFInt32)){
        return "SFInt32";
    } else if(fieldType == typeid(MFInt32)){
        return "MFInt32";
    } else if(fieldType == typeid(SFFloat)){
        return "SFFloat";
    } else if(fieldType == typeid(MFFloat)){
        return "MFFloat";
    } else if(fieldType == typeid(SFVec2f)){
        return "SFVec3f";
    } else if(fieldType == typeid(MFVec2f)){
        return "MFVec2f";
    } else if(fieldType == typeid(SFVec3f)){
        return "SFVec3f";
    } else if(fieldType == typeid(MFVec3f)){
        return "MFVec3f";
    } else if(fieldType == typeid(SFRotation)){
        return "SFRotation";
    } else if(fieldType == typeid(MFRotation)){
        return "MFRotation";
    } else if(fieldType == typeid(SFTime)){
        return "SFTime";
    } else if(fieldType == typeid(MFTime)){
        return "MFTime";
    } else if(fieldType == typeid(SFColor)){
        return "SFColor";
    } else if(fieldType == typeid(MFColor)){
        return "MFColor";
    } else if(fieldType == typeid(SFString)){
        return "SFString";
    } else if(fieldType == typeid(MFString)){
        return "MFString";
    } else if(fieldType == typeid(SFNode)){
        return "SFNode";
    } else if(fieldType == typeid(MFNode)){
        return "MFNode";
    } else if(fieldType == typeid(SFBool)){
        return "SFBool";
    } else if(fieldType == typeid(SFImage)){
        return "SFImage";
    } else {
        return "Unknown Field Type";
    }
}


inline VRMLNode::VRMLNode()
{

}


inline VRMLNode::~VRMLNode()
{

}


inline bool VRMLNode::isCategoryOf(VRMLNodeCategory category)
{
    return (category == ANY_NODE) ? true : categorySet.test(category);
}



inline VRMLUnsupportedNode::VRMLUnsupportedNode(const std::string& nodeTypeName) :
    nodeTypeName(nodeTypeName)
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
}


inline const char* VRMLUnsupportedNode::typeName() const
{
    return "Unknown";
}


inline VRMLViewpoint::VRMLViewpoint()
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);
    
    fieldOfView = 0.785398;
    jump = true;
    orientation.axis() = SFVec3f::UnitZ();
    orientation.angle() = 0.0;
    position << 0.0, 0.0, 10.0;
}


inline const char* VRMLViewpoint::typeName() const
{
    return "Viewpoint";
}


inline VRMLNavigationInfo::VRMLNavigationInfo() :
    avatarSize(3)
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);

    avatarSize[0] = 0.25;
    avatarSize[1] = 1.6;
    avatarSize[2] = 0.75;

    headlight = true;
    speed = 1.0;
    visibilityLimit = 0.0;

    type.push_back("WALK");
}


inline const char* VRMLNavigationInfo::typeName() const
{
    return "NavigationInfo";
}


inline VRMLBackground::VRMLBackground()
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);
}


inline const char* VRMLBackground::typeName() const
{
    return "Background";
}


inline AbstractVRMLGroup::AbstractVRMLGroup()
{
    categorySet.set(TOP_NODE);
    categorySet.set(GROUPING_NODE);
    categorySet.set(CHILD_NODE);
}


inline void AbstractVRMLGroup::removeChild(int childIndex)
{
    replaceChild(childIndex, 0);
}


inline VRMLGroup::VRMLGroup()
{
    bboxCenter.setZero();
    bboxSize.fill(-1.0);
}


inline const char* VRMLGroup::typeName() const
{
    return "Group";
}


inline MFNode& VRMLGroup::getChildren()
{
    return children;
}


inline int VRMLGroup::countChildren()
{
    return children.size();
}


inline VRMLNode* VRMLGroup::getChild(int index)
{
    return children[index].get();
}


inline void VRMLGroup::replaceChild(int childIndex, VRMLNode* childNode)
{
    if(!childNode){
        children.erase(children.begin() + childIndex);
    } else {
        children[childIndex] = childNode;
    }
}


inline VRMLTransform::VRMLTransform()
{
    center.setZero();
    rotation.axis() = SFVec3f::UnitZ();
    rotation.angle() = 0.0;
    scale.setOnes();
    scaleOrientation.axis() = SFVec3f::UnitZ();
    scaleOrientation.angle() = 0.0;
    translation.setZero();
}


inline const char* VRMLTransform::typeName() const
{
    return "Transform";
}


inline Eigen::Affine3d VRMLTransform::toAffine3d()
{
    const Eigen::Translation3d C(center);
    const SFRotation& SR = scaleOrientation;
    const Eigen::AlignedScaling3d S(scale);
    const SFRotation& R = rotation;
    const Eigen::Translation3d T(translation);

    return T * C * R * SR * S * SR.inverse() * C.inverse();
}


inline VRMLInline::VRMLInline()
{
    categorySet.set(INLINE_NODE);
}


inline const char* VRMLInline::typeName() const
{
    return "Inline";
}


inline VRMLNonVrmlInline::VRMLNonVrmlInline()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
}


inline const char* VRMLNonVrmlInline::typeName() const
{
    return "NonVRML";
}


inline VRMLShape::VRMLShape()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
    categorySet.set(SHAPE_NODE);
}


inline const char* VRMLShape::typeName() const
{
    return "Shape";
}


inline VRMLAppearance::VRMLAppearance()
{
    categorySet.set(APPEARANCE_NODE);
}


inline const char* VRMLAppearance::typeName() const
{
    return "Appearance";
}


inline VRMLMaterial::VRMLMaterial()
{
    categorySet.set(MATERIAL_NODE);

    diffuseColor << 0.8f, 0.8f, 0.8f;
    emissiveColor.setZero();
    specularColor.setZero();
    ambientIntensity = 0.2;
    shininess = 0.2;
    transparency = 0.0;
}


inline const char* VRMLMaterial::typeName() const
{
    return "Material";
}


inline VRMLTexture::VRMLTexture()
{
    categorySet.set(TEXTURE_NODE);
}


inline VRMLImageTexture::VRMLImageTexture()
{
    repeatS = true; 
    repeatT = true; 
}


inline const char* VRMLImageTexture::typeName() const
{
    return "ImageTexture";
}


inline VRMLTextureTransform::VRMLTextureTransform()
{
    categorySet.set(TEXTURE_TRANSFORM_NODE);

    center.setZero();
    scale.setOnes();
    translation.setZero();
    rotation = 0.0;
}


inline const char* VRMLTextureTransform::typeName() const
{
    return "TextureTransform";
}


inline VRMLGeometry::VRMLGeometry()
{
    categorySet.set(GEOMETRY_NODE);
}


inline VRMLBox::VRMLBox()
{
    size.fill(2.0);
}


inline const char* VRMLBox::typeName() const
{
    return "Box";
}


inline VRMLCone::VRMLCone()
{
    bottom = true;
    bottomRadius = 1.0;
    height = 2.0;
    side = true;
}


inline const char* VRMLCone::typeName() const
{
    return "Cone";
}


inline VRMLCylinder::VRMLCylinder()
{
    height = 2.0;
    radius = 1.0;
    bottom = true;
    side = true;
    top = true;
}


inline const char* VRMLCylinder::typeName() const
{
    return "Cylinder";
}


inline VRMLSphere::VRMLSphere()
{
    radius = 1.0; 
}


inline const char* VRMLSphere::typeName() const
{
    return "Sphere";
}


inline VRMLFontStyle::VRMLFontStyle()
{
    categorySet.set(FONT_STYLE_NODE);
  
    family.push_back("SERIF");
    horizontal = true;
    justify.push_back("BEGIN");
    leftToRight = true;
    size = 1.0;
    spacing = 1.0;
    style = "PLAIN";
    topToBottom = true;
}


inline const char* VRMLFontStyle::typeName() const
{
    return "FontStyle";
}


inline VRMLText::VRMLText()
{
    maxExtent = 0.0;
}


inline const char* VRMLText::typeName() const
{
    return "Text";
}


inline VRMLIndexedLineSet::VRMLIndexedLineSet()
{
    colorPerVertex = true;
}


inline const char* VRMLIndexedLineSet::typeName() const
{
    return "IndexedLineSet";
}


inline VRMLIndexedFaceSet::VRMLIndexedFaceSet()
{
    ccw = true;
    convex = true;
    creaseAngle = 0.0;
    normalPerVertex = true;
    solid = true;
}


inline const char* VRMLIndexedFaceSet::typeName() const
{
    return "IndexedFaseSet";
}


inline VRMLColor::VRMLColor()
{
    categorySet.set(COLOR_NODE);
}


inline const char* VRMLColor::typeName() const
{
    return "Color";
}


inline VRMLCoordinate::VRMLCoordinate()
{
    categorySet.set(COORDINATE_NODE);
}


inline const char* VRMLCoordinate::typeName() const
{
    return "Coordinate";
}


inline VRMLTextureCoordinate::VRMLTextureCoordinate()
{
    categorySet.set(TEXTURE_COORDINATE_NODE);
}


inline const char* VRMLTextureCoordinate::typeName() const
{
    return "TextureCoordinate";
}


inline VRMLNormal::VRMLNormal()
{
    categorySet.set(NORMAL_NODE);
}


inline const char* VRMLNormal::typeName() const
{
    return "Normal";
}


inline VRMLCylinderSensor::VRMLCylinderSensor()
{
    categorySet.set(CHILD_NODE);
    categorySet.set(SENSOR_NODE);
  
    autoOffset = true;
    diskAngle = 0.262;
    enabled = true;
    maxAngle = -1;
    minAngle = 0;
    offset = 0;
}


inline const char* VRMLCylinderSensor::typeName() const
{
    return "CylinderSensor";
}


inline VRMLPointSet::VRMLPointSet()
{
    coord = NULL;
    color = NULL;
}


inline const char* VRMLPointSet::typeName() const
{
    return "PointSet";
}


inline VRMLPixelTexture::VRMLPixelTexture()
{
    image.width  = 0;
    image.height = 0;
    image.numComponents = 0;
    image.pixels.clear();
	
    repeatS = true;
    repeatT = true;
}


inline const char* VRMLPixelTexture::typeName() const
{
    return "PixelTexture";
}


inline VRMLMovieTexture::VRMLMovieTexture()
{
    // url
    loop = false;
    speed = 0;
    startTime = 0.0;
    stopTime = 0.0;
    repeatS = true;
    repeatT = true;
}


inline const char* VRMLMovieTexture::typeName() const
{
    return "MovieTexture";
}


inline VRMLElevationGrid::VRMLElevationGrid()
{
    xDimension = 0;
    zDimension = 0;
    xSpacing = 0.0;
    zSpacing = 0.0;
    // height	// MFFloat
    ccw = true;
    colorPerVertex = true;
    creaseAngle = 0.0;
    normalPerVertex = true;
    solid = true;
    color = NULL;
    normal = NULL;
    texCoord = NULL;
}


inline const char* VRMLElevationGrid::typeName() const
{
    return "ElevationGrid";
}


inline VRMLExtrusion::VRMLExtrusion()
{
    // crossSection
    // spine
    beginCap = true;
    endCap   = true;
    solid    = true;
    ccw      = true;
    convex   = true;
    creaseAngle = 0;
    orientation.push_back(SFRotation(0.0, SFVec3f::UnitZ()));
    scale.push_back(SFVec2f(1.0, 1.0));
}


inline const char* VRMLExtrusion::typeName() const
{
    return "Extrusion";
}


inline VRMLSwitch::VRMLSwitch()
{
    whichChoice = -1;
}


inline const char* VRMLSwitch::typeName() const
{
    return "Switch";
}


inline MFNode& VRMLSwitch::getChildren()
{
    return choice;
}


inline int VRMLSwitch::countChildren()
{
    return choice.size();
}


inline VRMLNode* VRMLSwitch::getChild(int index)
{
    return choice[index].get();
}


inline void VRMLSwitch::replaceChild(int childIndex, VRMLNode* childNode)
{
    if(!childNode){
        choice.erase(choice.begin() + childIndex);
        if(whichChoice == childIndex){
            whichChoice = -1;
        } else if(whichChoice > childIndex){
            whichChoice -= 1;
        }
    } else {
        choice[childIndex] = childNode;
    }
}


inline VRMLLOD::VRMLLOD()
{
    center.setZero();
}


inline const char* VRMLLOD::typeName() const
{
    return "LOD";
}


inline MFNode& VRMLLOD::getChildren()
{
    return level;
}


inline int VRMLLOD::countChildren()
{
    return level.size();
}


inline VRMLNode* VRMLLOD::getChild(int index)
{
    return level[index].get();
}


inline void VRMLLOD::replaceChild(int childIndex, VRMLNode* childNode)
{
    if(!childNode){
        level.erase(level.begin() + childIndex);
        if(!level.empty()){
            int rangeIndexToRemove = (childIndex > 0) ? (childIndex - 1) : 0;
            range.erase(range.begin() + rangeIndexToRemove);
        }
    } else {
        level[childIndex] = childNode;
    }
}


inline VRMLCollision::VRMLCollision()
{
    collide = true;
}


inline const char* VRMLCollision::typeName() const
{
    return "Collision";
}


inline VRMLAnchor::VRMLAnchor()
{

}


inline const char* VRMLAnchor::typeName() const
{
    return "Anchor";
}


inline VRMLBillboard::VRMLBillboard()
{
    axisOfRotation.setZero();
}


inline const char* VRMLBillboard::typeName() const
{
    return "Billboard";
}


inline VRMLFog::VRMLFog()
{
    categorySet.set(TOP_NODE);    
    categorySet.set(CHILD_NODE);
    color.setZero();
    visibilityRange = 0.0;
    fogType = "LINEAR";
}


inline const char* VRMLFog::typeName() const
{
    return "Fog";
}


inline VRMLWorldInfo::VRMLWorldInfo()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
}


inline const char* VRMLWorldInfo::typeName() const
{
    return "WorldInfo";
}


inline VRMLLight::VRMLLight()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
    categorySet.set(LIGHT_NODE);

    on = true;
    color.setOnes();
    intensity = 1.0;
    ambientIntensity = 0.0;
}


inline VRMLPointLight::VRMLPointLight()
{
    location.setZero();
    radius = 100.0;
    attenuation << 1.0, 0.0, 0.0;
}


inline const char* VRMLPointLight::typeName() const
{
    return "PointLight";
}


inline VRMLDirectionalLight::VRMLDirectionalLight()
{
    direction << 0.0, 0.0, -1.0;
}


inline const char* VRMLDirectionalLight::typeName() const
{
    return "DirectionalLight";
}


inline VRMLSpotLight::VRMLSpotLight()
{
    direction << 0.0, 0.0, -1.0;
    beamWidth = 1.570796;
    cutOffAngle = 0.785398;
}


inline const char* VRMLSpotLight::typeName() const
{
    return "SpotLight";
}


inline VRMLProto::VRMLProto(const std::string& n) : protoName(n)
{
    categorySet.set(TOP_NODE);
    categorySet.set(PROTO_DEF_NODE);
}


inline const char* VRMLProto::typeName() const
{
    return "Proto";
}


inline VRMLProtoInstance::VRMLProtoInstance(VRMLProtoPtr proto0) :
    proto(proto0),
    fields(proto0->fields) 
{
    categorySet.set(TOP_NODE);
    categorySet.set(PROTO_INSTANCE_NODE);;
    categorySet.set(CHILD_NODE);
}


inline const char* VRMLProtoInstance::typeName() const
{
    return "ProtoInstance";
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_VRML_CPP_H
