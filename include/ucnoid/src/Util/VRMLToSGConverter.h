/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_VRML_TO_SG_CONVERTER_H
#define UCNOID_UTIL_VRML_TO_SG_CONVERTER_H

#include "VRML.h"
#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class VRMLToSGConverterImpl;

class UCNOID_EXPORT VRMLToSGConverter
{
public:
    VRMLToSGConverter();
    ~VRMLToSGConverter();

    int divisionNumber() const;

    void setMessageSink(std::ostream& os);
    void setTriangulationEnabled(bool on);
    void setDivisionNumber(int divisionNumber);
    void setNormalGenerationEnabled(bool on, bool doOverwrite = false);
    void setMinCreaseAngle(double angle);
    void setMaxCreaseAngle(double angle);

    void clearConvertedNodeMap();
        
    SgNodePtr convert(VRMLNodePtr vrmlNode);

private:
    VRMLToSGConverterImpl* impl;
};
    
}   // inline namespace ucnoid
}

#include "VRMLToSGConverter.cpp.h"

#endif
