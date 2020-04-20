/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_VRML_BODY_LOADER_H
#define UCNOID_BODY_VRML_BODY_LOADER_H

#include "AbstractBodyLoader.h"
#include <ucnoid/VRML>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class Link;
class VRMLBodyLoaderImpl;
  
class UCNOID_EXPORT VRMLBodyLoader : public AbstractBodyLoader
{
public:
    VRMLBodyLoader();
    ~VRMLBodyLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void enableShapeLoading(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual bool load(Body* body, const std::string& filename);
    VRMLNodePtr getOriginalNode(Link* link);

private:
    VRMLBodyLoaderImpl* impl;
};

}   // inline namespace ucnoid
}

#include "VRMLBodyLoader.cpp.h"

#endif
