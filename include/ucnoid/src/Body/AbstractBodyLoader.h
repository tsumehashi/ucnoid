/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_ABSTRACT_BODY_LOADER_H
#define UCNOID_BODY_ABSTRACT_BODY_LOADER_H

#include <string>
#include <memory>
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class Body;

class UCNOID_EXPORT AbstractBodyLoader
{
public:
    AbstractBodyLoader();
    virtual ~AbstractBodyLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void setShapeLoadingEnabled(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);
    virtual bool load(Body* body, const std::string& filename) = 0;
};

typedef std::shared_ptr<AbstractBodyLoader> AbstractBodyLoaderPtr;

}    // inline namespace ucnoid
}

#include "AbstractBodyLoader.cpp.h"

#endif
