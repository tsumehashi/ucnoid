/**
 @author Shin'ichiro Nakaoka
*/
#ifndef UCNOID_UTIL_ABSTRACT_SCENE_LOADER_H
#define UCNOID_UTIL_ABSTRACT_SCENE_LOADER_H

#include "SceneGraph.h"
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class UCNOID_EXPORT AbstractSceneLoader
{
public:
    virtual ~AbstractSceneLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);
    virtual SgNode* load(const std::string& filename) = 0;
};

}   // inline namespace ucnoid
}

#include "AbstractSceneLoader.cpp.h"

#endif
