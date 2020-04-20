/**
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_ABSTRACT_SCENE_LOADER_CPP_H
#define UCNOID_UTIL_ABSTRACT_SCENE_LOADER_CPP_H

#include "AbstractSceneLoader.h"

namespace cnoid {
inline namespace ucnoid {

AbstractSceneLoader::~AbstractSceneLoader()
{

}


void AbstractSceneLoader::setMessageSink(std::ostream& /* os */)
{

}


void AbstractSceneLoader::setDefaultDivisionNumber(int /* n */)
{

}


void AbstractSceneLoader::setDefaultCreaseAngle(double /* theta */)
{

}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_ABSTRACT_SCENE_LOADER_CPP_H
