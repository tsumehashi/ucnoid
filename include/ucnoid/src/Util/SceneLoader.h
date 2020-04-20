/**
 @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_SCENE_LOADER_H
#define UCNOID_UTIL_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class SceneLoaderImpl;

class UCNOID_EXPORT SceneLoader : public AbstractSceneLoader
{
public:
    //! @param extensions semi-colon separated extension list
    static void registerLoader(const char* extensions, std::function<std::shared_ptr<AbstractSceneLoader>()> factory);

    //! This function returns a semi-colon separated list of availabe file extensions.
    static std::string availableFileExtensions();

    SceneLoader();
    virtual ~SceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual void setDefaultCreaseAngle(double theta) override;
    virtual SgNode* load(const std::string& filename) override;

private:
    SceneLoaderImpl* impl;
};

}   // inline namespace ucnoid
}

#include "SceneLoader.cpp.h"

#endif
