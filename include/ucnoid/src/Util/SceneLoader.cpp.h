/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_SCENE_LOADER_CPP_H
#define UCNOID_UTIL_SCENE_LOADER_CPP_H

#include "SceneLoader.h"
#include "NullOut.h"
#include "FileUtil.h"
#if UCNOID_NOT_SUPPORTED
#include <fmt/format.h>
#endif  // UCNOID_NOT_SUPPORTED
#include <mutex>
#include <map>
#include <algorithm>
#include "gettext.h"

namespace cnoid {
inline namespace ucnoid {

namespace detail::scene_loader {

typedef std::shared_ptr<AbstractSceneLoader> AbstractSceneLoaderPtr;
typedef std::function<AbstractSceneLoaderPtr()> LoaderFactory;
typedef std::map<std::string, int> LoaderIdMap;
inline LoaderIdMap loaderIdMap;
inline std::vector<LoaderFactory> loaderFactories;
inline std::mutex loaderMutex;

}   // namespace detail::scene_loader

class SceneLoaderImpl
{
public:
    std::ostream* os_;
    std::ostream& os() { return *os_; }
    typedef std::map<int, detail::scene_loader::AbstractSceneLoaderPtr> LoaderMap;
    LoaderMap loaders;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    SceneLoaderImpl();
    detail::scene_loader::AbstractSceneLoaderPtr findLoader(std::string ext);
    SgNode* load(const std::string& filename);
};


inline void SceneLoader::registerLoader(const char* extensions, std::function<detail::scene_loader::AbstractSceneLoaderPtr()> factory)
{
    using namespace detail::scene_loader;
    std::vector<std::string> extensionArray;
    const char* str = extensions;;
    do {
        const char* begin = str;
        while(*str != ';' && *str) ++str;
        extensionArray.push_back(std::string(begin, str));
    } while(0 != *str++);
    
    {
        std::lock_guard<std::mutex> lock(loaderMutex);
        const int id = loaderFactories.size();
        loaderFactories.push_back(factory);
        for(size_t i=0; i < extensionArray.size(); ++i){
            loaderIdMap[extensionArray[i]] = id;
        }
    }
}


inline std::string SceneLoader::availableFileExtensions()
{
    using namespace detail::scene_loader;
    std::string extensions;
    for(auto iter = loaderIdMap.begin(); iter != loaderIdMap.end(); ++iter){
        const std::string& extension = iter->first;
        if(!extensions.empty()){
            extensions += ";";
        }
        extensions += extension;
    }
    return extensions;
}


inline SceneLoader::SceneLoader()
{
    impl = new SceneLoaderImpl;
}


inline SceneLoaderImpl::SceneLoaderImpl()
{
    os_ = &nullout();
    defaultDivisionNumber = -1;
    defaultCreaseAngle = -1.0;
}


inline SceneLoader::~SceneLoader()
{
    delete impl;
}


inline void SceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


inline void SceneLoader::setDefaultDivisionNumber(int n)
{
    impl->defaultDivisionNumber = n;
}


inline void SceneLoader::setDefaultCreaseAngle(double theta)
{
    impl->defaultCreaseAngle = theta;
}


inline detail::scene_loader::AbstractSceneLoaderPtr SceneLoaderImpl::findLoader(std::string ext)
{
    using namespace detail::scene_loader;
    AbstractSceneLoaderPtr loader;
    
    std::lock_guard<std::mutex> lock(loaderMutex);

    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    auto p = loaderIdMap.find(ext);
    if(p != loaderIdMap.end()){
        const int loaderId = p->second;
        auto q = loaders.find(loaderId);
        if(q == loaders.end()){
            loader = loaderFactories[loaderId]();
            loaders[loaderId] = loader;
        } else {
            loader = q->second;
        }
    }

    return loader;
}


inline SgNode* SceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


inline SgNode* SceneLoaderImpl::load(const std::string& filename)
{
    std::filesystem::path filepath(filename);

    std::string ext = getExtension(filepath);
    if(ext.empty()){
#if UCNOID_NOT_SUPPORTED
        os() << fmt::format(_("The file format of \"{}\" is unknown because it lacks a file name extension."),
                getFilename(filepath)) << endl;
#else   // UCNOID_NOT_SUPPORTED
        os() << "The file format of \"" << getFilename(filepath) << "\" is unknown because it lacks a file name extension." << std::endl;
#endif  // UCNOID_NOT_SUPPORTED
        return 0;
    }

    SgNode* node = 0;
    auto loader = findLoader(ext);
    if(!loader){
#if UCNOID_NOT_SUPPORTED
        os() << fmt::format(_("The file format of \"{}\" is not supported by the scene loader."),
                getFilename(filepath)) << endl;
#else   // UCNOID_NOT_SUPPORTED
        os() << "The file format of \"" << getFilename(filepath) << "\" is not supported by the scene loader." << std::endl;
#endif  // UCNOID_NOT_SUPPORTED
    } else {
        loader->setMessageSink(os());
        if(defaultDivisionNumber > 0){
            loader->setDefaultDivisionNumber(defaultDivisionNumber);
        }
        if(defaultCreaseAngle >= 0.0){
            loader->setDefaultCreaseAngle(defaultCreaseAngle);
        }
        node = loader->load(filename);
    }

    return node;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_SCENE_LOADER_CPP_H
