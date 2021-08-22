/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_BODY_LOADER_CPP_H
#define UCNOID_BODY_BODY_LOADER_CPP_H

#include "BodyLoader.h"
#if UCNOID_NOT_SUPPORTED
#include "YAMLBodyLoader.h"
#endif  // UCNOID_NOT_SUPPORTED
#include "VRMLBodyLoader.h"
#include "Body.h"
#include <ucnoid/SceneLoader>
#if UCNOID_NOT_SUPPORTED
#include <ucnoid/STLSceneLoader>
#include <ucnoid/YAMLSceneLoader>
#endif  // UCNOID_NOT_SUPPORTED
#include <ucnoid/ValueTree>
#include <ucnoid/Exception>
#include <ucnoid/FileUtil>
#include <ucnoid/NullOut>
#if UCNOID_NOT_SUPPORTED
#include <fmt/format.h>
#endif  // UCNOID_NOT_SUPPORTED
#include <mutex>
#include "gettext.h"

namespace cnoid {
inline namespace ucnoid {

namespace detail::body_loader {

typedef std::function<AbstractBodyLoaderPtr()> LoaderFactory;
typedef std::map<std::string, LoaderFactory> LoaderFactoryMap;
inline LoaderFactoryMap loaderFactoryMap;
inline std::mutex loaderFactoryMapMutex;

class SceneLoaderAdapter : public AbstractBodyLoader
{
    AbstractSceneLoader* loader;
    std::ostream* os;

public:
    SceneLoaderAdapter(AbstractSceneLoader* loader) : loader(loader) {
        os = &nullout();
    }
    ~SceneLoaderAdapter() { delete loader; }

    virtual void setMessageSink(std::ostream& os_)
    {
        os = &os_;
    }

    virtual bool load(Body* body, const std::string& filename) {

        body->clearDevices();
        body->clearExtraJoints();

        loader->setMessageSink(*os);
        SgNode* scene = loader->load(filename);
        if(scene){
            Link* link = body->createLink();
            link->setName("Root");
            link->setShape(scene);
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            body->setRootLink(link);
            body->setModelName(std::filesystem::path(filename).stem().string());
        }

        return (scene != 0);
    }
};

struct FactoryRegistration
{
    FactoryRegistration(){
#if UCNOID_NOT_SUPPORTED
        BodyLoader::registerLoader(
            "body", [](){ return std::make_shared<YAMLBodyLoader>(); });
        BodyLoader::registerLoader(
            "yaml", [](){ return std::make_shared<YAMLBodyLoader>(); });
        BodyLoader::registerLoader(
            "yml", [](){ return std::make_shared<YAMLBodyLoader>(); });
#endif  // UCNOID_NOT_SUPPORTED
        BodyLoader::registerLoader(
            "wrl", [](){ return std::make_shared<VRMLBodyLoader>(); });
#if UCNOID_NOT_SUPPORTED
        BodyLoader::registerLoader(
            "scen", [](){ return std::make_shared<SceneLoaderAdapter>(new YAMLSceneLoader); });
        BodyLoader::registerLoader(
            "stl", [](){ return std::make_shared<SceneLoaderAdapter>(new STLSceneLoader); });
        BodyLoader::registerLoader(
            "dae", [](){ return std::make_shared<SceneLoaderAdapter>(new SceneLoader); });
#endif  // UCNOID_NOT_SUPPORTED
    }
} factoryRegistration;
    
}   // namespace detail::body_loader

inline bool BodyLoader::registerLoader(const std::string& extension, std::function<AbstractBodyLoaderPtr()> factory)
{
    using namespace detail::body_loader;
    std::lock_guard<std::mutex> lock(loaderFactoryMapMutex);
    loaderFactoryMap[extension] = factory;
    return  true;
}
   

class BodyLoaderImpl
{
public:
    std::ostream* os;
    AbstractBodyLoaderPtr actualLoader;
    bool isVerbose;
    bool isShapeLoadingEnabled;
    int defaultDivisionNumber;
    double defaultCreaseAngle;

    typedef std::map<std::string, AbstractBodyLoaderPtr> BodyLoaderMap;
    BodyLoaderMap bodyLoaderMap;
        
    BodyLoaderImpl();
    ~BodyLoaderImpl();
    bool load(Body* body, const std::string& filename);
    void mergeExtraLinkInfos(Body* body, Mapping* info);
};



inline BodyLoader::BodyLoader()
{
    impl = new BodyLoaderImpl();
}


inline BodyLoaderImpl::BodyLoaderImpl()
{
    os = &nullout();
    isVerbose = false;
    isShapeLoadingEnabled = true;
    defaultDivisionNumber = -1;
    defaultCreaseAngle = -1.0;
}


inline BodyLoader::~BodyLoader()
{
    delete impl;
}


inline BodyLoaderImpl::~BodyLoaderImpl()
{

}


inline void BodyLoader::setMessageSink(std::ostream& os)
{
    impl->os = &os;
}


inline void BodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


inline void BodyLoader::setShapeLoadingEnabled(bool on)
{
    impl->isShapeLoadingEnabled = on;
}
    

inline void BodyLoader::setDefaultDivisionNumber(int n)
{
    impl->defaultDivisionNumber = n;
}


inline void BodyLoader::setDefaultCreaseAngle(double theta)
{
    impl->defaultCreaseAngle = theta;
}


inline bool BodyLoader::load(Body* body, const std::string& filename)
{
    body->info()->clear();    
    return impl->load(body, filename);
}


inline Body* BodyLoader::load(const std::string& filename)
{
    Body* body = new Body();
    if(load(body, filename)){
        return body;
    } else {
        delete body;
        return 0;
    }
}


inline bool BodyLoaderImpl::load(Body* body, const std::string& filename)
{
    using namespace detail::body_loader;
    bool result = false;

    std::filesystem::path path(filename);
    std::string ext = getExtension(path);

    try {
        auto p = bodyLoaderMap.find(ext);
        if(p != bodyLoaderMap.end()){
            actualLoader = p->second;
        } else {
            std::lock_guard<std::mutex> lock(loaderFactoryMapMutex);
            auto q = loaderFactoryMap.find(ext);
            if(q != loaderFactoryMap.end()){
                LoaderFactory factory = q->second;
                actualLoader = factory();
                bodyLoaderMap[ext] = actualLoader;
            }
        }

        if(!actualLoader){
            (*os) <<
#if UCNOID_NOT_SUPPORTED
                fmt::format(_("The file format of \"{}\" is not supported by the body loader.\n"),
                            path.filename().string());
#else   // UCNOID_NOT_SUPPORTED
            ssformat("The file format of \"", path.filename().string(), "\" is not supported by the body loader.\n");
#endif  // UCNOID_NOT_SUPPORTED
        } else {
            actualLoader->setMessageSink(*os);
            actualLoader->setVerbose(isVerbose);
            actualLoader->setShapeLoadingEnabled(isShapeLoadingEnabled);
            actualLoader->setDefaultDivisionNumber(defaultDivisionNumber);
            actualLoader->setDefaultCreaseAngle(defaultCreaseAngle);

            result = actualLoader->load(body, filename);
        }
        
    } catch(const ValueNode::Exception& ex){
        (*os) << ex.message();
    } catch(const nonexistent_key_error& error){
#if UCNOID_NOT_SUPPORTED
        if(const std::string* message = boost::get_error_info<error_info_message>(error)){
            (*os) << *message;
        }
#else   // UCNOID_NOT_SUPPORTED
        (*os) << error.message();
#endif  // UCNOID_NOT_SUPPORTED
    } catch(const std::exception& ex){
        (*os) << ex.what();
    }
    os->flush();
    
    return result;
}


inline AbstractBodyLoaderPtr BodyLoader::lastActualBodyLoader() const
{
    return impl->actualLoader;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_BODY_LOADER_CPP_H
