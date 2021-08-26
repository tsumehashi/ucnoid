/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_YAML_READER_CPP_H
#define UCNOID_UTIL_YAML_READER_CPP_H

#include "YAMLReader.h"
#include <cerrno>
#include <stack>
#include <iostream>
#ifdef UCNOID_USE_EXTERNAL_YAML
#include <yaml.h>
#else   // UCNOID_USE_EXTERNAL_YAML
#include "../thirdparty/yaml-0.1.7/include/yaml.h"
#endif  // UCNOID_USE_EXTERNAL_YAML
#if UCNOID_NOT_SUPPORTED
#include <fmt/format.h>
#endif  // UCNOID_NOT_SUPPORTED
#include <unordered_map>
#include "gettext.h"

namespace cnoid {
inline namespace ucnoid {

namespace detail::yaml_reader {
constexpr bool debugTrace = false;

}   // namespace detail::yaml_reader

class YAMLReaderImpl
{
public:
    YAMLReaderImpl(YAMLReader* self);
    ~YAMLReaderImpl();

    void setMappingFactory(YAMLReader::MappingFactoryBase* factory);
    void clearDocuments();
    bool load(const std::string& filename);
    bool parse(const std::string& yamlstring);
    bool parse();
    void popNode(yaml_event_t& event);
    void addNode(ValueNode* node, yaml_event_t& event);
    void setAnchor(ValueNode* node, yaml_char_t* anchor, const yaml_mark_t& mark);
    void onDocumentStart(yaml_event_t& event);
    void onDocumentEnd(yaml_event_t& event);
    void onMappingStart(yaml_event_t& event);
    void onMappingEnd(yaml_event_t& event);
    void onListingStart(yaml_event_t& event);
    void onListingEnd(yaml_event_t& event);
    void onScalar(yaml_event_t& event);
    void onAlias(yaml_event_t& event);

    static ScalarNode* createScalar(const yaml_event_t& event);

    YAMLReader* self;
    
    yaml_parser_t parser;
    FILE* file;

    YAMLReader::MappingFactoryBase* mappingFactory;

    std::vector<ValueNodePtr> documents;
    int currentDocumentIndex;

    enum State { NONE, MAPPING_KEY, MAPPING_VALUE, LISTING };

    struct NodeInfo {
        ValueNodePtr node;
        std::string key;
    };

    std::stack<NodeInfo> nodeStack;

    typedef std::unordered_map<std::string, ValueNodePtr> AnchorMap;
    AnchorMap anchorMap;
    AnchorMap importedAnchorMap;

    bool isRegularMultiListingExpected;
    std::vector<int> expectedListingSizes;

    std::string errorMessage;
};


inline YAMLReader::YAMLReader()
{
    impl = new YAMLReaderImpl(this);
}


inline YAMLReaderImpl::YAMLReaderImpl(YAMLReader* self)
    : self(self)
{
    file = 0;
    mappingFactory = new YAMLReader::MappingFactory<Mapping>();
    currentDocumentIndex = 0;
    isRegularMultiListingExpected = false;
}


inline YAMLReader::~YAMLReader()
{
    delete impl;
}


inline YAMLReaderImpl::~YAMLReaderImpl()
{
    yaml_parser_delete(&parser);
    
    if(file){
        fclose(file);
        file = 0;
    }

    delete mappingFactory;
}


inline void YAMLReader::setMappingFactory(MappingFactoryBase* factory)
{
    impl->setMappingFactory(factory);
}


inline void YAMLReaderImpl::setMappingFactory(YAMLReader::MappingFactoryBase* factory)
{
    delete mappingFactory;
    mappingFactory = factory;
}


inline void YAMLReader::expectRegularMultiListing()
{
    impl->isRegularMultiListingExpected = true;
}


inline void YAMLReader::clearDocuments()
{
    impl->clearDocuments();
}


inline void YAMLReaderImpl::clearDocuments()
{
    while(!nodeStack.empty()){
        nodeStack.pop();
    }
    anchorMap.clear();
    documents.clear();
}


inline bool YAMLReader::load(const std::string& filename)
{
    return impl->load(filename);
}


inline bool YAMLReaderImpl::load(const std::string& filename)
{
    yaml_parser_initialize(&parser);
    clearDocuments();

    if(isRegularMultiListingExpected){
        expectedListingSizes.clear();
    }
    
    currentDocumentIndex = 0;

    bool result = false;

    FILE* file = fopen(filename.c_str(), "rb");

    if(file==NULL){
        errorMessage = strerror(errno);
    } else {
        yaml_parser_set_input_file(&parser, file);
        try {
            result = parse();
        }
        catch(const ValueNode::Exception& ex){
#if UCNOID_NOT_SUPPORTED
            errorMessage = format(_("{0} at line {1}, column {2}"),
                    ex.message(), ex.line(), ex.column());
#else   // UCNOID_NOT_SUPPORTED
            errorMessage = ssformat(ex.message(), " at line ", ex.line(), ", column ", ex.column());
#endif   // UCNOID_NOT_SUPPORTED
        }
        fclose(file);
    }

    return result;
}


inline bool YAMLReader::parse(const std::string& yamlstring)
{
    return impl->parse(yamlstring);
}

inline bool YAMLReaderImpl::parse(const std::string& yamlstring)
{
    yaml_parser_initialize(&parser);
    clearDocuments();

    if(isRegularMultiListingExpected){
        expectedListingSizes.clear();
    }
    
    currentDocumentIndex = 0;

    bool result = false;
    
    yaml_parser_set_input_string(&parser, (const unsigned char *)(yamlstring.c_str()), yamlstring.length());
    try {
        result = parse();
    }
    catch(const ValueNode::Exception& ex){
#if UCNOID_NOT_SUPPORTED
        errorMessage = format(_("{0} at line {1}, column {2}"),
                ex.message(), ex.line(), ex.column());
#else   // UCNOID_NOT_SUPPORTED
        errorMessage = ssformat(ex.message(), " at line ", ex.line(), ", column ", ex.column());
#endif   // UCNOID_NOT_SUPPORTED
    }

    return result;
}


inline bool YAMLReaderImpl::parse()
{
    yaml_event_t event;
    
    bool done = false;
    
    while (!done) {

        if(!yaml_parser_parse(&parser, &event)){
            goto error;
        }

        switch(event.type){
            
        case YAML_STREAM_START_EVENT:
            break;
            
        case YAML_STREAM_END_EVENT:
            done = true;
            break;
            
        case YAML_DOCUMENT_START_EVENT:
            onDocumentStart(event);
            break;
            
        case YAML_DOCUMENT_END_EVENT:
            onDocumentEnd(event);
            break;
            
        case YAML_MAPPING_START_EVENT:
            onMappingStart(event);
            break;
            
        case YAML_MAPPING_END_EVENT:
            onMappingEnd(event);
            break;
            
        case YAML_SEQUENCE_START_EVENT:
            onListingStart(event);
            break;
            
        case YAML_SEQUENCE_END_EVENT:
            onListingEnd(event);
            break;
            
        case YAML_SCALAR_EVENT:
            onScalar(event);
            break;
            
        case YAML_ALIAS_EVENT:
            onAlias(event);
            break;
            
        default:
            break;
        }

        yaml_event_delete(&event);
    }

    return !documents.empty();

error:
    if(detail::yaml_reader::debugTrace){
        std::cout << "error" << std::endl;
    }
    if(parser.error != YAML_NO_ERROR && parser.problem != NULL){
        ValueNode::Exception ex;
        ex.setPosition(parser.problem_mark.line+1, parser.problem_mark.column+1);
        ex.setMessage(parser.problem);
        throw ex;
    }
    return false;
}


inline void YAMLReaderImpl::popNode(yaml_event_t& event)
{
    ValueNodePtr current = nodeStack.top().node;
    nodeStack.pop();
    if(nodeStack.empty()){
        documents.push_back(current);
    } else {
        addNode(current, event);
    }
}


inline void YAMLReaderImpl::addNode(ValueNode* node, yaml_event_t& event)
{
    NodeInfo& info = nodeStack.top();
    ValueNode* parent = info.node;

    if(parent->isListing()){
        Listing* listing = static_cast<Listing*>(parent);
        listing->append(node);

    } else if(parent->isMapping()){

        Mapping* mapping = static_cast<Mapping*>(parent);

        if(info.key == "<<"){
            if(node->isMapping()){
                mapping->insert(static_cast<Mapping*>(node));
            } else if(node->isListing()){
                Listing* listing = static_cast<Listing*>(node);
                for(auto& element : *listing){
                    if(element->isMapping()){
                        mapping->insert(static_cast<Mapping*>(element.get()));
                    } else {
                        ValueNode::SyntaxException ex;
                        ex.setMessage(_("An element to merge by the \"<<\" key must be a mapping"));
                        const yaml_mark_t& start_mark = event.start_mark;
                        ex.setPosition(start_mark.line, start_mark.column);
                        throw ex;
                    }
                }
            } else {
                ValueNode::SyntaxException ex;
                ex.setMessage(_("A value to merge by the \"<<\" key must be mapping or listing"));
                const yaml_mark_t& start_mark = event.start_mark;
                ex.setPosition(start_mark.line, start_mark.column);
                throw ex;
            }
        }
        
        mapping->insert(info.key, node);
        info.key.clear();
    }
}


inline void YAMLReaderImpl::setAnchor(ValueNode* node, yaml_char_t* anchor, const yaml_mark_t& mark)
{
    std::pair<AnchorMap::iterator, bool> inserted =
        anchorMap.insert(AnchorMap::value_type((char*)anchor, node));
    if(!inserted.second){
        ValueNode::Exception ex;
#if UCNOID_NOT_SUPPORTED
        ex.setMessage(format(_("Anchor \"{}\" is duplicated"), (char*)anchor));
#else   // UCNOID_NOT_SUPPORTED
        ex.setMessage(ssformat("Anchor \"", (char*)anchor, "\" is duplicated"));
#endif  // UCNOID_NOT_SUPPORTED
        ex.setPosition(mark.line, mark.column);
        throw ex;
    }
}

    
inline void YAMLReaderImpl::onDocumentStart(yaml_event_t&)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onDocumentStart()" << std::endl;
    }
}


inline void YAMLReaderImpl::onDocumentEnd(yaml_event_t&)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onDocumentEnd()" << std::endl;
    }
}


inline void YAMLReaderImpl::onMappingStart(yaml_event_t& event)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onMappingStart()" << std::endl;
    }

    NodeInfo info;
    const yaml_mark_t& mark = event.start_mark;
    Mapping* mapping = mappingFactory->create(mark.line, mark.column);
    mapping->setFlowStyle(event.data.mapping_start.style == YAML_FLOW_MAPPING_STYLE);
    info.node = mapping;

    nodeStack.push(info);

    if(event.data.mapping_start.anchor){
        setAnchor(mapping, event.data.mapping_start.anchor, mark);
    }
}


inline void YAMLReaderImpl::onMappingEnd(yaml_event_t& event)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onMappingEnd()" << std::endl;
    }

    popNode(event);
}


inline void YAMLReaderImpl::onListingStart(yaml_event_t& event)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onListingStart()" << std::endl;
    }

    NodeInfo info;
    Listing* listing;

    const yaml_mark_t& mark = event.start_mark;

    if(!isRegularMultiListingExpected){
        listing = new Listing(mark.line, mark.column);
    } else {
        size_t level = nodeStack.size();
        if(expectedListingSizes.size() <= level){
            expectedListingSizes.resize(level + 1, 0);
        }
        const int prevSize = expectedListingSizes[level];
        listing = new Listing(mark.line, mark.column, prevSize);
    }

    listing->setFlowStyle(event.data.sequence_start.style == YAML_FLOW_SEQUENCE_STYLE);
    info.node = listing;
    nodeStack.push(info);

    if(event.data.sequence_start.anchor){
        setAnchor(listing, event.data.sequence_start.anchor, mark);
    }
}


inline void YAMLReaderImpl::onListingEnd(yaml_event_t& event)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onListingEnd()" << std::endl;
    }

    if(isRegularMultiListingExpected){
        Listing* listing = static_cast<Listing*>(nodeStack.top().node.get());
        const int level = nodeStack.size() - 1;
        expectedListingSizes[level] = listing->size();
    }
    
    popNode(event);
}


inline void YAMLReaderImpl::onScalar(yaml_event_t& event)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onScalar()" << std::endl;
    }

    yaml_char_t* value = event.data.scalar.value;
    size_t length = event.data.scalar.length;

    if(nodeStack.empty()){
        ValueNode::SyntaxException ex;
        ex.setMessage(_("Scalar value cannot be put on the top-level text position"));
        const yaml_mark_t& start_mark = event.start_mark;
        ex.setPosition(start_mark.line, start_mark.column);
        throw ex;
    }

    NodeInfo& info = nodeStack.top();
    ValueNodePtr& parent = info.node;

    ScalarNode* scalar = 0;
     
    if(parent->isMapping()){
        if(info.key.empty()){
            info.key = std::string((char*)value, length);
            if(info.key.empty()){
                ValueNode::SyntaxException ex;
                ex.setMessage(_("empty key"));
                const yaml_mark_t& start_mark = event.start_mark;
                ex.setPosition(start_mark.line, start_mark.column);
                throw ex;
            }
        } else {
            scalar = createScalar(event);
        }
    } else if(parent->isListing()){
        scalar = createScalar(event);
    }
    if(scalar){
        addNode(scalar, event);
        if(event.data.scalar.anchor){
            setAnchor(scalar, event.data.scalar.anchor, event.start_mark);
        }
    }
}


inline ScalarNode* YAMLReaderImpl::createScalar(const yaml_event_t& event)
{
    ScalarNode* scalar = new ScalarNode((char*)event.data.scalar.value, event.data.scalar.length);

    const yaml_mark_t& start_mark = event.start_mark;
    scalar->line_ = start_mark.line;
    scalar->column_ = start_mark.column;

    switch(event.data.scalar.style){
    case YAML_PLAIN_SCALAR_STYLE:
        scalar->stringStyle_ = PLAIN_STRING;
        break;
    case YAML_SINGLE_QUOTED_SCALAR_STYLE:
        scalar->stringStyle_ = SINGLE_QUOTED;
        break;
    case YAML_DOUBLE_QUOTED_SCALAR_STYLE:
        scalar->stringStyle_ = DOUBLE_QUOTED;
        break;
    case YAML_LITERAL_SCALAR_STYLE:
        scalar->stringStyle_ = LITERAL_STRING;
        break;
    case YAML_FOLDED_SCALAR_STYLE:
        scalar->stringStyle_ = FOLDED_STRING;
        break;
    default:
        scalar->stringStyle_ = DOUBLE_QUOTED;
    }

    return scalar;
}


inline void YAMLReaderImpl::onAlias(yaml_event_t& event)
{
    if(detail::yaml_reader::debugTrace){
        std::cout << "YAMLReaderImpl::onAlias()" << std::endl;
    }

    auto node = self->findAnchoredNode((char*)event.data.alias.anchor);

    if(!node){
        ValueNode::Exception ex;
#if UCNOID_NOT_SUPPORTED
        ex.setMessage(format(_("Anchor \"{}\" is not defined"), (char*)event.data.alias.anchor));
#else   // UCNOID_NOT_SUPPORTED
        ex.setMessage(ssformat("Anchor \"", (char*)event.data.alias.anchor, "\" is not defined"));
#endif  // UCNOID_NOT_SUPPORTED
        const yaml_mark_t& mark = event.start_mark;
        ex.setPosition(mark.line, mark.column);
        throw ex;
    }
    
    addNode(node, event);
}


inline ValueNode* YAMLReader::findAnchoredNode(const std::string& anchor)
{
    ValueNode* node = nullptr;
    
    auto p = impl->anchorMap.find(anchor);
    if(p != impl->anchorMap.end()){
        node = p->second;
    } else {
        auto q = impl->importedAnchorMap.find(anchor);
        if(q != impl->importedAnchorMap.end()){
            node = q->second;
        }
    }

    return node;
}


inline void YAMLReader::importAnchors(const YAMLReader& anotherReader)
{
    auto& mapToImport = anotherReader.impl->anchorMap;
    impl->importedAnchorMap.insert(mapToImport.begin(), mapToImport.end());
}


inline int YAMLReader::numDocuments()
{
    return impl->documents.size();
}


inline ValueNode* YAMLReader::document(int index)
{
    if(index >= static_cast<int>(impl->documents.size())){
        ValueNode::DocumentNotFoundException ex;
        if(index == 0){
            ex.setMessage(_("The yaml file does not contains any documents."));
        } else {
#if UCNOID_NOT_SUPPORTED
            ex.setMessage(
                format(_("The yaml file does not contains {}-th document."), index));
#else   // UCNOID_NOT_SUPPORTED
            ex.setMessage(
                ssformat("The yaml file does not contains ", index, "-th document."));
#endif  // UCNOID_NOT_SUPPORTED
        }
        ex.setPosition(-1, -1);
        throw ex;
    }
    
    return impl->documents[index];
}


inline ValueNode* YAMLReader::loadDocument(const std::string& filename)
{
    if(!load(filename)){
        ValueNode::FileException ex;
        ex.setMessage(errorMessage());
        ex.setPosition(-1, -1);
        throw ex;
    }
    return document();
}


inline const std::string& YAMLReader::errorMessage()
{
    return impl->errorMessage;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_YAML_READER_CPP_H
