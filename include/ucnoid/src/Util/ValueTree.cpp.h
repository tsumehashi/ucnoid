/**
   @author Shin'ichiro Nakaoka
*/
#ifndef UCNOID_UTIL_VALUE_TREE_CPP_H
#define UCNOID_UTIL_VALUE_TREE_CPP_H

#include "ValueTree.h"
#include <stack>
#include <iostream>
#if UCNOID_NOT_SUPPORTED
#include <fmt/format.h>
#endif  // UCNOID_NOT_SUPPORTED
#include <filesystem>
#include "gettext.h"

#ifdef _WIN32
#define snprintf _snprintf_s
#endif

namespace cnoid {
inline namespace ucnoid {

namespace detail::value_tree {

const bool debugTrace = false;

inline const char* getTypeName(int typeBits){
    if(typeBits & ValueNode::SCALAR){
        return "scalar";
    } else if(typeBits & ValueNode::MAPPING){
        return "mapping";
    } else if(typeBits & ValueNode::LISTING){
        return "listing";
    } else {
        return "unknown type node";
    }
}

inline std::map<std::string, bool> booleanSymbols;

static const char* defaultDoubleFormat = "%.6g";

inline ValueNodePtr invalidNode;
inline MappingPtr invalidMapping;
inline ListingPtr invalidListing;

inline ValueNode::Initializer initializer;

}   // namespace detail::value_tree

//ValueNode::Initializer ValueNode::initializer;

inline ValueNode::Initializer::Initializer()
{
    using namespace detail::value_tree;
    invalidNode = new ValueNode(INVALID_NODE);
    invalidMapping = new Mapping();
    invalidMapping->typeBits = INVALID_NODE;
    invalidListing = new Listing();
    invalidListing->typeBits = INVALID_NODE;
    
    booleanSymbols["true"] = true;
    booleanSymbols["yes"] = true;
    booleanSymbols["on"] = true;
    booleanSymbols["false"] = false;
    booleanSymbols["no"] = false;
    booleanSymbols["off"] = false;
}


inline ValueNode::Exception::Exception()
{
    line_ = -1;
    column_ = -1;
}


inline ValueNode::Exception::~Exception()
{

}


inline std::string ValueNode::Exception::message() const
{
    if(!message_.empty()){
        if(line_ >= 0){
#if UCNOID_NOT_SUPPORTED
            return fmt::format(_("{0} at line {1}, column {2}."), message_, line_, column_);
#else
            return ssformat(message_, " at line ", line_, ", column ", column_, ".");
#endif
        } else {
#if UCNOID_NOT_SUPPORTED
            return fmt::format("{}.", message_);
#else
            return ssformat(message_, ".");
#endif
        }
    } else {
        if(line_ >= 0){
#if UCNOID_NOT_SUPPORTED
            return fmt::format(_("Error at line {0}, column {1}."), line_, column_);
#else
            return ssformat("Error at line ", line_, ", column ", column_, ".");
#endif
        } else {
            return std::string();
        }
    }
}


inline ValueNode::ValueNode(const ValueNode& org)
    : typeBits(org.typeBits),
      line_(org.line_),
      column_(org.column_),
      indexInMapping_(org.indexInMapping_)
{

}


inline ValueNode* ValueNode::clone() const
{
    return new ValueNode(*this);
}


// disabled
inline ValueNode& ValueNode::operator=(const ValueNode&)
{
    // throw an exception here ?
    return *this;
}

/*
  void ValueNode::initialize()
  {
  static bool initialized = false;

  if(!initialized){

  invalidNode = new ValueNode(INVALID_NODE);

  invalidMapping = new Mapping();
  invalidMapping->typeBits = INVALID_NODE;

  invalidListing = new Listing();
  invalidListing->typeBits = INVALID_NODE;
        
  booleanSymbols["true"] = true;
  booleanSymbols["yes"] = true;
  booleanSymbols["on"] = true;
  booleanSymbols["false"] = false;
  booleanSymbols["no"] = false;
  booleanSymbols["off"] = false;

  initialized = true;
  }
  }
*/


inline bool ValueNode::read(int &out_value) const
{
    if(isScalar()){
        const char* nptr = &(static_cast<const ScalarNode* const>(this)->stringValue_[0]);
        char* endptr;
        out_value = strtol(nptr, &endptr, 10);
        if(endptr > nptr){
            return true;
        }
    }
    return false;
}


inline int ValueNode::toInt() const
{
    if(!isScalar()){
        throwNotScalrException();
    }

    const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);
    
    const char* nptr = &(scalar->stringValue_[0]);
    char* endptr;
    const int value = strtol(nptr, &endptr, 10);

    if(endptr == nptr){
        ScalarTypeMismatchException ex;
        ex.setPosition(line(), column());
#if UCNOID_NOT_SUPPORTED
        ex.setMessage(fmt::format(_("The value \"{}\" must be an integer value"), scalar->stringValue_));
#else
        ex.setMessage(ssformat("The value \"", scalar->stringValue_, "\" must be an integer value"));
#endif
        throw ex;
    }

    return value;
}


inline bool ValueNode::read(double& out_value) const
{
    if(isScalar()){
        const char* nptr = &(static_cast<const ScalarNode* const>(this)->stringValue_[0]);
        char* endptr;
        out_value = strtod(nptr, &endptr);
        if(endptr > nptr){
            return true;
        }
    }
    return false;
}


inline bool ValueNode::read(float& out_value) const
{
    if(isScalar()){
        const char* nptr = &(static_cast<const ScalarNode* const>(this)->stringValue_[0]);
        char* endptr;
        out_value = strtof(nptr, &endptr);
        if(endptr > nptr){
            return true;
        }
    }
    return false;
}


inline double ValueNode::toDouble() const
{
    if(!isScalar()){
        throwNotScalrException();
    }

    const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);

    const char* nptr = &(scalar->stringValue_[0]);
    char* endptr;
    const double value = strtod(nptr, &endptr);

    if(endptr == nptr){
        ScalarTypeMismatchException ex;
        ex.setPosition(line(), column());
#if UCNOID_NOT_SUPPORTED
        ex.setMessage(fmt::format(_("The value \"{}\" must be a double value"), scalar->stringValue_));
#else
        ex.setMessage(ssformat("The value \"", scalar->stringValue_, "\" must be a double value"));
#endif
        throw ex;
    }

    return value;
}


inline bool ValueNode::read(bool& out_value) const
{
    using namespace detail::value_tree;
    if(isScalar()){
        const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);
        std::map<std::string, bool>::iterator p = booleanSymbols.find(scalar->stringValue_);
        if(p != booleanSymbols.end()){
            out_value = p->second;
            return true;
        }
    }
    return false;
}


inline bool ValueNode::toBool() const
{
    using namespace detail::value_tree;
    if(!isScalar()){
        throwNotScalrException();
    }

    const ScalarNode* const scalar = static_cast<const ScalarNode* const>(this);
    std::map<std::string, bool>::iterator p = booleanSymbols.find(scalar->stringValue_);
    if(p != booleanSymbols.end()){
        return p->second;
    }
    
    ScalarTypeMismatchException ex;
    ex.setPosition(line(), column());
#if UCNOID_NOT_SUPPORTED
    ex.setMessage(fmt::format(_("The value \"{}\" must be a boolean value"), scalar->stringValue_));
#else
    ex.setMessage(ssformat("The value \"", scalar->stringValue_, "\" must be a boolean value"));
#endif
    throw ex;
}


#ifdef _WIN32

bool ValueNode::read(std::string& out_value) const
{
    if(isScalar()){
        out_value = static_cast<const ScalarNode* const>(this)->stringValue_;
        return !out_value.empty();
    }
    return false;
}


const std::string ValueNode::toString() const
{
    if(!isScalar()){
        throwNotScalrException();
    }
    return static_cast<const ScalarNode* const>(this)->stringValue_;
}

#else

inline bool ValueNode::read(std::string& out_value) const
{
    if(isScalar()){
        out_value = static_cast<const ScalarNode* const>(this)->stringValue_;
        return !out_value.empty();
    }
    return false;
}


inline const std::string& ValueNode::toString() const
{
    if(!isScalar()){
        throwNotScalrException();
    }
    return static_cast<const ScalarNode* const>(this)->stringValue_;
}

#endif


inline ScalarNode::ScalarNode(const std::string& value, StringStyle stringStyle)
    : stringValue_(value),
      stringStyle_(stringStyle)
{
    typeBits = SCALAR;
    line_ = -1;
    column_ = -1;
}


inline ScalarNode::ScalarNode(const char* text, size_t length)
    : stringValue_(text, length)
{
    typeBits = SCALAR;
    stringStyle_ = PLAIN_STRING;
}


inline ScalarNode::ScalarNode(const char* text, size_t length, StringStyle stringStyle)
    : stringValue_(text, length),
      stringStyle_(stringStyle)
{
    typeBits = SCALAR;
    line_ = -1;
    column_ = -1;
}


inline ScalarNode::ScalarNode(int value)
    : stringValue_(std::to_string(value))
{
    typeBits = SCALAR;
    line_ = -1;
    column_ = -1;
    stringStyle_ = PLAIN_STRING;
}


inline ScalarNode::ScalarNode(const ScalarNode& org)
    : ValueNode(org),
      stringValue_(org.stringValue_),
      stringStyle_(org.stringStyle_)
{

}


inline ValueNode* ScalarNode::clone() const
{
    return new ScalarNode(*this);
}


inline const Mapping* ValueNode::toMapping() const
{
    if(!isMapping()){
        throwNotMappingException();
    }
    return static_cast<const Mapping*>(this);
}


inline Mapping* ValueNode::toMapping()
{
    if(!isMapping()){
        throwNotMappingException();
    }
    return static_cast<Mapping*>(this);
}


inline const Listing* ValueNode::toListing() const
{
    if(!isListing()){
        throwNotListingException();
    }
    return static_cast<const Listing*>(this);
}


inline Listing* ValueNode::toListing()
{
    if(!isListing()){
        throwNotListingException();
    }
    return static_cast<Listing*>(this);
}


inline void ValueNode::throwException(const std::string& message) const
{
    Exception ex;
    ex.setPosition(line(), column());
    ex.setMessage(message);
    throw ex;
}


inline void ValueNode::throwNotScalrException() const
{
    using namespace detail::value_tree;
    NotScalarException ex;
    ex.setPosition(line(), column());
#if UCNOID_NOT_SUPPORTED
    ex.setMessage(fmt::format(_("A {} value must be a scalar value"), getTypeName(typeBits)));
#else
    ex.setMessage(ssformat("A ", getTypeName(typeBits), " value must be a scalar value"));
#endif
    throw ex;
}


inline void ValueNode::throwNotMappingException() const
{
    NotMappingException ex;
    ex.setPosition(line(), column());
    ex.setMessage(_("The value is not a mapping"));
    throw ex;
}


inline void ValueNode::throwNotListingException() const
{
    NotListingException ex;
    ex.setPosition(line(), column());
    throw ex;
}


inline Mapping::Mapping()
{
    typeBits = MAPPING;
    line_ = -1;
    column_ = -1;
    mode = READ_MODE;
    indexCounter = 0;
    keyStringStyle_ = PLAIN_STRING;
    isFlowStyle_ = false;
    doubleFormat_ = detail::value_tree::defaultDoubleFormat;
}


inline Mapping::Mapping(int line, int column)
{
    typeBits = MAPPING;
    line_ = line;
    column_ = column;
    mode = READ_MODE;
    indexCounter = 0;
    isFlowStyle_ = false;
    doubleFormat_ = detail::value_tree::defaultDoubleFormat;
}


inline Mapping::Mapping(const Mapping& org)
    : ValueNode(org),
      values(org.values),
      mode(org.mode),
      doubleFormat_(org.doubleFormat_),
      isFlowStyle_(org.isFlowStyle_),
      keyStringStyle_(org.keyStringStyle_)
{
    
}


inline ValueNode* Mapping::clone() const
{
    return new Mapping(*this);
}


inline Mapping* Mapping::cloneMapping() const
{
    return new Mapping(*this);
}


inline Mapping::~Mapping()
{
    clear();
}


inline void Mapping::clear()
{
    values.clear();
    indexCounter = 0;
}


inline void Mapping::setDoubleFormat(const char* format)
{
    doubleFormat_ = format;
}


inline void Mapping::setKeyQuoteStyle(StringStyle style)
{
    keyStringStyle_ = style;
}


inline ValueNode* Mapping::find(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(key);
    if(p != values.end()){
        return p->second.get();
    } else {
        return detail::value_tree::invalidNode.get();
    }
}


inline Mapping* Mapping::findMapping(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(key);
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(node->isMapping()){
            return static_cast<Mapping*>(node);
        }
    }
    return detail::value_tree::invalidMapping.get();
}


inline Listing* Mapping::findListing(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(key);
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(node->isListing()){
            return static_cast<Listing*>(node);
        }
    }
    return detail::value_tree::invalidListing.get();
}


inline ValueNodePtr Mapping::extract(const std::string& key)
{
    if(!isValid()){
        throwNotMappingException();
    }
    iterator p = values.find(key);
    if(p != values.end()){
        ValueNodePtr value = p->second;
        values.erase(p);
        return value;
    }
    return 0;
}


inline bool Mapping::extract(const std::string& key, double& out_value)
{
    ValueNodePtr node = extract(key);
    if(node){
        out_value = node->toDouble();
        return true;
    }
    return false;
}


inline bool Mapping::extract(const std::string& key, std::string& out_value)
{
    ValueNodePtr node = extract(key);
    if(node){
        out_value = node->toString();
        return true;
    }
    return false;
}


inline ValueNode& Mapping::get(const std::string& key) const
{
    if(!isValid()){
        throwNotMappingException();
    }
    const_iterator p = values.find(key);
    if(p == values.end()){
        throwKeyNotFoundException(key);
    }
    return *p->second;
}


inline void Mapping::throwKeyNotFoundException(const std::string& key) const
{
    KeyNotFoundException ex;
    ex.setPosition(line(), column());
    ex.setKey(key);
#if UCNOID_NOT_SUPPORTED
    ex.setMessage(fmt::format(_("Key \"{}\" is not found in the mapping"), key));
#else
    ex.setMessage(ssformat("Key \"", key, "\" is not found in the mapping"));
#endif
    throw ex;
}


inline void Mapping::insertSub(const std::string& key, ValueNode* node)
{
    if(key.empty()){
        EmptyKeyException ex;
        throw ex;
    }
    //values.insert(make_pair(key, node));
    values[key] = node;
    node->indexInMapping_ = indexCounter++;
}


inline void Mapping::insert(const std::string& key, ValueNode* node)
{
    if(!isValid()){
        throwNotMappingException();
    }
    if(!node){
        throwException(_("A node to insert into a Mapping is a null node"));
    }
    const std::string uKey(key);
    insertSub(uKey, node);
}


inline void Mapping::insert(const Mapping* other)
{
    if(!isValid()){
        throwNotMappingException();
    }
    values.insert(other->values.begin(), other->values.end());
}


inline Mapping* Mapping::openMapping_(const std::string& key, bool doOverwrite)
{
    if(!isValid()){
        throwNotMappingException();
    }

    Mapping* mapping = 0;
    const std::string uKey(key);
    iterator p = values.find(uKey);
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(!node->isMapping()){
            values.erase(p);
        } else {
            mapping = static_cast<Mapping*>(node);
            if(doOverwrite){
                mapping->clear();
            }
            mapping->indexInMapping_ = indexCounter++;
        }
    }

    if(!mapping){
        mapping = new Mapping();
        mapping->doubleFormat_ = doubleFormat_;
        insertSub(uKey, mapping);
    }

    return mapping;
}


inline Mapping* Mapping::openFlowStyleMapping_(const std::string& key, bool doOverwrite)
{
    Mapping* m = openMapping_(key, doOverwrite);
    m->setFlowStyle(true);
    return m;
}


inline Listing* Mapping::openListing_(const std::string& key, bool doOverwrite)
{
    if(!isValid()){
        throwNotMappingException();
    }

    Listing* sequence = 0;
    const std::string uKey(key);
    iterator p = values.find(uKey);
    if(p != values.end()){
        ValueNode* node = p->second.get();
        if(!node->isListing()){
            values.erase(p);
        } else {
            sequence = static_cast<Listing*>(node);
            if(doOverwrite){
                sequence->clear();
            }
            sequence->indexInMapping_ = indexCounter++;
        }
    }

    if(!sequence){
        sequence = new Listing();
        sequence->doubleFormat_ = doubleFormat_;
        insertSub(uKey, sequence);
    }

    return sequence;
}


inline Listing* Mapping::openFlowStyleListing_(const std::string& key, bool doOverwrite)
{
    Listing* s = openListing_(key, doOverwrite);
    s->setFlowStyle(true);
    return s;
}


inline bool Mapping::remove(const std::string& key)
{
    return (values.erase(key) > 0);
}


inline bool Mapping::read(const std::string &key, std::string &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


inline bool Mapping::read(const std::string &key, bool &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


inline bool Mapping::read(const std::string &key, int &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


inline bool Mapping::read(const std::string &key, double &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


inline bool Mapping::read(const std::string &key, float &out_value) const
{
    ValueNode* node = find(key);
    if(node->isValid()){
        return node->read(out_value);
    }
    return false;
}


inline void Mapping::write(const std::string &key, const std::string& value, StringStyle stringStyle)
{
    iterator p = values.find(key);
    if(p == values.end()){
        insertSub(key, new ScalarNode(value, stringStyle));
    } else {
        ValueNode* node = p->second.get();
        if(node->isScalar()){
            ScalarNode* scalar = static_cast<ScalarNode*>(node);
            scalar->stringValue_ = value;
            scalar->stringStyle_ = stringStyle;
            scalar->indexInMapping_ = indexCounter++;
        } else {
            throwNotScalrException();
        }
    }
}


inline void Mapping::writeSub(const std::string &key, const char* text, size_t length, StringStyle stringStyle)
{
    iterator p = values.find(key);
    if(p == values.end()){
        insertSub(key, new ScalarNode(text, length, stringStyle));
    } else {
        ValueNode* node = p->second.get();
        if(node->isScalar()){
            ScalarNode* scalar = static_cast<ScalarNode*>(node);
            scalar->stringValue_ = std::string(text, length);
            scalar->stringStyle_ = stringStyle;
            scalar->indexInMapping_ = indexCounter++;
        } else {
            throwNotScalrException();
        }
    }
}



inline void Mapping::write(const std::string &key, bool value)
{
    if(value){
        writeSub(key, "true", 4, PLAIN_STRING);
    } else {
        writeSub(key, "false", 5, PLAIN_STRING);
    }
}


inline void Mapping::write(const std::string &key, int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    writeSub(key, buf, n, PLAIN_STRING);
}


inline void Mapping::write(const std::string &key, double value)
{
    char buf[32];
    int n = snprintf(buf, 32, doubleFormat_, value);
    writeSub(key, buf, n, PLAIN_STRING);
}


inline void Mapping::writePath(const std::string &key, const std::string& value)
{
    write(key, std::filesystem::path(value).string(), DOUBLE_QUOTED);
}


inline Listing::Listing()
{
    typeBits = LISTING;
    line_ = -1;
    column_ = -1;
    doubleFormat_ = detail::value_tree::defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


inline Listing::Listing(int size)
    : values(size)
{
    typeBits = LISTING;
    line_ = -1;
    column_ = -1;
    doubleFormat_ = detail::value_tree::defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


inline Listing::Listing(int line, int column)
{
    typeBits = LISTING;
    line_ = line;
    column_ = column;
    doubleFormat_ = detail::value_tree::defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


inline Listing::Listing(int line, int column, int reservedSize)
    : values(reservedSize)
{
    typeBits = LISTING;
    line_ = line;
    column_ = column;
    values.resize(0);
    doubleFormat_ = detail::value_tree::defaultDoubleFormat;
    isFlowStyle_ = false;
    doInsertLFBeforeNextElement = false;
}


inline Listing::Listing(const Listing& org)
    : ValueNode(org),
      values(org.values),
      doubleFormat_(org.doubleFormat_),
      isFlowStyle_(org.isFlowStyle_),
      doInsertLFBeforeNextElement(org.doInsertLFBeforeNextElement)
{

}


inline ValueNode* Listing::clone() const
{
    return new Listing(*this);
}


inline Listing::~Listing()
{
    clear();
}


inline void Listing::clear()
{
    values.clear();
}


inline void Listing::reserve(int size)
{
    values.reserve(size);
}


inline void Listing::setDoubleFormat(const char* format)
{
    doubleFormat_ = format;
}


inline void Listing::insertLF(int maxColumns, int numValues)
{
    if(values.empty()){
        if(numValues > 0 && numValues > maxColumns){
            doInsertLFBeforeNextElement = true;
        }
    } else if((values.size() % maxColumns) == 0){
        values.back()->typeBits |= (TypeBit)APPEND_LF;
    }
}


inline void Listing::appendLF()
{
    if(values.empty()){
        doInsertLFBeforeNextElement = true;
    } else {
        values.back()->typeBits |= APPEND_LF;
    }
}


inline Mapping* Listing::newMapping()
{
    Mapping* mapping = new Mapping();
    mapping->doubleFormat_ = doubleFormat_;
    append(mapping);
    return mapping;
}


inline void Listing::append(int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    ScalarNode* node = new ScalarNode(buf, n, PLAIN_STRING);
    if(doInsertLFBeforeNextElement){
        node->typeBits |= INSERT_LF;
        doInsertLFBeforeNextElement = false;
    }
    values.push_back(node);
}


inline void Listing::write(int i, int value)
{
    char buf[32];
    int n = snprintf(buf, 32, "%d", value);
    values[i] = new ScalarNode(buf, n, PLAIN_STRING);
}


/*
  void Listing::append(size_t value)
  {
  char buf[32];
  int n = snprintf(buf, 32, "%zd", value);
  values.push_back(new ScalarNode(buf, n, PLAIN_STRING));
  }
*/


inline void Listing::append(double value)
{
    char buf[32];
    int n = snprintf(buf, 32, doubleFormat_, value);
    ScalarNode* node = new ScalarNode(buf, n, PLAIN_STRING);
    if(doInsertLFBeforeNextElement){
        node->typeBits |= INSERT_LF;
        doInsertLFBeforeNextElement = false;
    }
    values.push_back(node);
}


inline void Listing::append(const std::string& value, StringStyle stringStyle)
{
    ScalarNode* node = new ScalarNode(value, stringStyle);
    if(doInsertLFBeforeNextElement){
        node->typeBits |= INSERT_LF;
        doInsertLFBeforeNextElement = false;
    }
    values.push_back(node);
}


inline void Listing::insert(int index, ValueNode* node)
{
    if(index >= 0){
        if(index > static_cast<int>(values.size())){
            index = values.size();
        }
        values.insert(values.begin() + index, node);
    }
}


inline void Listing::write(int i, const std::string& value, StringStyle stringStyle)
{
    values[i] = new ScalarNode(value, stringStyle);
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_VALUE_TREE_CPP_H
