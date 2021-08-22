/**
   \file
   \brief Implementations of the LinkPath class
   \author Shin'ichiro Nakaoka
*/
  
#include "LinkPath.h"
#include "Link.h"
#include <algorithm>

namespace cnoid {
inline namespace ucnoid {

inline LinkPath::LinkPath()
{

}


inline LinkPath::LinkPath(Link* base, Link* end)
{
    setPath(base, end);
}


/// path from the root link
inline LinkPath::LinkPath(Link* base)
{
    setPath(base);
}


/// This method is disabled.
inline void LinkPath::find(Link* /* root */, bool /* doUpward */, bool /* doDownward */)
{
    throw "The find method for LinkTraverse cannot be used in LinkPath";
}


inline bool LinkPath::setPath(Link* base, Link* end)
{
    links.clear();
    numUpwardConnections = 0;
    bool found = findPathSub(base, 0, end, false);
    if(!found){
        links.clear();
    }
    return found;
}


inline bool LinkPath::findPathSub(Link* link, Link* prev, Link* end, bool isUpward)
{
    links.push_back(link);
    if(isUpward){
        ++numUpwardConnections;
    }
    
    if(link == end){
        return true;
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        if(child != prev){
            if(findPathSub(child, link, end, false)){
                return true;
            }
        }
    }

    Link* parent = link->parent();
    if(parent && parent != prev){
        if(findPathSub(parent, link, end, true)){
            return true;
        }
    }

    links.pop_back();
    if(isUpward){
        --numUpwardConnections;
    }

    return false;
}


/// path from the root link
inline void LinkPath::setPath(Link* end)
{
    links.clear();
    numUpwardConnections = 0;
    findPathFromRootSub(end);
    std::reverse(links.begin(), links.end());
}


inline void LinkPath::findPathFromRootSub(Link* link)
{
    links.push_back(link);
    if(link->parent()){
        findPathFromRootSub(link->parent());
    }
}

}   // inline namespace ucnoid
}   // namespace cnoid
