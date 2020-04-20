/**
   @author Shin'ichiro Nakaoka
*/

#include "Referenced.h"

using namespace cnoid;

Referenced::~Referenced()
{
    if(weakCounter_){
        weakCounter_->setDestructed();
    }
}
