#include <iostream>
#include <ucnoid/BodyLoader>
#include <ucnoid/Body>
#include <ucnoid/Link>

int main(int argc, char* argv[])
{
    std::cout << "EmscriptenSimpleDemo" << std::endl;
    std::string modelfile = "SR1-2D.wrl";
    cnoid::BodyLoader loader;
    cnoid::Body* body = loader.load(modelfile);

    body->calcForwardKinematics();

    for (int i = 0; i < body->numLinks(); ++i) {
        cnoid::Link* link = body->link(i);
        std::cout << link->name() << ":" << std::endl;
        std::cout << "p = " << link->p().transpose() << std::endl;
    }

    return 0;
}
