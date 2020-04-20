cmake_minimum_required(VERSION 2.8)
set(PROJ_NAME EmscriptenDemo)
project(${PROJ_NAME} CXX C)

set(EIGEN_DIR /usr/include/eigen3 CACHE STRING "The directory of the Eigen library")
set(UCNOID_DIR ../.. CACHE STRING "The directory of the ucnoid")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -Wall -std=c++17")

include_directories(
    ${EIGEN_DIR}
    ${UCNOID_DIR}/include
)

add_executable(${PROJ_NAME}
    main.cpp
)

if (EMSCRIPTEN)
    set_target_properties(${PROJ_NAME} PROPERTIES LINK_FLAGS "-s WASM=0 --embed-file SR1-2D.wrl")   # Disable wasm.
    #set_target_properties(${PROJ_NAME} PROPERTIES LINK_FLAGS "--embed-file SR1-2D.wrl")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
else()
    target_link_libraries(${PROJ_NAME}
        -lstdc++fs
    )
endif()
