project(raytracer)
cmake_minimum_required(VERSION 3.0)


add_executable(${PROJECT_NAME}

        src/camera.cpp
        src/camera.h

        src/color.cpp
        src/color.h

        src/common.h

        src/container.cpp
        src/container.h

        src/helper.cpp
        src/helper.h

        src/material.h

        src/primitives.cpp
        src/primitives.h

        src/raytracer.cpp

        src/scene.cpp
        src/scene.h

        src/acceleration.cpp src/acceleration.h 

	src/model.cpp src/model.h)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wpedantic -Wshadow -Weffc++ -pedantic" )

find_package(Threads)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
find_package(OpenCV)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
find_package(OpenMP)
if (OPENMP_FOUND AND 1)         # use AND 0 to switch to c++11 threads
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OPENMP=1)
endif()
