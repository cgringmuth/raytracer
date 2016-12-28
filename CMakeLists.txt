project(raytracer)
add_executable(${PROJECT_NAME} raytracer.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

find_package(Threads)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
find_package(OpenCV)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
