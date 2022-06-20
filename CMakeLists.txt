cmake_minimum_required(VERSION 3.22)
project(pcmattex)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 14)

#set(SOURCE_FILES main.cpp PCPointMaterial.cpp PCPointMaterial.h)
add_library(pcmat
        PCPointMaterial.cpp
        PCPointMaterial.h
        PCPointMaterialDirector.cpp
        PCPointMaterialDirector.h
        PCPointMaterialBuilder.cpp
        PCPointMaterialBuilder.h
        PCViewer.cpp
        PCViewer.h
        PCReader.cpp
        PCReader.h
        PCNormal.cpp
        PCNormal.h
        MockIntensityNoCorrectionPCPointMaterialBuilder.cpp
        MockIntensityNoCorrectionPCPointMaterialBuilder.h
        MockIntensityPCPointMaterialBuilder.cpp
        MockIntensityPCPointMaterialBuilder.h
        PointXYZRGBMaterial.h)

add_library(vector3
        Quaternion.cpp
        Quaternion.h
        Vector3.cpp
        Vector3.hpp)

add_executable(pcmattex main.cpp)


target_link_libraries(pcmattex pcmat)
target_link_libraries(pcmattex vector3)
target_link_libraries(pcmattex ${PCL_LIBRARIES})