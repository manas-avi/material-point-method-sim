cmake_minimum_required(VERSION 3.2)
project(MPM)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-O3 ")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES main.cpp advection.h interpolation.h writeframe.h readfile.h mpmInitialize.h transfer.h global.h setBoundaryVelocity.h UpdateF.h UpdateF.cpp SVD.h constitutiveModel.h Test/derivativeTest.cpp Test/derivativeTest.h Test/kernelTest.cpp Test/kernelTest.h advection.cpp interpolation.cpp transfer.cpp constitutiveModel.cpp SVD.cpp computeMomentum.h AnalyticLevelSet.cpp AnalyticLevelSet.h AnalyticCollisionObject.cpp AnalyticCollisionObject.h)

add_executable(MPM ${SOURCE_FILES})
target_link_libraries(MPM ${PCL_LIBRARIES})
