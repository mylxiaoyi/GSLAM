cmake_minimum_required(VERSION 2.8)
project(GSLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g -fPIC -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -fPIC -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Theia REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)
find_package(Clp QUIET)         

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${THEIA_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${CLP_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_POSITION_INDEPENDENT_CODE TURE)

add_library(${PROJECT_NAME} #SHARED
src/convolve.cpp
src/Drawer.cpp
src/error.cpp
src/Estimation.cpp
src/Frame.cpp
src/Geometry.cpp
src/GlobalReconstruction.cpp
src/GlobalRotationEstimation.cpp
src/GlobalScaleEstimation.cpp
src/GlobalTranslationEstimation.cpp
src/KeyFrameConnection.cpp
src/KeyFrame.cpp
src/KeyFrameDatabase.cpp
src/KLTUtil.cpp
src/las2.cpp
src/MapPoint.cpp
src/ORBextractor.cc
src/ORBmatcher.cc
src/pyramid.cpp
src/svdlib.cpp
src/svdutil.cpp
src/System.cpp
src/TrackUtil.cpp
src/Transform.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
/usr/local/lib/libtheia.a
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${Pangolin_LIBRARIES}
${THEIA_LIBRARIES}
${CERES_LIBRARIES}
Clp
CoinUtils
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(GSLAM_bin src/main.cpp)
target_link_libraries(GSLAM_bin ${PROJECT_NAME})
