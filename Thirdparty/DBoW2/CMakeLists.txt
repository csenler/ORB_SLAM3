cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

# set(OpenCV_DIR "/home/csenler/cpp_ws/opencv_install/lib/cmake/opencv4")
## check if OPENCV_CUSTOM_INSTALL_PATH environment variable is set and OpenCV found there has xfeatures2d module
if(DEFINED ENV{OPENCV_CUSTOM_INSTALL_PATH})
  message("OPENCV_CUSTOM_INSTALL_PATH is set")
  set(OPENCV_CUSTOM_INSTALL_PATH $ENV{OPENCV_CUSTOM_INSTALL_PATH})
  message("OPENCV_CUSTOM_INSTALL_PATH : " ${OPENCV_CUSTOM_INSTALL_PATH})
  find_package(OpenCV 4.2 EXACT REQUIRED PATHS $ENV{OPENCV_CUSTOM_INSTALL_PATH} NO_DEFAULT_PATH)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Custom OpenCV (should have contrib and non-free enabled) > 4.2 not found.")
  endif()
else()
  message("OPENCV_CUSTOM_INSTALL_PATH is not set")
  find_package(OpenCV 4.2 EXACT REQUIRED)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.2 not found.")
  endif()
endif()

# find_package(OpenCV 4 QUIET)
# find_package(OpenCV 4 QUIET PATHS ${PROJECT_SOURCE_DIR}/../../../opencv_4.6.0)
message(STATUS "DBoW2 -> OpenCV version: ${OpenCV_VERSION}")
message(STATUS "DBoW2 -> OpenCV DIR: ${OpenCV_DIR}")

if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

