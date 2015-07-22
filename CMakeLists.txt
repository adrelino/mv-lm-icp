# define new project
PROJECT(mv-lm-icp)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0 FATAL_ERROR)

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # -DCMAKE_BUILD_TYPE=Debug
#set(CMAKE_BUILD_TYPE "Release")

#this forces release build, otherwise ceres autodiff is really slow:
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
#to debug add this when runniing CMAKE: -DCMAKE_BUILD_TYPE=Debug
#set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)


if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# Set static build for GLFW
SET(BUILD_SHED_LIBS OFF)
ADD_SUBDIRECTORY(ext/glfw-3.1.1)

# Set shared lib build for the rest
SET(BUILD_SHARED_LIBS ON)

# Find dependencies
# SET(EIGEN_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/ext/Eigen-3.1.2")
SET(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3/")

FIND_PACKAGE(Ceres REQUIRED)
set(G2O_LIBRARIES
    g2o_core g2o_stuff g2o_types_icp g2o_types_slam3d #g2o_solver_csparse g2o_csparse_extension
)

include_directories(include/)

file(GLOB HEADERS
    include/*.h
    include/*.hpp
)

file(GLOB SOURCES src/internal/*.cpp)
#set(SOURCES
#    src/algorithms.cpp
#    src/CPUTimer.cpp
#)

# define additional include directories and linking targets
INCLUDE_DIRECTORIES("ext/glfw-3.1.1/include" ${EIGEN_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})


#set(SOURCES2
#    ${SOURCES}
#    src/frame.cpp
#    src/Visualize.cpp
#    src/algorithms.cpp
#    src/CPUTimer.cpp
#    src/approachcomponents.cpp
#)
## define executable
ADD_EXECUTABLE(multiview src/main_multiview.cpp ${HEADERS} ${SOURCES})
TARGET_LINK_LIBRARIES(multiview glfw ${OPENGL_glu_LIBRARY} ${GLFW_LIBRARIES} ${G2O_LIBRARIES} ${CERES_LIBRARIES})

ADD_EXECUTABLE(pairwise src/main_pairwise.cpp ${HEADERS} ${SOURCES})
TARGET_LINK_LIBRARIES(pairwise glfw ${OPENGL_glu_LIBRARY} ${GLFW_LIBRARIES} ${G2O_LIBRARIES} ${CERES_LIBRARIES})
