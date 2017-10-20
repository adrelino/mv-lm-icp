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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

### GLFW
SET(BUILD_SHARED_LIBS OFF) # Set static build for GLFW
ADD_SUBDIRECTORY(ext/glfw)
SET(BUILD_SHARED_LIBS ON) # Set shared lib build for the rest

### Sophus
# use lovegroves version of sophus with ceres extensions!
#ADD_SUBDIRECTORY(ext/sophus-ceres)
SET(Sophus_INCLUDE_DIR ext/sophus-ceres)

### Eigen
FIND_PACKAGE(Eigen3 REQUIRED)

### GFLAGS
FIND_PACKAGE(gflags REQUIRED)
MESSAGE("gflags found ${Gflags_FOUND} at ${GFLAGS_INCLUDE_DIRS} with namespace: ${GFLAGS_NAMESPACE}")

### CERES
FIND_PACKAGE(Ceres REQUIRED)
MESSAGE("ceres found ${Ceres_FOUND} at ${Ceres_DIR}")

### G2O
FIND_PACKAGE(G2O REQUIRED)
set(G2O_LIBRARIES
    "${G2O_CORE_LIBRARY}" "${G2O_STUFF_LIBRARY}" "${G2O_TYPES_ICP}" "${G2O_TYPES_SLAM3D}"
) #g2o_core g2o_stuff g2o_types_icp g2o_types_slam3d
MESSAGE("g2o found: ${G2O_FOUND} at: ${G2O_INCLUDE_DIR} libraries: ${G2O_LIBRARIES}")
#if(UNIX AND NOT APPLE)
#    if(EXISTS /opt/ros/indigo/include/g2o)  #and ros and ros-indigo-libg2o is installed
#        MESSAGE("using ROS g2o from package ros-indigo-libg2o")
#        link_directories(/opt/ros/indigo/lib/)
#    endif()
#endif()


include_directories(include/)
INCLUDE_DIRECTORIES(ext/glfw/include)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})

MESSAGE("Sophus_INCLUDE_DIR ${Sophus_INCLUDE_DIR}")
MESSAGE("Eigen3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}")
MESSAGE("Ceres_INCLUDE_DIR ${CERES_INCLUDE_DIRS}")
MESSAGE("Ceres_LIBRARIES ${CERES_LIBRARIES}")



file(GLOB HEADERS
    include/*.h
    include/*.hpp
)

file(GLOB SOURCES src/internal/*.cpp)

ADD_EXECUTABLE(multiview src/main_multiview.cpp ${HEADERS} ${SOURCES})
TARGET_LINK_LIBRARIES(multiview glfw ${OPENGL_glu_LIBRARY} ${GLFW_LIBRARIES} ${G2O_LIBRARIES} ${GFLAGS_LIBRARIES} ${CERES_LIBRARIES})

ADD_EXECUTABLE(pairwise src/main_pairwise.cpp ${HEADERS} ${SOURCES})
TARGET_LINK_LIBRARIES(pairwise glfw ${OPENGL_glu_LIBRARY} ${GLFW_LIBRARIES} ${G2O_LIBRARIES} ${GFLAGS_LIBRARIES} ${CERES_LIBRARIES})
