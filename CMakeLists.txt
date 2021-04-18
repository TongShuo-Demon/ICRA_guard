cmake_minimum_required(VERSION 3.5.1)
project(ICRA-guard)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
set(CMAKE_CXX_STANDARD 14)

# Try to find OpenCV
# set(OpenCV_DIR ....)
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    # If the package has been found, several variables will
    # be set, you can find the full list with descriptions
    # in the OpenCVConfig.cmake file.
    # Print some message showing some of them
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}" \n)
else ()
    message(FATAL_ERROR "Could not locate OpenCV" \n)
endif()

set(Torch_DIR libtorch/share/cmake/Torch)

find_package(Torch PATHS ${Torch_DIR}  NO_DEFAULT REQUIRED)
if (Torch_FOUND)
    message(STATUS "Torch library found!")
    message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}" \n)
else ()
    message(FATAL_ERROR "Could not locate Torch" \n)
endif()

#! 矩阵库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


#设置执行文件输出目录
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(INC_DIR     /opt/DahuaTech/MVViewer/include
        ${Boost_INCLUDE_DIRS})

set(LINK_DIR    /opt/DahuaTech/MVViewer/lib/GenICam/bin/Linux64_x64
        /opt/DahuaTech/MVViewer/lib
        ${PROJECT_SOURCE_DIR}/lib/cubot)

#寻找boost库，并且添加头文件
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#添加动态库,#添加系统关联库
link_directories(${LINK_DIR} )

include_directories(${PROJECT_SOURCE_DIR}/yolov5/include)
include_directories(${PROJECT_SOURCE_DIR}/yellow_tag/include)
include_directories(${PROJECT_SOURCE_DIR}/camera_pose)
include_directories(${PROJECT_SOURCE_DIR}/socket)
include_directories(${PROJECT_SOURCE_DIR}/MLP_pose)
include_directories(${PROJECT_SOURCE_DIR}/camera/inc ${INC_DIR})
include_directories(${PROJECT_SOURCE_DIR}/init/include)
include_directories(${PROJECT_SOURCE_DIR}/others)

file(GLOB SOURCE_FILES_YOLOV5 yolov5/src/*.cpp)
file(GLOB SOURCE_FILES_TAG yellow_tag/src/*.cpp)
file(GLOB SOURCE_FILES_POSE camera_pose/*.cpp)
file(GLOB SOURCE_FILES_socket socket/*.cpp)
file(GLOB SOURCE_FILES_MLP_POSE MLP_pose/*.cpp)
file(GLOB SOURCE_FILES_INIT init/src/*.cpp)
file(GLOB SOURCE_FILES_OTHERS others/*.cpp)



#添加子目录
add_subdirectory(${PROJECT_SOURCE_DIR}/camera/src)


add_executable(${CMAKE_PROJECT_NAME}  main.cpp  ${SOURCE_FILES_TAG} ${SOURCE_FILES_POSE} ${SOURCE_FILES_socket} ${SOURCE_FILES_YOLOV5}
        ${SOURCE_FILES_INIT}  ${SOURCE_FILES_OTHERS} camera/src/shm.cpp  switch_function.h )


#! 第一个节点   主程序
target_link_libraries ( ICRA-guard
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
        ${Boost_LIBRARY}
        boost_thread boost_system  rt -pthread -lboost_date_time
        )





#! 第二个节点   数据采集使用
add_executable(caiji  ${SOURCE_FILES_socket} ${SOURCE_FILES_YOLOV5} mytool/camera_MLP_get.cpp camera/src/shm.cpp  switch_function.h)

target_link_libraries ( caiji
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
        ${Boost_LIBRARY}
        boost_thread boost_system  rt -pthread -lboost_date_time
        )



#！第三个节点  绘制场地图
add_executable(ros  mytool/ros_mat.cpp)
target_link_libraries ( ros
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
        ${Boost_LIBRARY}
        boost_thread boost_system  rt -pthread -lboost_date_time
        )



#！ 第四个节点,采集标签的四个角点，用来计算pnp
add_executable(POSE_MOUSE mytool/test_mouse.cpp)
target_link_libraries ( POSE_MOUSE
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
        ${Boost_LIBRARY}
        boost_thread boost_system  rt -pthread -lboost_date_time
        )
