set(daHua_shared_libs
        GCBase_gcc421_v3_0
        GenApi_gcc421_v3_0
        Log_gcc421_v3_0
        MathParser_gcc421_v3_0
        log4cpp_gcc421_v3_0
        NodeMapData_gcc421_v3_0
        XmlParser_gcc421_v3_0
        ImageConvert
        MVSDK
        log4cpp
        )


set(INC_DIR     /opt/DahuaTech/MVviewer/include
        ${Boost_INCLUDE_DIRS})

set(LINK_DIR    /opt/DahuaTech/MVviewer/lib/GenICam/bin/Linux64_x64
        /opt/DahuaTech/MVviewer/lib
        )




link_directories( ${LINK_DIR} )
include_directories(${INC_DIR} )




add_executable(cam_node
        cam_mode.cpp
        camParaConfig.cpp shm.cpp startFetch.cpp streamRetrieve.cpp
        ${DIR_SRC})

target_link_libraries(cam_node
                             ${daHua_shared_libs}
                             ${OpenCV_LIBS}
                            rt -pthread
                            boost_system
                            boost_thread
                            -lboost_date_time
                            )

