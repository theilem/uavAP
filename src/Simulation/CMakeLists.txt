set(SIM_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/FixedWing.cpp)


add_library(uavap_sim SHARED ${SIM_SRCS})

target_link_libraries(uavap_sim
        uavap_core
        ${CPS_CORE_LIB}
        ${BOOST}
        ${SYSTEM_LIBS}
        pthread
)

install(TARGETS uavap_sim
        RUNTIME DESTINATION bin/uavAP
        LIBRARY DESTINATION lib)
