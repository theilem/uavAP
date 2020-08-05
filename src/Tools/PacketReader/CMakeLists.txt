add_executable(PacketReader PacketReader.cpp)

target_link_libraries(PacketReader  uavap_core
                                    cps_core)
