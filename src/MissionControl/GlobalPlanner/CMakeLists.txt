set(MISSION_CONTROL_SRC ${MISSION_CONTROL_SRC}
						${CMAKE_CURRENT_SOURCE_DIR}/FilletGlobalPlanner/FilletGlobalPlanner.cpp
						${CMAKE_CURRENT_SOURCE_DIR}/SplineGlobalPlanner/SplineGlobalPlanner.cpp
						${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp
						${CMAKE_CURRENT_SOURCE_DIR}/ApproachPlanner/ApproachPlanner.cpp
						${CMAKE_CURRENT_SOURCE_DIR}/ApproachPlanner/approach.cpp
						PARENT_SCOPE)

                                               
