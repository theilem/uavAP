set(FLIGHT_ANALYSIS_SRC ${FLIGHT_ANALYSIS_SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/DurationCondition.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SensorDataCondition.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SteadyStateCondition.cpp
        #		${CMAKE_CURRENT_SOURCE_DIR}/RectanguloidCondition.cpp
        PARENT_SCOPE)
