add_library(pathProcessing pathProcessing.cpp)
target_include_directories(pathProcessing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OMPL_LIBRARIES} ${CMAKE_SOURCE_DIR}/core/utils)
