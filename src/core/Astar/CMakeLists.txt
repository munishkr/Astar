add_library(HybridAstar HybridAstar.cpp)
target_include_directories(HybridAstar PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OMPL_LIBRARIES} ${CMAKE_SOURCE_DIR}/core/utils)
