add_library(MapLoader MapLoader.cpp)
target_include_directories(MapLoader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MapLoader ${OpenCV_LIBS})
