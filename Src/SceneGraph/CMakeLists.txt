file(GLOB_RECURSE srcs CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
add_library(SceneGraph ${srcs})
target_link_libraries(SceneGraph PUBLIC Eigen3::Eigen)
