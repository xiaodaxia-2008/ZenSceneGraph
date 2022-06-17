file(GLOB_RECURSE srcs CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/*.cxx)
add_library(dlibxx STATIC ${srcs})
target_link_libraries(dlibxx PUBLIC dl)
