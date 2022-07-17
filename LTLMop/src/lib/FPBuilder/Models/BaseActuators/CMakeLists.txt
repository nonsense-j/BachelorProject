set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/#Base#Start.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/#Base#Start.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/#Base#Move.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/#Base#Move.fpp"
  ## add your actuators ##
)

find_library(LIBROSCPP libroscpp "/opt/ros/noetic/lib")
add_library(libroscpp SHARED IMPORTED GLOBAL)
set_target_properties(libroscpp PROPERTIES IMPORTED_LOCATION /opt/ros/noetic/lib/libroscpp.so)

find_library(LIBROSCONSOLE librosconsole "/opt/ros/noetic/lib")
add_library(librosconsole SHARED IMPORTED GLOBAL)
set_target_properties(librosconsole PROPERTIES IMPORTED_LOCATION /opt/ros/noetic/lib/librosconsole.so)

find_library(LIBROSCPPSER libroscpp_serialization "/opt/ros/noetic/lib")
add_library(libroscpp_serialization SHARED IMPORTED GLOBAL)
set_target_properties(libroscpp_serialization PROPERTIES IMPORTED_LOCATION /opt/ros/noetic/lib/libroscpp_serialization.so)

set(MOD_DEPS
    libroscpp
    librosconsole
    libroscpp_serialization
)

include_directories(
    /opt/ros/noetic/include
)

link_directories(
    /opt/ros/noetic/lib
)

register_fprime_module()