cmake_minimum_required(VERSION 3.5)
project(nav2_graph_map_server)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(graph_map_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

include_directories(include)

set(map_server_executable map_server)

add_executable(${map_server_executable}
  src/main.cpp)


set(library_name ${map_server_executable}_core)


add_library(${library_name} SHARED
  src/map_server.cpp
  )



set(map_server_dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  nav2_msgs
  graph_map_msgs
  yaml_cpp_vendor
  std_msgs
  nav2_util)



ament_target_dependencies(${map_server_executable}
  ${map_server_dependencies})

ament_target_dependencies(${library_name}
  ${map_server_dependencies})


target_link_libraries(${library_name}
  ${map_io_library_name})

target_link_libraries(${map_server_executable}
  ${library_name})

if(WIN32)
  target_compile_definitions(${map_server_executable} PRIVATE
    YAML_CPP_DLL)
endif()


rclcpp_components_register_nodes(${library_name} "nav2_graph_map_server::MapServer")

install(TARGETS
    ${library_name} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
    ${map_server_executable}  
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
)
ament_export_dependencies(${map_server_dependencies})
ament_package()
