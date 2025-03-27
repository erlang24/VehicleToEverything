cmake_minimum_required(VERSION 3.8)
project(grpc_bridge_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(Protobuf_DIR  /usr/lib/cmake)
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
pkg_check_modules(protobuf REQUIRED protobuf>=3.0)
set(Protobuf_DIR  /usr/lib/cmake)
link_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(grpc_bridge_gui src/grpc_bridge_gui.cpp ${PROTO_FILES} )
# add_library(${PROJECT_NAME} )
include_directories(${CMAKE_CURRENT_BINARY_DIR}   ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_target_dependencies(grpc_bridge_gui
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs
                          gps_msgs
                          Protobuf
                          gRPC
                          )

target_link_libraries(grpc_bridge_gui      
                      protobuf::libprotobuf
                      gRPC::grpc
                      gRPC::grpc++                        
                      obu_grpc_proto  -pthread ${protobuf_LIBRARIES} )
target_include_directories(grpc_bridge_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(grpc_bridge_gui PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS grpc_bridge_gui
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
