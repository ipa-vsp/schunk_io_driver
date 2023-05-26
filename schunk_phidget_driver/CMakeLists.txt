cmake_minimum_required(VERSION 3.8)
project(schunk_phidget_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(phidgets_drivers REQUIRED)
find_package(phidgets_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  phidgets_api
  phidgets_drivers
  phidgets_msgs
  rclcpp_components
)

add_library(phidget_container SHARED src/phidgets_container.cpp)
target_include_directories(phidget_container PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(phidget_container ${dependencies})

add_executable(shunk_phidget_driver_node src/schunk_phidget_driver_node.cpp)
target_include_directories(shunk_phidget_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(shunk_phidget_driver_node phidget_container)
ament_target_dependencies(shunk_phidget_driver_node ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)
# install launch file
install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}
)
# install libraries
install(
  TARGETS phidget_container
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install executable
install(TARGETS shunk_phidget_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

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
