cmake_minimum_required(VERSION 3.8)
project(sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找 Boost 库
#find_package(Boost REQUIRED COMPONENTS system filesystem)
# set(BOOST_ROOT "/home/robot/opt/boost_1_84_0/")
find_package(Boost REQUIRED)
if(Boost_FOUND)
    message(STATUS "Boost found:")
    message(STATUS "Boost include directory: ${Boost_INCLUDE_DIRS}")
    # set(Boost_INCLUDE_DIRS "/home/gamemaster/opt/boost/")
    set(Boost_INCLUDE_DIRS "/home/robot/opt/boost_1_84_0/")
    message(STATUS "Reset Boost include directory to: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost library directory: ${Boost_LIBRARY_DIRS}")
else()
    message(FATAL_ERROR "Boost not found.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
# 包含 Boost 头文件的路径
INCLUDE_DIRECTORIES(
    /usr/include/eigen3
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/inc/include
)


LINK_DIRECTORIES(
    /usr/lib/x86_64-linux-gun
    ${PROJECT_SOURCE_DIR}/lib
)

add_executable(control_sub src/control_sub.cpp)
  TARGET_LINK_LIBRARIES(control_sub
    RCI_Client.a
    xMateModel.a
    orocos-kdl.a
    glog.a
    pthread
    ${Boost_LIBRARIES}
    )
ament_target_dependencies(control_sub rclcpp std_msgs)

install(TARGETS
  control_sub
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
