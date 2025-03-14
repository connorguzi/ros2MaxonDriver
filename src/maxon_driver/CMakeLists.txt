cmake_minimum_required(VERSION 3.8)
project(maxon_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(maxon_epos_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(maxon_driver src/maxon_driver.cpp)
target_include_directories(maxon_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(maxon_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME} rclcpp maxon_epos_msgs)


# Define the path to the EposCmd library
set(EPOS_LIB_PATH /opt/EposCmdLib_6.8.1.0/lib/x86_64)
set(EPOS_INCLUDE_PATH /opt/EposCmdLib_6.8.1.0/include)

# Include the EposCmd headers
include_directories(${EPOS_INCLUDE_PATH})

# Link the EposCmd library
find_library(EPOS_LIBRARY NAMES EposCmd PATHS ${EPOS_LIB_PATH})

# Link the library with your target
target_link_libraries(${PROJECT_NAME} ${EPOS_LIBRARY})


install(TARGETS maxon_driver
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
