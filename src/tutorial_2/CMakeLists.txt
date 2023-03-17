cmake_minimum_required(VERSION 3.5)
project(tutorial_2)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


include_directories(include)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/lift_controller.py
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()