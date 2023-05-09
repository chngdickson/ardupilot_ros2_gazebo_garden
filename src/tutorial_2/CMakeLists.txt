cmake_minimum_required(VERSION 3.5)
project(tutorial_2)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(include)

install(
  DIRECTORY config launch models worlds model_sensors rviz scripts geoids
  DESTINATION share/${PROJECT_NAME}
)

function(py_executable subfolder)
  file(GLOB_RECURSE PY_FILES
    ${PROJECT_SOURCE_DIR}/${subfolder}/*.py
  )
  foreach(filename ${PY_FILES})
    install(PROGRAMS
      ${filename}
      DESTINATION lib/${PROJECT_NAME}
    )
  endforeach()
endfunction()

py_executable(scripts)
# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS
#   scripts/helper.py
#   DESTINATION lib/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# For Gazebo
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()