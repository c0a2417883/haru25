cmake_minimum_required(VERSION 3.8)
project(haru25_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

aux_source_directory(src NAV_SRCS)
ament_auto_add_executable(navigation ${NAV_SRCS})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CNPY_INCLUDE_DIRS})

install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  navigation
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
