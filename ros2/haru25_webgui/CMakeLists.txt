cmake_minimum_required(VERSION 3.8)
project(haru25_webgui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
set(Boost_USE_DEBUG_LIBS        OFF)  # ignore debug libs and
set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)

find_package(Boost 1.8.0 REQUIRED system filesystem thread COMPONENTS)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

aux_source_directory(src WEB_SRCS)

ament_auto_add_executable(haru25_webgui ${WEB_SRCS})

target_link_libraries(haru25_webgui ${Boost_LIBRARIES})

install(
  DIRECTORY include www
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  haru25_webgui
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
