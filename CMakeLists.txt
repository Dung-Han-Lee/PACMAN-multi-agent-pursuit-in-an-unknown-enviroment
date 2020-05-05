cmake_minimum_required(VERSION 2.8.3)
project(run_pacman)
find_package( OpenCV REQUIRED )

# Check and use c++ 11
include_directories(${CMAKE_CURRENT_LIST_DIR}/lib)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wextra")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Put bin file under /build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(
include/pursuit
include/exploration
include/world
include/hunt
include/evasion
  ${catkin_INCLUDE_DIRS}
)


set(SOURCES ${SOURCES}
  ${CMAKE_CURRENT_LIST_DIR}/src/world/map_reader.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/world/base_agent.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/pursuit/pursuer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/evasion/prey.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/hunt/hunter.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/exploration/exploration.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${SOURCES})

# Link OpenCV
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

