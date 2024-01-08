# CMakeList.txt : CMake project for AxonV-Task, include source and define
# project specific logic here.
#

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Project files
set(AXV_PROJECT_NAME AxonV-Task)
set(AXV_SOURCE_DIR src)
set(AXV_DATA_DIR data)
set(AXV_TEST_DIR test)

configure_file(${AXV_DATA_DIR}/input.pcd input.pcd COPYONLY)

set(AXV_SOURCE_MAIN "${AXV_SOURCE_DIR}/AxonV-Task.cpp" )

add_executable (AxonV-Task ${AXV_SOURCE_MAIN})
target_link_libraries (AxonV-Task ${PCL_LIBRARIES})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET AxonV-Task PROPERTY CXX_STANDARD 20)
endif()

