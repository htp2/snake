cmake_minimum_required (VERSION 3.10)
project (snake)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(bigssUtil REQUIRED CONFIG
	NAMES BIGSSUtil
  PATHS "~/bigss/util/build")
include_directories(${BIGSS_UTIL_INCLUDE_DIRS})
link_directories(${BIGSS_UTIL_LIBS})

set (REQUIRED_CISST_LIBRARIES
     cisstMultiTask
     cisstCommon
     cisstOSAbstraction
     )

find_package (cisst 1.1.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  cisst_ros_bridge
  cisst
  geometry_msgs
  sensor_msgs
  std_msgs
  )
include_directories(${catkin_INCLUDE_DIRS})


# catkin/ROS paths
cisst_is_catkin_build (snake_IS_CATKIN_BUILT)
if (snake_IS_CATKIN_BUILT)
  set (snake_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/share/snake/cmake")
  set (LIBRARY_OUTPUT_PATH "${snake_BINARY_DIR}/lib")
else ()
  set (snake_CONFIG_FILE_DIR "${snake_BINARY_DIR}")
endif ()

# create/configure file for find_package 
set (snake_INCLUDE_DIR
     "${snake_SOURCE_DIR}/include"
     "${snake_BINARY_DIR}/include")
set (snake_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
set (snake_LIBRARIES snake)

include_directories (${snake_INCLUDE_DIR})
set (snake_HEADER_DIR "${snake_SOURCE_DIR}/include/snake")



catkin_package(INCLUDE_DIRS include)



  set (HEADER_FILES 
      ${snake_HEADER_DIR}/mtsTaskTest.h
       )

  set (SOURCE_FILES
       )

  add_library (snake
               ${HEADER_FILES}
               ${SOURCE_FILES}
               )

set_property (TARGET snake PROPERTY FOLDER "snake")
set_target_properties(snake PROPERTIES LINKER_LANGUAGE CXX)
cisst_target_link_libraries (snake ${REQUIRED_CISST_LIBRARIES})
target_link_libraries(snake Qt5::Widgets ${BIGSS_UTIL_LIB})

add_executable(startMaxonWithUI code/main_MaxonMotorWithUI.cpp)
target_link_libraries(startMaxonWithUI snake ${catkin_LIBRARIES})

add_executable(MAINTEST code/main_test.cpp)
target_link_libraries(MAINTEST snake ${catkin_LIBRARIES})

# Install targets for headers and library
install (DIRECTORY
"${snake_SOURCE_DIR}/include/"
"${snake_BINARY_DIR}/include/"
DESTINATION include
PATTERN .svn EXCLUDE)

install (TARGETS snake
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)

install (FILES "${snake_CMAKE_CONFIG_FILE}"
           DESTINATION "share/snake")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)

