#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
add_executable(obstructor Obstructor.cpp)
add_dependencies(obstructor ${PROJECT_NAME}_gencfg)
target_link_libraries(obstructor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#catkin_install_python(PROGRAMS GenerateWaypoints
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(TARGETS
  obstructor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
