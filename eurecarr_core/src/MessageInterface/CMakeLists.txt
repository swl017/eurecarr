include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
add_executable(MessageInterface
               MessageInterface.cpp)
add_dependencies(MessageInterface autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(MessageInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  MessageInterface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
