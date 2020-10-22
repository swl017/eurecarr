add_executable(MessageInterface
               MessageInterface.cpp)
add_dependencies(MessageInterface autorally_msgs_gencpp)
target_link_libraries(MessageInterface ${catkin_LIBRARIES})

install(TARGETS
  MessageInterface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
