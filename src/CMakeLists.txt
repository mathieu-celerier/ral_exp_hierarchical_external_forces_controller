set(controller_SRC
  RALExpHierarchicalController.cpp
)

set(controller_HDR
  ${CMAKE_SOURCE_DIR}/include/RALExpHierarchicalController/RALExpHierarchicalController.h
  ${CMAKE_SOURCE_DIR}/include/RALExpHierarchicalController/utils/ROSSubscriber.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DRALExpHierarchicalController_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)