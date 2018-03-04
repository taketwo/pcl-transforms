enable_testing(true)

add_custom_target(tests "${CMAKE_CTEST_COMMAND}" "-V" VERBATIM)

foreach(_flag ${FLAGS})
  set(_tgt test_transforms_${_flag})
  add_executable(${_tgt} test_transforms.cpp)
  target_include_directories(${_tgt} PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(${_tgt} transforms ${GTEST_LIBRARIES})
  add_flag(${_tgt} ${_flag})
  add_dependencies(${_tgt} gtest)
  add_dependencies(tests ${_tgt})
  add_test(NAME ${_flag} COMMAND ${_tgt})
endforeach()
