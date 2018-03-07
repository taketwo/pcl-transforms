add_custom_target(benchmarks "bash" "${CMAKE_CURRENT_LIST_DIR}/benchmark.sh" "bench_transforms" "${COMPILER}" "${FLAGS}" VERBATIM)

foreach(_flag ${FLAGS})
  set(_tgt bench_transforms_${_flag})
  add_executable(${_tgt} bench_transforms.cpp)
  target_link_libraries(${_tgt} transforms ${CELERO_LIBRARIES})
  target_include_directories(${_tgt} PRIVATE SYSTEM ${CELERO_INCLUDE_DIRS})
  target_compile_options(${_tgt} PRIVATE "-save-temps")
  set_target_properties(${_tgt} PROPERTIES INSTALL_RPATH ${CELERO_RPATH} BUILD_WITH_INSTALL_RPATH TRUE)
  add_flag(${_tgt} ${_flag})
  add_dependencies(${_tgt} celero)
  add_dependencies(benchmarks ${_tgt})
endforeach()

