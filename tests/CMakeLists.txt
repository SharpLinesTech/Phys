function(phys_unit_test test_name)
  include_directories(${gtest_SOURCE_DIR}/include)
  add_executable(${test_name} "${test_name}.cpp" )
  target_link_libraries(${test_name} gtest_main)
  add_test(${test_name} ${test_name})
  set_target_properties(${test_name} PROPERTIES FOLDER "tests")
endfunction()

add_subdirectory(collision)