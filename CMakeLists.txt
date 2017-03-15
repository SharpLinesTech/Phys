cmake_minimum_required(VERSION 3.1)

project(phys)

include_directories(include third_party/glm)

if (MSVC)
  add_definitions(-DWIN32_LEAN_AND_MEAN)
  foreach (flag_var
               CMAKE_CXX_FLAGS 
               CMAKE_CXX_FLAGS_DEBUG
               CMAKE_CXX_FLAGS_MINSIZEREL
               CMAKE_CXX_FLAGS_RELEASE
               CMAKE_C_FLAGS 
               CMAKE_C_FLAGS_DEBUG 
               CMAKE_C_FLAGS_RELEASE
               CMAKE_C_FLAGS_MINSIZEREL)
    string(REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
    string(REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
  endforeach()
endif()

enable_testing()
add_subdirectory(third_party/googletest)

add_subdirectory(examples)
add_subdirectory(tests)