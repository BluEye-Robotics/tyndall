cmake_minimum_required(VERSION 3.5)

project(tyndall)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(C_CXX_EXTRA_FLAGS "-O2 -Wall -Wconversion -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fconcepts-diagnostics-depth=4")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C_CXX_EXTRA_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_CXX_EXTRA_FLAGS}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DDEBUG)

include_directories(${CMAKE_SOURCE_DIR})
set(LD_FLAGS "-lpthread -lzmq -lprotobuf -lfmt -lrt -latomic")

find_package(ament_cmake)
find_package(rclcpp)
find_package(builtin_interfaces)

if("${rclcpp_FOUND}")
  set(LD_FLAGS "${LD_FLAGS} -lpthread")
else()
  add_definitions("-DNO_ROS")
endif()

file(GLOB CFILES "tyndall/*.c*" "tyndall/*/*.c*")
add_library(tyndall SHARED ${CFILES})
target_link_libraries(tyndall ${LD_FLAGS})
install(TARGETS tyndall DESTINATION lib)

if("${rclcpp_FOUND}")
  ament_target_dependencies(tyndall rclcpp builtin_interfaces)
endif()

file(GLOB HEADERS "tyndall/*.h*" "tyndall/*/*.h*")

foreach(HEADER ${HEADERS})
  execute_process(COMMAND bash "-c" "dirname -z $(realpath --relative-to=${CMAKE_SOURCE_DIR} ${HEADER})" OUTPUT_VARIABLE HEADER_DIR)
  install(FILES ${HEADER} DESTINATION include/${HEADER_DIR})
endforeach(HEADER)

execute_process(COMMAND bash "-c" "echo -n \"${rclcpp_LINK_DIRECTORIES}\" | sed -n -e 's/^.*\\(\\/opt\\)/\\1/p'" OUTPUT_VARIABLE PKGCONF_ROS_LIBLINK_PATH)
set(PKGCONF_ROS_LIBLINK "-L${PKGCONF_ROS_LIBLINK_PATH}")
set(PKGCONF_LD_FLAGS ${LD_FLAGS})
configure_file(tyndall.pc.in tyndall.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tyndall.pc DESTINATION lib/pkgconfig)

set(EXAMPLE_PREFIX "${CMAKE_PROJECT_NAME}_ex_")
file(GLOB EXAMPLES "examples/*.c*")

foreach(EXAMPLE ${EXAMPLES})
  execute_process(COMMAND bash "-c" "printf ${EXAMPLE_PREFIX}$(basename ${EXAMPLE} | cut -f 1 -d '.')" OUTPUT_VARIABLE EXAMPLE_TARGET)
  add_executable(${EXAMPLE_TARGET} EXCLUDE_FROM_ALL ${EXAMPLE})
  target_link_libraries(${EXAMPLE_TARGET} tyndall)
  install(TARGETS ${EXAMPLE_TARGET} DESTINATION bin OPTIONAL)
  set(EXAMPLE_TARGETS "${EXAMPLE_TARGETS}${EXAMPLE_TARGET};")
endforeach(EXAMPLE)

file(GLOB SUBEXAMPLES "examples/*/*.c*")

foreach(EXAMPLE ${SUBEXAMPLES})
  execute_process(COMMAND bash "-c" "printf ${EXAMPLE_PREFIX}$(dirname ${EXAMPLE} | xargs basename)_$(basename ${EXAMPLE} | cut -f 1 -d '.')" OUTPUT_VARIABLE EXAMPLE_TARGET)
  add_executable(${EXAMPLE_TARGET} EXCLUDE_FROM_ALL ${EXAMPLE})
  target_link_libraries(${EXAMPLE_TARGET} tyndall)
  install(TARGETS ${EXAMPLE_TARGET} DESTINATION bin OPTIONAL)
  set(EXAMPLE_TARGETS "${EXAMPLE_TARGETS}${EXAMPLE_TARGET};")
endforeach(EXAMPLE)

add_custom_target(examples)
add_dependencies(examples ${EXAMPLE_TARGETS})

set(TEST_PREFIX "${CMAKE_PROJECT_NAME}_test_")
file(GLOB CPP_TESTS "tests/*/*.cpp")

foreach(CPP_TEST ${CPP_TESTS})
  execute_process(COMMAND bash "-c" "printf ${TEST_PREFIX}$(dirname ${CPP_TEST} | xargs basename)_$(basename ${CPP_TEST} | cut -f 1 -d '.')" OUTPUT_VARIABLE CPP_TEST_TARGET)
  add_executable(${CPP_TEST_TARGET} EXCLUDE_FROM_ALL ${CPP_TEST})
  target_link_libraries(${CPP_TEST_TARGET} tyndall)
  install(TARGETS ${CPP_TEST_TARGET} DESTINATION bin OPTIONAL)
  set(CPP_TEST_TARGETS "${CPP_TEST_TARGETS}${CPP_TEST_TARGET};")
endforeach(CPP_TEST)

IF(CMAKE_CROSSCOMPILING)
  set(TEST_CMD "echo WARNING: Tests are not executed when cross compiling")
ELSE()
  set(TEST_CMD "set -e; for t in ${CMAKE_BINARY_DIR}/${TEST_PREFIX}*; do $t; done")
ENDIF()

add_custom_target(tests COMMAND /bin/bash -c "${TEST_CMD}" VERBATIM DEPENDS ${CPP_TEST_TARGETS})

set(TOOL_PREFIX "${CMAKE_PROJECT_NAME}_tool_")
file(GLOB TOOLS "tools/*.c*")

foreach(TOOL ${TOOLS})
  execute_process(COMMAND bash "-c" "printf ${TOOL_PREFIX}$(basename ${TOOL} | cut -f 1 -d '.')" OUTPUT_VARIABLE TOOL_TARGET)
  add_executable(${TOOL_TARGET} EXCLUDE_FROM_ALL ${TOOL})
  target_link_libraries(${TOOL_TARGET} tyndall)
  install(TARGETS ${TOOL_TARGET} DESTINATION bin OPTIONAL)
  set(TOOL_TARGETS "${TOOL_TARGETS}${TOOL_TARGET};")
endforeach(TOOL)

add_custom_target(tools)
add_dependencies(tools ${TOOL_TARGETS})

function(dump_cmake_variables)
  get_cmake_property(_variableNames VARIABLES)
  list(SORT _variableNames)

  foreach(_variableName ${_variableNames})
    if(ARGV0)
      unset(MATCHED)
      string(REGEX MATCH ${ARGV0} MATCHED ${_variableName})

      if(NOT MATCHED)
        continue()
      endif()
    endif()

    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
endfunction()

# dump_cmake_variables()