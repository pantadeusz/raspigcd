cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

project (raspigcd CXX)

set (raspigcd_VERSION_MAJOR 0)
set (raspigcd_VERSION_MINOR 1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

add_definitions(-Wall)

install(DIRECTORY src/ DESTINATION include/raspigcd
        FILES_MATCHING PATTERN "*.hpp")


FIND_PACKAGE( OpenMP )
  if(OPENMP_FOUND)
     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
include_directories(SYSTEM ${OpenMP_INCLUDE_PATH})


execute_process( COMMAND wget -qO "${CMAKE_BINARY_DIR}/getdeps.js" "https://raw.githubusercontent.com/pantadeusz/cppdepsystem/master/getdeps.js" )
execute_process( COMMAND cp "${PROJECT_SOURCE_DIR}/getdeps.json" "${CMAKE_BINARY_DIR}/getdeps.json" )
execute_process( COMMAND node "${CMAKE_BINARY_DIR}/getdeps.js" "--list"
    OUTPUT_VARIABLE DEPSLIST
)
string(REGEX REPLACE "\n$" "" DEPSLIST "${DEPSLIST}")
foreach(dep ${DEPSLIST})
    include_directories("${CMAKE_BINARY_DIR}/.cppdeps/${dep}/include")
    link_directories("${CMAKE_BINARY_DIR}/.cppdeps/${dep}/lib")
endforeach()
add_custom_target( cppdeps
    COMMAND node ${CMAKE_BINARY_DIR}/getdeps.js "${CMAKE_BINARY_DIR}/.cppdeps" 
)



include_directories("${PROJECT_SOURCE_DIR}/src")

set(app_SOURCES "a_main.cpp" )
file(GLOB_RECURSE lib_SOURCES "src/*.cpp")
file(GLOB_RECURSE app_TESTS "tests/*.cpp")

add_library(raspigcd SHARED ${lib_SOURCES})
add_executable(app ${app_SOURCES})
add_executable(app_test EXCLUDE_FROM_ALL ${app_TESTS} ) 

install (TARGETS raspigcd DESTINATION lib)
target_link_libraries(app raspigcd libtpcommon.so)
target_link_libraries(app_test raspigcd libtpcommon.so)


add_dependencies(raspigcd cppdeps)
add_dependencies(app cppdeps)
add_dependencies(app_test cppdeps)




enable_testing()


add_test( NAME app_test 
          COMMAND "${CMAKE_BINARY_DIR}/app_test"
          WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"  )


add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS app_test)


SET(CPACK_PACKAGE_VERSION_MAJOR ${raspigcd_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${raspigcd_VERSION_MINOR})
# SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
# SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_GENERATOR "DEB" "TGZ")

SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Tadeusz Puźniakowski")
INCLUDE(CPack)


 
# To use this:
# make package

