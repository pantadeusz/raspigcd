cmake_minimum_required(VERSION 3.6.0 FATAL_ERROR)

project (raspigcd2 CXX)

set (raspigcd2_VERSION_MAJOR 0)
set (raspigcd2_VERSION_MINOR 1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  #set(CMAKE_BUILD_TYPE Debug)
endif()

include(CheckLibraryExists)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb ")
# consider also -flto
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -frename-registers -funroll-loops -DNDEBUG")

add_definitions(-Wall)


install(DIRECTORY include/ DESTINATION include/raspigcd2
        FILES_MATCHING PATTERN "*.hpp")

find_package(Catch2)
find_package(Git)
find_package(Threads)
find_package(Doxygen 
             OPTIONAL_COMPONENTS mscgen dia dot)

find_package(SDL2)
if(SDL2_FOUND)
  add_definitions(-DHAVE_SDL2)
endif()


# FIND_PACKAGE( OpenMP )
#   if(OPENMP_FOUND)
#      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#   endif()
# include_directories(SYSTEM ${OpenMP_INCLUDE_PATH})


include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/thirdparty")


set(raspigcd2_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/raspigcd.cpp )
file(GLOB_RECURSE lib_SOURCES "src/*.cpp" "src/*/*.cpp")
# file(GLOB_RECURSE raspigcd2_TESTS "tests/*.cpp")
list(REMOVE_ITEM lib_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/raspigcd.cpp)

add_library(raspigcd2 SHARED ${lib_SOURCES})
add_executable(gcd ${raspigcd2_SOURCES} ${lib_SOURCES})
target_link_libraries(gcd ${CMAKE_THREAD_LIBS_INIT})

if(SDL2_FOUND)
  include_directories(${SDL2_INCLUDE_DIRS})
  target_link_libraries(gcd ${SDL2_LIBRARIES})
endif()

install (TARGETS raspigcd2 DESTINATION lib)
install (TARGETS gcd DESTINATION bin)

## file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/tests/*_test.cpp")
## foreach(file ${files})
##   get_filename_component(fn_target ${file}
##                        NAME)
##   add_executable(${fn_target} ${file})
##   target_link_libraries(${fn_target} raspigcd2 ${CMAKE_THREAD_LIBS_INIT})
##   add_test(NAME ${fn_target} COMMAND "${CMAKE_BINARY_DIR}/${fn_target}" WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" )
## endforeach()

file(GLOB_RECURSE tests_SOURCES "${PROJECT_SOURCE_DIR}/tests/*_test.cpp" "${PROJECT_SOURCE_DIR}/tests/*/*_test.cpp")
add_executable(tests ${tests_SOURCES} "tests/tests.cpp" "thirdparty/lodepng/lodepng.cpp" )
target_link_libraries(tests raspigcd2 ${CMAKE_THREAD_LIBS_INIT}  Catch2::Catch2)
include_directories("${PROJECT_SOURCE_DIR}/tests")
# add_test(NAME ${fn_target} COMMAND "${CMAKE_BINARY_DIR}/${fn_target}" WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" )

if(Doxygen_FOUND)
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_GENERATE_MAN YES)
set(DOXYGEN_RECURSIVE YES)
set(CALL_GRAPH YES)
set(CALLER_GRAPH YES)
set(DOXYGEN_EXCLUDE_PATTERNS
*/thirdparty/*
*/__old_experiments_01/*
*/.git/*
*/.svn/*
*/.hg/*
*/CMakeFiles/*
*/_CPack_Packages/*
DartConfiguration.tcl
CMakeLists.txt
CMakeCache.txt
)

doxygen_add_docs(
    doxygen
    "${PROJECT_SOURCE_DIR}"
    COMMENT "Generate man pages"
)
endif()


if(Catch2_FOUND)
  include(CTest)
  include(Catch)
  catch_discover_tests(tests)
endif()

SET(CPACK_PACKAGE_VERSION_MAJOR ${raspigcd2_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${raspigcd2_VERSION_MINOR})
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_GENERATOR "DEB" "TGZ")

SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Tadeusz Puźniakowski")
INCLUDE(CPack)
 
# To use this:
# make package

