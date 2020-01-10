cmake_minimum_required(VERSION 3.6.0 FATAL_ERROR)

project (raspigcd2 CXX)

set (raspigcd2_VERSION_MAJOR 2)
set (raspigcd2_VERSION_MINOR 0.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  #set(CMAKE_BUILD_TYPE Debug)
endif()

include(CheckLibraryExists)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb ")
# consider also -flto
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -frename-registers -funroll-loops -DNDEBUG")

add_definitions(-Wall)

if (NOT EXISTS "${PROJECT_SOURCE_DIR}/thirdparty/distance_t.hpp")
    file(DOWNLOAD "https://github.com/pantadeusz/distance_t/releases/download/0.1.1/distance_t.hpp" "${PROJECT_SOURCE_DIR}/thirdparty/distance_t.hpp")
endif()

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


include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/thirdparty")


set(raspigcd2_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/raspigcd.cpp )
file(GLOB_RECURSE lib_SOURCES "src/*.cpp" "src/*/*.cpp")
# file(GLOB_RECURSE raspigcd2_TESTS "tests/*.cpp")
list(REMOVE_ITEM lib_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/raspigcd.cpp)

add_library(raspigcd2 SHARED ${lib_SOURCES})
if(SDL2_FOUND)
  include_directories(${SDL2_INCLUDE_DIRS})
  target_link_libraries(raspigcd2 ${SDL2_LIBRARIES})
endif()
add_executable(gcd ${raspigcd2_SOURCES} ${lib_SOURCES})
target_link_libraries(gcd ${CMAKE_THREAD_LIBS_INIT})

if(SDL2_FOUND)
  include_directories(${SDL2_INCLUDE_DIRS})
  target_link_libraries(gcd ${SDL2_LIBRARIES})
endif()

SET(PKG_CONFIG_REQUIRES glib-2.0)
SET(PKG_CONFIG_LIBDIR
    "\${prefix}/lib"
)
SET(PKG_CONFIG_INCLUDEDIR
    "\${prefix}/include/raspigcd2"
)
SET(PKG_CONFIG_LIBS
    "-L\${libdir} -lraspigcd2"
)
SET(PKG_CONFIG_CFLAGS
    "-I\${includedir}"
)

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
)



install (TARGETS raspigcd2 DESTINATION lib)
install (TARGETS gcd DESTINATION bin)
install (FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
DESTINATION lib/pkgconfig)




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

