
cmake_minimum_required(VERSION 2.8)

project(p4model)

#if(APPLE)
#set(CMAKE_CXX_COMPILER /usr/bin/clang++)
#else()
#set(CMAKE_CXX_COMPILER g++)
#endif()

find_package(Eigen3 REQUIRED)

#add_definitions(-std=c++11 -Wall -DdDOUBLE -O2 -g)
if(${CMAKE_COMPILER_IS_GNUCC})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2 -std=gnu99")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -std=c++11")
endif()


include_directories(
  ${PROJECT_SOURCE_DIR}/h/${PROJECT_NAME}
  ${mfoundation_SOURCE_DIR}/h/mfoundation
  #/usr/include/c++/4.8/
  #/usr/include/x86_64-linux-gnu/c++/4.8/
  .
)

#add_definitions(-D_DEBUG)

file(GLOB CXXSRCS "${PROJECT_SOURCE_DIR}/src/*.cc")
file(GLOB CSRCS "${PROJECT_SOURCE_DIR}/src/*.c")

add_library (${PROJECT_NAME} SHARED ${CSRCS} ${CXXSRCS})

add_subdirectory(exec)

if(${UNIX})
    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/h/${PROJECT_NAME} DESTINATION include)
elseif(${MINGW})
    install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${INSTALL_DIR})
endif()


