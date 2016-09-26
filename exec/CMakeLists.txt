
cmake_minimum_required(VERSION 2.8)

project(p4model_cmd)

#if(APPLE)
#set(CMAKE_CXX_COMPILER /usr/bin/clang++)
#else()
#set(CMAKE_CXX_COMPILER g++)
#endif()

add_definitions(-std=gnu99 -Wall -O2 -g)

include_directories(
  ${mfoundation_SOURCE_DIR}/h/mfoundation
  ${p4model_SOURCE_DIR}/h/p4model
)

#link_libraries(p4model)

file(GLOB SRCS "${PROJECT_SOURCE_DIR}/p4model_chg.c")
add_executable(p4model_chg ${SRCS})
target_link_libraries(p4model_chg p4model)
#add_dependencies(p4model_chg p4model)

file(GLOB SRCS "${PROJECT_SOURCE_DIR}/p4model_add_sensor.c")
add_executable(p4model_add_sensor ${SRCS})
target_link_libraries(p4model_add_sensor p4model)

if(${UNIX})
    install(TARGETS p4model_chg p4model_add_sensor RUNTIME DESTINATION bin)
elseif(${MINGW})
    #install(FILES "hook/bell/actual_arm/ihook.txt" DESTINATION ${INSTALL_DIR})
    install(TARGETS p4model_chg p4model_add_sensor RUNTIME DESTINATION ${INSTALL_DIR})
endif()

