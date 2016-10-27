
cmake_minimum_required(VERSION 2.8)

project(p4model_cmd)

#if(APPLE)
#set(CMAKE_CXX_COMPILER /usr/bin/clang++)
#else()
#set(CMAKE_CXX_COMPILER g++)
#endif()

include_directories(
  ${mfoundation_SOURCE_DIR}/h/mfoundation
  ${p4model_SOURCE_DIR}/h/p4model
)

list(APPEND DEPENDENCIES mfoundation)
if(${XENOMAI})
  list(APPEND DEPENDENCIES ${XENO_DEPENDENCIES})
endif()

#link_libraries(p4model)

file(GLOB SRCS "${PROJECT_SOURCE_DIR}/p4model_chg.c")
add_executable(p4model_chg ${SRCS})
target_link_libraries(p4model_chg p4model ${DEPENDENCIES})
#add_dependencies(p4model_chg p4model)

file(GLOB SRCS "${PROJECT_SOURCE_DIR}/p4model_add_sensor.c")
add_executable(p4model_add_sensor ${SRCS})
target_link_libraries(p4model_add_sensor p4model ${DEPENDENCIES})

file(GLOB SRCS "${PROJECT_SOURCE_DIR}/p4model_add_dummy_rotary_links.c")
add_executable(p4model_add_dummy_rotary_links ${SRCS})
target_link_libraries(p4model_add_dummy_rotary_links p4model ${DEPENDENCIES})

if(${UNIX})
    install(TARGETS p4model_chg p4model_add_sensor p4model_add_dummy_rotary_links RUNTIME DESTINATION bin)
elseif(${MINGW})
    #install(FILES "hook/bell/actual_arm/ihook.txt" DESTINATION ${INSTALL_DIR})
    install(TARGETS p4model_chg p4model_add_sensor p4model_add_dummy_rotary_links RUNTIME DESTINATION ${INSTALL_DIR})
endif()

