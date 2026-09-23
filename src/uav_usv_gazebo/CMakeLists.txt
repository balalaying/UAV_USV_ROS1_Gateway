cmake_minimum_required(VERSION 3.16)
project(uav_usv_gazebo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_SIM REQUIRED gz-sim8)
pkg_check_modules(GZ_PLUGIN REQUIRED gz-plugin2)
pkg_check_modules(GZ_MATH REQUIRED gz-math7)
pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)
pkg_check_modules(GZ_COMMON REQUIRED gz-common5)
pkg_check_modules(GZ_RENDERING REQUIRED gz-rendering8)

catkin_package()

function(add_gz_system_plugin target source)
  add_library(${target} SHARED ${source})
  target_include_directories(${target} PRIVATE
    ${GZ_SIM_INCLUDE_DIRS}
    ${GZ_PLUGIN_INCLUDE_DIRS}
    ${GZ_MATH_INCLUDE_DIRS}
    ${GZ_TRANSPORT_INCLUDE_DIRS}
    ${GZ_MSGS_INCLUDE_DIRS}
  )
  target_link_directories(${target} PRIVATE
    ${GZ_SIM_LIBRARY_DIRS}
    ${GZ_PLUGIN_LIBRARY_DIRS}
    ${GZ_MATH_LIBRARY_DIRS}
    ${GZ_TRANSPORT_LIBRARY_DIRS}
    ${GZ_MSGS_LIBRARY_DIRS}
  )
  target_link_libraries(${target} PRIVATE
    ${GZ_SIM_LIBRARIES}
    ${GZ_PLUGIN_LIBRARIES}
    ${GZ_MATH_LIBRARIES}
    ${GZ_TRANSPORT_LIBRARIES}
    ${GZ_MSGS_LIBRARIES}
  )
endfunction()

add_gz_system_plugin(BoatWaveFollower plugins/BoatWaveFollower.cc)
add_gz_system_plugin(DroneDeckFollower plugins/DroneDeckFollower.cc)
add_gz_system_plugin(VisualPoseFollower plugins/VisualPoseFollower.cc)
add_gz_system_plugin(RandomVesselMotion plugins/RandomVesselMotion.cc)
add_gz_system_plugin(ArduPilotMotorBridge plugins/ArduPilotMotorBridge.cc)
add_gz_system_plugin(ArduPilotRoverBridge plugins/ArduPilotRoverBridge.cc)

set(ARDUPILOT_GAZEBO_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ardupilot_gazebo")
if(NOT EXISTS "${ARDUPILOT_GAZEBO_ROOT}/src/ArduPilotPlugin.cc")
  message(FATAL_ERROR
    "Missing official ArduPilot Gazebo source: ${ARDUPILOT_GAZEBO_ROOT}")
endif()
add_library(ArduPilotPlugin SHARED
  "${ARDUPILOT_GAZEBO_ROOT}/src/ArduPilotPlugin.cc"
  "${ARDUPILOT_GAZEBO_ROOT}/src/SocketUDP.cc"
  "${ARDUPILOT_GAZEBO_ROOT}/src/Util.cc"
)
target_include_directories(ArduPilotPlugin PRIVATE
  "${ARDUPILOT_GAZEBO_ROOT}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/rapidjson/include"
  ${GZ_SIM_INCLUDE_DIRS}
  ${GZ_PLUGIN_INCLUDE_DIRS}
  ${GZ_MATH_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
)
target_link_directories(ArduPilotPlugin PRIVATE
  ${GZ_SIM_LIBRARY_DIRS}
  ${GZ_PLUGIN_LIBRARY_DIRS}
  ${GZ_MATH_LIBRARY_DIRS}
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
)
target_link_libraries(ArduPilotPlugin PRIVATE
  ${GZ_SIM_LIBRARIES}
  ${GZ_PLUGIN_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)

add_library(VrxWavefield SHARED
  plugins/vrx_wavefield/Wavefield.cc
)
target_include_directories(VrxWavefield PRIVATE
  ${GZ_SIM_INCLUDE_DIRS}
  ${GZ_MATH_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
  ${GZ_COMMON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/vrx_wavefield
)
target_link_directories(VrxWavefield PRIVATE
  ${GZ_SIM_LIBRARY_DIRS}
  ${GZ_MATH_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
  ${GZ_COMMON_LIBRARY_DIRS}
)
target_link_libraries(VrxWavefield PRIVATE
  ${GZ_SIM_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
  ${GZ_COMMON_LIBRARIES}
)

add_library(VrxWaveVisual SHARED
  plugins/vrx_wavefield/WaveVisual.cc
)
target_include_directories(VrxWaveVisual PRIVATE
  ${GZ_SIM_INCLUDE_DIRS}
  ${GZ_MATH_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
  ${GZ_COMMON_INCLUDE_DIRS}
  ${GZ_RENDERING_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/vrx_wavefield
)
target_link_directories(VrxWaveVisual PRIVATE
  ${GZ_SIM_LIBRARY_DIRS}
  ${GZ_MATH_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
  ${GZ_COMMON_LIBRARY_DIRS}
  ${GZ_RENDERING_LIBRARY_DIRS}
  ${GZ_TRANSPORT_LIBRARY_DIRS}
)
target_link_libraries(VrxWaveVisual PRIVATE
  VrxWavefield
  ${GZ_SIM_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
  ${GZ_COMMON_LIBRARIES}
  ${GZ_RENDERING_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
)

add_library(VrxWavefieldPublisher SHARED
  plugins/vrx_wavefield/VrxWavefieldPublisher.cc
)
target_include_directories(VrxWavefieldPublisher PRIVATE
  ${GZ_SIM_INCLUDE_DIRS}
  ${GZ_MATH_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/vrx_wavefield
)
target_link_directories(VrxWavefieldPublisher PRIVATE
  ${GZ_SIM_LIBRARY_DIRS}
  ${GZ_MATH_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
  ${GZ_TRANSPORT_LIBRARY_DIRS}
)
target_link_libraries(VrxWavefieldPublisher PRIVATE
  VrxWavefield
  ${GZ_SIM_LIBRARIES}
  ${GZ_MATH_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
)

install(TARGETS
  BoatWaveFollower
  DroneDeckFollower
  VisualPoseFollower
  RandomVesselMotion
  ArduPilotMotorBridge
  ArduPilotRoverBridge
  ArduPilotPlugin
  VrxWavefield
  VrxWaveVisual
  VrxWavefieldPublisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/plugins
)

install(PROGRAMS
  tools/run_gz_world.sh
  tools/run_gz_world_mid360.sh
  tools/prepare_coastline.sh
  tools/build_coastline_assets.py
  tools/prepare_fleet_mid360.py
  tools/verify_repo_models.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  models
  worlds
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
