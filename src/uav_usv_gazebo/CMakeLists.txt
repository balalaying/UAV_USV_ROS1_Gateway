cmake_minimum_required(VERSION 3.16)
project(uav_usv_gazebo)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_SIM REQUIRED gz-sim8)
pkg_check_modules(GZ_PLUGIN REQUIRED gz-plugin2)
pkg_check_modules(GZ_MATH REQUIRED gz-math7)
pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)
pkg_check_modules(GZ_COMMON REQUIRED gz-common5)
pkg_check_modules(GZ_RENDERING REQUIRED gz-rendering8)

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

# VRX-compatible spectral wavefield and visualisation.  These targets are
# kept separate from the legacy visual-only Gerstner model so the experimental
# water world can be run without changing the default worlds.
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
  VrxWavefield
  VrxWaveVisual
  VrxWavefieldPublisher
  LIBRARY DESTINATION lib/${PROJECT_NAME}/plugins
)

install(PROGRAMS
  tools/run_gz_world.sh
  tools/prepare_coastline.sh
  tools/build_coastline_assets.py
  tools/prepare_fleet_mid360.py
  tools/prepare_fft_preview_assets.sh
  tools/run_fft_water_preview.sh
  tools/run_vrx_water_preview.sh
  tools/sync_to_px4.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  models
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
