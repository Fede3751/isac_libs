cmake_minimum_required(VERSION 3.10)
project(isac_libs_resources)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ignition-cmake2 REQUIRED)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-sensors6 REQUIRED)


set(IGN_SENSORS_VER ${ignition-sensors6_VERSION_MAJOR})# or ignition-gazebo7 / gazebo
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_library(ran_antenna SHARED plugins/RanAntenna.cpp)
# target_link_libraries(ran_antenna
#   ignition-gazebo6::core
# )


# # Optional but recommended: set the output library name explicitly
# set_target_properties(ran_antenna_plugin PROPERTIES
#     OUTPUT_NAME "ran_antenna"
# )

# # Install the plugin library
# install(TARGETS ran_antenna
#     LIBRARY DESTINATION lib
# )



add_library(ISACAntenna SHARED plugins/ISACAntenna.cc)
target_link_libraries(ISACAntenna
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PRIVATE ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
)
target_include_directories(ISACAntenna
    PUBLIC plugins)


install(TARGETS ISACAntenna
    LIBRARY DESTINATION lib
)


add_library(ISACAntennaSystem SHARED plugins/ISACAntennaSystem.cc)
target_link_libraries(ISACAntennaSystem
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PRIVATE ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
  ISACAntenna
)
target_include_directories(ISACAntennaSystem
    PUBLIC plugins)


install(TARGETS ISACAntennaSystem
    LIBRARY DESTINATION lib
)

ament_environment_hooks(hooks/ign_plugin_path.sh.in)
install(FILES hooks/ign_plugin_path.sh.in
  DESTINATION share/${PROJECT_NAME}/environment
)


install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)


install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
