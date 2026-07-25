cmake_minimum_required(VERSION 3.8)
project(inspectra_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Explicit executable install as a safety net — USE_SOURCE_PERMISSIONS above
# should already carry the +x bit, but this guarantees it regardless.
install(PROGRAMS urdf/generate_panda_urdf.sh
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
