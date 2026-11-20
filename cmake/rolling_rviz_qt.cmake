# Build workaround for the ROS Rolling packages released for Ubuntu 24.04 (no longer updated).
#
# There rviz_common is built against Qt5, but (1) its exported dependency list names plain "Qt5",
# which the system Qt5Config.cmake refuses ("requires at least one component"), and (2) its
# rviz_common-extras.cmake looks for Qt6 first, which gets found when qt6-base-dev is installed
# next to qtbase5-dev, and then clashes with Qt5. Finding Qt5 with a component first solves (1) and
# hiding Qt6 from CMake solves (2). No third-party source needs editing.
#
# Only needed for packages that find rviz_common (navmap_rviz_plugin), and only on Rolling:
#   colcon build ... --cmake-args \
#     -DCMAKE_PROJECT_navmap_rviz_plugin_INCLUDE=<path to this file>
set(CMAKE_DISABLE_FIND_PACKAGE_Qt6 ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
