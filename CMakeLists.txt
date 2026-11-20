cmake_minimum_required(VERSION 3.8)
project(bigmap_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# bigmap_bringup.dem: how the DEM of a place is made, also from the world of a simulation
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/wait_for_robot.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the parameter files against each other and against the code that reads them
  ament_add_pytest_test(config_files test/test_config.py)
  # how the DEM is made (no process is started)
  ament_add_pytest_test(dem test/test_dem.py)
  # the decisions of the launch file (no process is started)
  ament_add_pytest_test(launch_logic test/test_launch.py)
  # the simulation of a place and the application in one launch (no process is started)
  ament_add_pytest_test(geo_launch test/test_geo_launch.py)
  ament_add_pytest_test(wait_for_robot test/test_wait_for_robot.py)
endif()

ament_package()
