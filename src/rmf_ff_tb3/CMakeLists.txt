cmake_minimum_required(VERSION 3.8)

project(rmf_ff_tb3)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  models
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
