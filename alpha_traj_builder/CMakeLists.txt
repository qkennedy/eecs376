cmake_minimum_required(VERSION 2.8.3)
project(alpha_traj_builder)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(alpha_traj_builder src/traj_builder.cpp)   

cs_add_executable(alpha_traj_builder_example_main src/traj_builder_example_main.cpp)

target_link_libraries(alpha_traj_builder_example_main alpha_traj_builder)

cs_install()
cs_export()
    
