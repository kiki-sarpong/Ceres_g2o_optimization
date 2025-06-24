cmake_minimum_required(VERSION 3.20)
project(PoseGraphCeres)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(G2O REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


include_directories(
    include
    /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/
    /opt/homebrew/Cellar/g2o/20241228/include  # this is isn't built with csparse
)

link_directories(/opt/homebrew/Cellar/g2o/20241228/lib)

add_executable(${PROJECT_NAME} 
src/main.cpp 
src/LoadData.cpp
src/Constraints.cpp 
src/Poses.cpp
src/PoseGraphOptimization.cpp 
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen Ceres::ceres glog::glog)


add_executable(slam_2d_posegraph src/OptimizerG2O.cpp)
target_link_libraries(slam_2d_posegraph 
g2o_core
g2o_stuff
# g2o_solver_cholmod  # Couldn't get the cholmod solver to link
g2o_solver_dense
g2o_types_sba
g2o_types_slam3d
g2o_types_slam2d
)