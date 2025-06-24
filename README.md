

### Project Description

This project demonstrates 2D/3D optimization with ceres and g2o.

This project is a followup to my visual odometry project here:
https://github.com/kiki-sarpong/visual_odometry

The dataset used for the 2d odometry problems were from:
https://github.com/OpenSLAM-org/openslam_vertigo/tree/master

The "deformed" 3d pose graph was generated with G2O.
G2O optimization is used here for only 2D problems.(3D ones were already done with CERES)


### RUN
```
mkdir build
cd build
cmake ..
cmake --build .
./../bin/PoseGraphCeres ../dataset/sphere.g2o   # For ceres 3D
./../bin/slam_2d_posegraph ../dataset/ringCity.g2o  # For g2o 2D

# G2O viewer is available through the dockerfile.
# copy the optimized results into the docker_g2o dir
# Build a docker container, and run the g2o_viewer to visualize results.
# Use x11 forwarding to visualize the GUI

```


Sample of 3D posegraph optimization with ceres:
```shell
I20250623 09:19:26.822774 223047542 LoadData.cpp:19] Loading data to be optimized...
I20250623 09:19:26.913645 223047542 LoadData.cpp:46] Data Loading complete.
I20250623 09:19:26.913667 223047542 LoadData.cpp:50] Starting ceres optimizer ....
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  4.772379e+09    0.00e+00    1.58e+06   0.00e+00   0.00e+00  1.00e+04        0    1.46e+00    1.47e+00
   1  2.262672e+08    4.55e+09    3.60e+05   0.00e+00   9.53e-01  3.00e+04        1    1.54e+00    3.01e+00
   2  1.270530e+06    2.25e+08    2.36e+04   3.42e+02   9.95e-01  9.00e+04        1    1.53e+00    4.54e+00
   3  7.829917e+04    1.19e+06    1.16e+03   1.02e+02   1.00e+00  2.70e+05        1    1.53e+00    6.07e+00
   4  7.441786e+04    3.88e+03    2.80e+02   6.98e+01   1.00e+00  8.10e+05        1    1.53e+00    7.59e+00
   5  7.364500e+04    7.73e+02    3.72e+01   7.94e+01   9.71e-01  2.43e+06        1    1.53e+00    9.13e+00
   6  7.299000e+04    6.55e+02    1.06e+02   2.08e+02   4.40e-01  2.43e+06        1    1.53e+00    1.07e+01
   7  7.144054e+04    1.55e+03    8.08e+01   1.90e+02   7.58e-01  2.81e+06        1    1.53e+00    1.22e+01
   8  7.034563e+04    1.09e+03    8.70e+01   2.00e+02   6.60e-01  2.91e+06        1    1.53e+00    1.37e+01
   9  6.921258e+04    1.13e+03    7.66e+01   1.88e+02   7.26e-01  3.20e+06        1    1.53e+00    1.52e+01
  10  6.829885e+04    9.14e+02    7.58e+01   1.88e+02   6.87e-01  3.38e+06        1    1.53e+00    1.68e+01
  11  6.744817e+04    8.51e+02    6.86e+01   1.78e+02   7.16e-01  3.67e+06        1    1.53e+00    1.83e+01
  12  6.673782e+04    7.10e+02    6.49e+01   1.73e+02   7.03e-01  3.94e+06        1    1.53e+00    1.98e+01
  13  6.611361e+04    6.24e+02    5.87e+01   1.64e+02   7.19e-01  4.30e+06        1    1.53e+00    2.13e+01
  14  6.559464e+04    5.19e+02    5.38e+01   1.57e+02   7.17e-01  4.68e+06        1    1.53e+00    2.29e+01
  15  6.515719e+04    4.37e+02    4.81e+01   1.49e+02   7.28e-01  5.17e+06        1    1.53e+00    2.44e+01
  16  6.480192e+04    3.55e+02    4.30e+01   1.41e+02   7.31e-01  5.74e+06        1    1.53e+00    2.59e+01
  17  6.451483e+04    2.87e+02    3.76e+01   1.31e+02   7.42e-01  6.47e+06        1    1.53e+00    2.75e+01
  18  6.429085e+04    2.24e+02    3.26e+01   1.22e+02   7.49e-01  7.38e+06        1    1.54e+00    2.90e+01
  19  6.411960e+04    1.71e+02    2.75e+01   1.13e+02   7.61e-01  8.60e+06        1    1.53e+00    3.05e+01
  20  6.399434e+04    1.25e+02    2.28e+01   1.03e+02   7.72e-01  1.02e+07        1    1.53e+00    3.20e+01
  21  6.390644e+04    8.79e+01    1.82e+01   9.17e+01   7.88e-01  1.27e+07        1    1.53e+00    3.36e+01
  22  6.384895e+04    5.75e+01    1.40e+01   8.03e+01   8.05e-01  1.64e+07        1    1.53e+00    3.51e+01
  23  6.381446e+04    3.45e+01    9.93e+00   6.78e+01   8.30e-01  2.30e+07        1    1.53e+00    3.66e+01
  24  6.379654e+04    1.79e+01    6.39e+00   5.45e+01   8.59e-01  3.65e+07        1    1.53e+00    3.82e+01
  25  6.378909e+04    7.44e+00    3.37e+00   3.96e+01   9.01e-01  7.51e+07        1    1.53e+00    3.97e+01
  26  6.378709e+04    2.01e+00    2.00e+00   2.37e+01   9.50e-01  2.25e+08        1    1.53e+00    4.12e+01
  27  6.378687e+04    2.22e-01    1.97e+00   8.56e+00   9.92e-01  6.76e+08        1    1.53e+00    4.27e+01
I20250623 09:20:09.840855 223047542 PoseGraphOptimization.cpp:44] Ceres optimizer summary:
I20250623 09:20:09.841732 223047542 PoseGraphOptimization.cpp:45] Ceres Solver Report: Iterations: 28, Initial cost: 4.772379e+09, Final cost: 6.378687e+04, Termination: CONVERGENCE
I20250623 09:20:09.843828 223047542 LoadData.cpp:52] Ceres optimizer complete!
I20250623 09:20:09.843848 223047542 LoadData.cpp:56] Writing data output to file: ../dataset/optimized_result.g2o
I20250623 09:20:09.929598 223047542 LoadData.cpp:69] Write complete.
I20250623 09:20:09.929620 223047542 main.cpp:39] Code execution took : 43.1069 seconds
I20250623 09:20:09.929637 223047542 LoadData.cpp:14] Number of total poses and constraints. Poses-> 2500 , constraints->  9799

```


### Results

Original sphere
<!-- ![image](https://github.com/kiki-sarpong/visual_odometry/blob/main/images/capture_kitti_03.PNG?raw=true) -->

Optimized sphere


Original ringcity odometry/map


Optimized ringcity odometry/map


Original manhattan odometry/map


Optimized manhattan odometry/map