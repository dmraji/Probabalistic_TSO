# Probabalistic Tri-State Occupancy
- Lightweight pointcloud raycasting;
- Determine occupied, free & unknown space;
- Occupancy probability tracking for Ghosting reduction;
- Fully running on CPU;
- Benchmarked on Ubuntu 16.04 LTS Xenial 64-bit, Intel Core i7-4770 CPU @ 3.40GHz x 8;

- Built with cmake 3.5.1;
- Optimized with Valgrind 3.14.0;

## Inputs:
Pose and Pointcloud run-data in HDF5 file format

Default recognized filename: `RunData.h5`

## System Requirements
- C++11 or better
- CMake

## Dependencies
- H5Cpp library (comes standard on Ubuntu Xenial)
- Octomap (https://github.com/OctoMap/octomap)
- Sparsepp (https://github.com/greg7mdp/sparsepp)

## Description:
This project is a bi-implemented algorithm that seeks to accomplish two goals. The first of those is assigning occupancy probabilities to voxelized space. The second is to determine which voxels are unknown, meaning that they could exist but there is no data to say whether they do. Both of these goals require input data of a scene in the form of a SLAM-ed LiDAR scan as well as poses to go with it.

The method of this code is as follows:
- (0) Data read;
- (1) Perform raycasting with pose and SLAM-ed pointcloud from LiDAR scan;
- (2) Voxelize space;
- (3) Update data structures and occupancy probabilities;
- (4) Parse for unknown voxels;
- (5) Repeat #1-4 for each pose-scan;
- (6) Output data;
