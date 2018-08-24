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
