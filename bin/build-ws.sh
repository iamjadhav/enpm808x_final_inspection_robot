#!/bin/bash

######################################################################################
# Run this script in the root of the project workspace e.g. /path/to/tiago_ws
######################################################################################

# Error out if anything fails
set -e

# If building with limited resources/hardware, first build larger pkgs with -j1
# This is triggered with a '-l' flag
if [[ 1 -le $# && "-l" == $1 ]]; then
    catkin_make_isolated -j1 \
	                     --install \
                         --only-pkg-with-deps gazebo_plugins tiago_pcl_tutorial \
					     --cmake-args "-DCATKIN_ENABLE_TESTING=0"
fi

# Pass the -o option to only build this package
if [[ ! ( 1 -le $# && "-o" == $1 ) ]]; then
    # Build all packages except 'enpm808x_final_inspection_robot' and do NOT build their tests
    catkin_make_isolated --install \
                         --ignore-pkg enpm808x_final_inspection_robot \
                         --cmake-args "-DCATKIN_ENABLE_TESTING=0"
fi

# Build only the 'enpm808x_final_inspection_robot' package and build its tests
catkin_make_isolated --install \
                     --pkg enpm808x_final_inspection_robot \
					 --cmake-args "-DCATKIN_ENABLE_TESTING=1"
