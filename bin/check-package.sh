#!/usr/bin/env bash

###############################################################################
# Run this script in the root of this package
###############################################################################

###############################################################################
# This script will help to run necessary checks easily. This does:
# - cppcheck on all C++ files
# - cpplint on all C++ files
# This is inspired by https://github.com/rnvandemark/enpm808x_midterm_path_planner/blob/master/bin/check-workspace.sh
###############################################################################

# Perform cppcheck on all C++ source and header files
echo "********************"
echo "******CPPCHECK******"
echo "********************"
echo
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem src/*.cpp include/enpm808x_final_inspection_robot/*.hpp

# Perform cpplint on all C++ source and header files
echo
echo "********************"
echo "*******CPPLINT******"
echo "********************"
echo
cpplint src/*.cpp include/enpm808x_final_inspection_robot/*.hpp
