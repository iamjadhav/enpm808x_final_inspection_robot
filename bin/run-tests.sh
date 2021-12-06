#!/bin/bash

######################################################################################
# Run this script in the root of the project workspace e.g. /path/to/tiago_ws
######################################################################################

# Error out if anything fails
set -e

# Run the tests for our package, and set the return value of this script
# as a function of those results with catkin_test_results
catkin_make_isolated --pkg enpm808x_final_inspection_robot --catkin-make-args run_tests && catkin_test_results
