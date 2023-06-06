#!/usr/bin/bash
# bazel build :image_stabilizer
# bazel run :image_stabilizer
mkdir -p build && cd build && cmake .. && make -j4
