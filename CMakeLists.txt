cmake_minimum_required(VERSION 3.12)
project(image_stabilizer)

set(CMAKE_CXX_STANDARD 11)
######
# CROSS TOOL CHAIN

# Set the target architecture for cross-compilation
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR arm)

# # Set the cross-compilation toolchain file
# set(CMAKE_C_COMPILER "path/to/your/cross-compiler/arm-linux-gnueabihf-gcc")
# set(CMAKE_CXX_COMPILER "path/to/your/cross-compiler/arm-linux-gnueabihf-g++")
# set(CMAKE_FIND_ROOT_PATH "path/to/your/sysroot")

# # Set the output directory for the compiled binary
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#####
# Find and include OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find the libiio library
find_library(LIBIIO_LIBRARY NAMES iio)

# Add the include directory for libiio
include_directories(${CMAKE_SOURCE_DIR}/include)  # Replace with the correct path to the iio.h header file

# Add the source files for your project
set(SOURCES
    image_stabilizer.cpp
    image_stabilizer.h
    imu_iio.cpp
    imu_iio.h
)

# Create the executable target
add_executable(image_stabilizer ${SOURCES})

# Link against OpenCV libraries
target_link_libraries(image_stabilizer ${LIBIIO_LIBRARY} ${OpenCV_LIBRARIES})
