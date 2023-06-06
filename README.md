In this simple project, I have created a demonstration for the Raspberry Pi to capture images from a camera module and perform Electronic Image Stabilization (EIS) using data from an IMU (gyroscopes and acceleration sensor) connected to the Raspberry Pi board via the SPI interface.

The diagram below illustrates the setup:
        SPI
RP board --------------> IMU (sparkfun LSM6DSV16X board)
The Linux kernel on the Raspberry Pi already supports the IIO driver for LSM6DSX, which also compatible with LSM6DSV16X sensor. The IMU folder contains the source code for the Linux driver.

The idea is not new, as the Raspberry Pi board captures the image simultaneously with the IMU. The captured data is then used for the image stabilization process, with the expectation of reducing blurring in the output image. It should be noted that this implementation is simplified, and the main challenge lies in synchronizing the IMU data with the captured image in terms of timing.

This code focuses on single-frame EIS and does not handle multiple frames.
=====================
In order to use this code you need to config the correct IIO device in main function,
you also need the proper kernel which support iio for this imu
=====================
HOW TO BUILD
for get about bazel build it doesnt work yet
1. you need to install opencv first
https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
2. you need libiio 
https://github.com/analogdevicesinc/libiio
or sudo apt install libiio-dev
3. after that just creat build folder and build then run
mkdir -p build && cd build && cmake .. && make -j4

=====================
And finally I haven't test bc i'm too broke to buy the IMU board so it's still in progress