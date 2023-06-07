#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <opencv2/opencv.hpp>
#include "image_stabilizer.h"
#include "imu_iio.h"
#include <iostream>
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>

// std::mutex mtx;  // Mutex for synchronization

// Define the gyro scale factor (convert raw gyro data to degrees per second)
double GYRO_SCALE_FACTOR = 1000.0f;

// Initialize video capture object
cv::VideoCapture cap;

// Initialize variables for motion estimation
cv::Mat prev_frame;
cv::Mat prev_gray;
cv::Vec3f prev_rotation(0, 0, 0);  // Initial rotation angles from gyroscope

void readImuData(IIO_IMU& imu, cv::Vec3f& gyro_data, cv::Vec3f& accel_data) {
    std::vector<double> gyro(3);
    std::vector<double> accel(3);
    
    if (imu.read_data(gyro, accel)) {
        gyro_data = cv::Vec3f(
            static_cast<float>(gyro[0]) / GYRO_SCALE_FACTOR,
            static_cast<float>(gyro[1]) / GYRO_SCALE_FACTOR,
            static_cast<float>(gyro[2]) / GYRO_SCALE_FACTOR
        );
        
        accel_data = cv::Vec3f(
            static_cast<float>(accel[0]),
            static_cast<float>(accel[1]),
            static_cast<float>(accel[2])
        );
    }
}

// Apply image stabilization using EIS algorithm
cv::Mat stabilizeImage_for_gyros(cv::Mat& frame, const cv::Vec3f& gyro_data)
{
    // Convert current frame to grayscale
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Estimate camera motion using gyroscope data
    cv::Vec3f current_rotation = gyro_data;
    cv::Vec3f motion = current_rotation - prev_rotation;
    prev_rotation = current_rotation;

    // Apply image transformation based on camera motion
    cv::Mat M = cv::getRotationMatrix2D(cv::Point2f(frame.cols / 2, frame.rows / 2), -motion[0], 1);
    cv::Mat transformed_frame;
    cv::warpAffine(frame, transformed_frame, M, frame.size());

    // Perform image stabilization using frame differencing
    cv::Mat diff;
    cv::absdiff(prev_gray, gray, diff);
    cv::Mat stabilized_frame;
    cv::addWeighted(frame, 1.5, transformed_frame, -0.5, 0, stabilized_frame);


    return stabilized_frame;
}


cv::Mat stabilizeImage_single_frame(cv::Mat& frame, const cv::Vec3f& gyro_data)
{
    // Convert current frame to grayscale
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Estimate camera motion using gyroscope data
    cv::Vec3f motion = gyro_data;

    // Apply image transformation based on camera motion
    cv::Mat M = cv::getRotationMatrix2D(cv::Point2f(frame.cols / 2, frame.rows / 2), -motion[0], 1);

    cv::Mat transformed_frame;
    cv::warpAffine(frame, transformed_frame, M, frame.size());
    cv::cvtColor(transformed_frame, transformed_frame, cv::COLOR_BGR2GRAY);

    // Resize the grayscale frame to match the dimensions of transformed_frame
    cv::resize(gray, gray, transformed_frame.size());

    // Check if the grayscale frames have the same dimensions
    if (gray.size() != transformed_frame.size()) {
        std::cerr << "Frame dimensions do not match" << std::endl;
        return cv::Mat();
    }

    // Perform image stabilization using frame differencing
    cv::Mat diff;
    cv::absdiff(gray, transformed_frame, diff);

    // Resize the original frame to match the size of transformed_frame
    cv::resize(frame, frame, transformed_frame.size());
    // std::cout<< "come here" <<std::endl;
    if (!frame.empty()) {
        int rows = frame.rows;
        int cols = frame.cols;
        int channels = frame.channels();

        std::cout << "Image shape: " << rows << " rows x " << cols << " columns x " << channels << " channels" << std::endl;
    }
    if (!transformed_frame.empty()) {
        int rows = transformed_frame.rows;
        int cols = transformed_frame.cols;
        int channels = transformed_frame.channels();

        std::cout << "Image shape: " << rows << " rows x " << cols << " columns x " << channels << " channels" << std::endl;
    }
    
    // Apply weighted addition to stabilize the frame
    // cv::Mat stabilized_frame;
    cv::Mat stabilized_frame = cv::Mat::zeros(frame.size(), frame.type());
    cv::addWeighted(gray, 1.5, transformed_frame, -0.5, 0, stabilized_frame);


    return stabilized_frame;
}



int main()
{
    // Create an instance of the IIO_IMU class
    //modify this device for your specific board/setting
    IIO_IMU imu("/sys/bus/iio/devices/iio:device0");

    // Initialize the IMU
    
    if (!imu.initialize()) {
        std::cout << "Failed to initialize IMU." << std::endl;
        return 1;
    }
    // imu.enableSensorFusionAlgorithm();
    
    ///////////////////////////
    //thread 1 to capture image
    // Open the video capture device
    cap.open(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video capture device" << std::endl;
        return -1;
    }

    // Read the first frame to initialize variables
    cv::Mat frame;
    //for testing purpose
    // frame = cv::imread("1.jpeg", cv::IMREAD_COLOR);
    // if (frame.empty()) {
    //     std::cerr << "Failed to read image" << std::endl;
    //     return -1;
    // }
    
    cap.read(frame);
 
    if (frame.empty()) {
        return -1;
    }
    //thread 2 to get IMU data
    // Read gyro data from LSM6DS IMU
    cv::Vec3f gyro_data(3) ;
    cv::Vec3f acc_data(3) ;
    readImuData(imu,gyro_data,acc_data);
    //fake gyros data for testing
    // gyro_data[0]= 11;
    // gyro_data[1]= 1;
    // gyro_data[2]= 2;
    // Apply image stabilization using EIS algorithm
    cv::Mat stabilized_frame = stabilizeImage_single_frame(frame, gyro_data);

    // Display stabilized frame
    cv::imwrite("Stabilized_img.jpg", stabilized_frame);
    cv::imwrite("origin_img.jpg", frame);
  
    // Release resources
    cap.release();
    // Close the IMU
    imu.close();

    return 0;
}
