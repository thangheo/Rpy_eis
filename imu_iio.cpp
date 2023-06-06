#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <iio.h>
#include "imu_iio.h"
#include <cstring>
IIO_IMU::IIO_IMU(const std::string& device_path)
    : device_path_(device_path), iio_fd_(-1)
{
}

IIO_IMU::~IIO_IMU()
{
    close();
}

bool IIO_IMU::initialize()
{
    if (open_device()) {
        std::cout << "IMU initialized." << std::endl;
        return true;
    }

    std::cout << "Failed to initialize IMU." << std::endl;
    return false;
}

void IIO_IMU::close()
{
    close_device();
    std::cout << "IMU closed." << std::endl;
}

bool IIO_IMU::read_data(std::vector<double>& gyro_data, std::vector<double>& accel_data)
{
    gyro_data.resize(3);
    accel_data.resize(3);

    gyro_data[0] = read_channel("in_anglvel_0");
    gyro_data[1] = read_channel("in_anglvel_1");
    gyro_data[2] = read_channel("in_anglvel_2");

    accel_data[0] = read_channel("in_accel_0");
    accel_data[1] = read_channel("in_accel_1");
    accel_data[2] = read_channel("in_accel_2");

    return true;
}

bool IIO_IMU::config(const std::string& key, const std::string& value)
{
    std::string sysfs_path = device_path_ + "/" + key;

    if (write_sysfs_file(sysfs_path, value)) {
        std::cout << "Configuration set: " << key << " = " << value << std::endl;
        return true;
    }

    std::cout << "Failed to set configuration: " << key << " = " << value << std::endl;
    return false;
}

bool IIO_IMU::open_device()
{
    iio_fd_ = ::open(device_path_.c_str(), O_RDWR);
    return (iio_fd_ != -1);
}

void IIO_IMU::close_device()
{
    if (iio_fd_ != -1) {
        ::close(iio_fd_);
        iio_fd_ = -1;
    }
}

int IIO_IMU::read_channel(const std::string& channel)
{
    std::string sysfs_path = device_path_ + "/scan_elements/" + channel + "_en";
    if (!write_sysfs_file(sysfs_path, "1")) {
        std::cout << "Failed to enable channel: " << channel << std::endl;
        return 0;
    }

    sysfs_path = device_path_ + "/scan_elements/" + channel;
    std::ifstream channel_file(sysfs_path);
    if (channel_file) {
        int value = 0;
        channel_file >> value;
        channel_file.close();
        return value;
    }

    std::cout << "Failed to read channel: " << channel << std::endl;
    return 0;
}

bool IIO_IMU::write_sysfs_file(const std::string& path, const std::string& value)
{
    std::ofstream file(path);
    if (file) {
        file << value;
        file.close();
        return true;
    }
    return false;
}
bool IIO_IMU::enableSensorFusionAlgorithm()
{
    struct iio_context* context = iio_create_local_context();
    if (!context) {
        std::cerr << "Failed to create IIO context" << std::endl;
        return false;
    }

    struct iio_device* device = iio_context_find_device(context, device_path_.c_str());
    if (!device) {
        std::cerr << "Failed to find IIO device" << std::endl;
        iio_context_destroy(context);
        return false;
    }

    const char* enable_attr = "enable_sensor_fusion_algorithm";
    struct iio_channel* fusion_chan = iio_device_find_channel(device, enable_attr, false);
    if (!fusion_chan) {
        std::cerr << "Failed to find IIO channel for sensor fusion" << std::endl;
        iio_context_destroy(context);
        return false;
    }

    const char enable_value = '1';
    ssize_t write_result = iio_channel_write_raw(fusion_chan, NULL, &enable_value, sizeof(enable_value));
    if (write_result < 0) {
        std::cerr << "Failed to enable sensor fusion algorithm" << std::endl;
        iio_context_destroy(context);
        return false;
    }

    std::cout << "Sensor fusion algorithm enabled" << std::endl;

    iio_context_destroy(context);

    return true;
}

// bool IIO_IMU::enableSensorFusionAlgorithm()
// {
//     struct iio_context* context = iio_create_local_context();
//     if (!context) {
//         std::cerr << "Failed to create IIO context" << std::endl;
//         return false;
//     }

//     struct iio_device* device = iio_context_find_device(context, device_path_.c_str());
//     if (!device) {
//         std::cerr << "Failed to find IIO device" << std::endl;
//         iio_context_destroy(context);
//         return false;
//     }

//     struct iio_buffer* buffer = iio_device_create_buffer(device, 1, false);
//     if (!buffer) {
//         std::cerr << "Failed to create IIO buffer" << std::endl;
//         iio_context_destroy(context);
//         return false;
//     }

//     const char* enable_attr = "enable_sensor_fusion_algorithm";
//     struct iio_channel* fusion_chan = iio_device_find_channel(device, enable_attr, false);
//     if (!fusion_chan) {
//         std::cerr << "Failed to find IIO channel for sensor fusion" << std::endl;
//         iio_buffer_destroy(buffer);
//         iio_context_destroy(context);
//         return false;
//     }

//     const char* enable_value = "1";
//     ssize_t write_result = iio_channel_write_raw(fusion_chan, enable_attr, enable_value, strlen(enable_value));
//     if (write_result < 0) {
//         std::cerr << "Failed to enable sensor fusion algorithm" << std::endl;
//         iio_buffer_destroy(buffer);
//         iio_context_destroy(context);
//         return false;
//     }

//     std::cout << "Sensor fusion algorithm enabled" << std::endl;

//     iio_buffer_destroy(buffer);
//     iio_context_destroy(context);

//     return true;
// }



// int test_imu(void)
// {
//     // Create an instance of the IIO_IMU class
//     IIO_IMU imu("/sys/bus/iio/devices/iio:device0");

//     // Initialize the IMU
//     if (!imu.initialize()) {
//         std::cout << "Failed to initialize IMU." << std::endl;
//         return 1;
//     }

//     // Read and display IMU data in a loop
//     while (true) {
//         std::vector<double> gyro_data;
//         std::vector<double> accel_data;

//         // Read gyroscope and accelerometer data from the IMU
//         if (imu.read_data(gyro_data, accel_data)) {
//             std::cout << "Gyroscope Data: ";
//             for (const auto& gyro : gyro_data) {
//                 std::cout << gyro << " ";
//             }
//             std::cout << std::endl;

//             std::cout << "Accelerometer Data: ";
//             for (const auto& accel : accel_data) {
//                 std::cout << accel << " ";
//             }
//             std::cout << std::endl;
//         }

//         // Sleep or perform other operations here
//     }

//     // Close the IMU
//     imu.close();

//     return 0;
// }
