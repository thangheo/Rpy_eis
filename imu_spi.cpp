// #include "imu_spi.h"
// #include <iostream>
// #include <cstring>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/spi/spidev.h>

// IMU_SPI::IMU_SPI(const std::string& device, int speed, int mode)
//     : device_(device), speed_(speed), mode_(mode), fileDescriptor_(-1)
// {
// }

// IMU_SPI::~IMU_SPI()
// {
//     close();
// }

// bool IMU_SPI::open()
// {
//     fileDescriptor_ = ::open(device_.c_str(), O_RDWR);
//     if (fileDescriptor_ < 0) {
//         std::cout << "Failed to open SPI device: " << device_ << std::endl;
//         return false;
//     }

//     // Set SPI mode
//     int ret = ioctl(fileDescriptor_, SPI_IOC_WR_MODE, &mode_);
//     if (ret == -1) {
//         std::cout << "Failed to set SPI mode." << std::endl;
//         close();
//         return false;
//     }

//     // Set SPI speed
//     ret = ioctl(fileDescriptor_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_);
//     if (ret == -1) {
//         std::cout << "Failed to set SPI speed." << std::endl;
//         close();
//         return false;
//     }

//     return true;
// }

// void IMU_SPI::close()
// {
//     if (fileDescriptor_ >= 0) {
//         ::close(fileDescriptor_);
//         fileDescriptor_ = -1;
//     }
// }

// bool IMU_SPI::transfer(unsigned char* txBuffer, unsigned char* rxBuffer, int length)
// {
//     struct spi_ioc_transfer transfer{};
//     transfer.tx_buf = reinterpret_cast<unsigned long>(txBuffer);
//     transfer.rx_buf = reinterpret_cast<unsigned long>(rxBuffer);
//     transfer.len = length;

//     int ret = ioctl(fileDescriptor_, SPI_IOC_MESSAGE(1), &transfer);
//     if (ret < 0) {
//         std::cout << "SPI transfer failed." << std::endl;
//         return false;
//     }

//     return true;
// }
// bool IMU_SPI::readData(float& gyroX, float& gyroY, float& gyroZ, float& accelX, float& accelY, float& accelZ)
// {
//     // Buffer to hold the SPI data
//     unsigned char txBuffer[12] = {0};  // 6 bytes for gyroscope data, 6 bytes for accelerometer data
//     unsigned char rxBuffer[12] = {0};

//     // Set up the SPI data to be sent
//     // TODO: Modify txBuffer with appropriate data for your IMU

//     // Perform SPI data transfer
//     if (!transfer(txBuffer, rxBuffer, sizeof(txBuffer))) {
//         std::cout << "SPI data transfer failed." << std::endl;
//         return false;
//     }

//     // Parse the received data and update gyroX, gyroY, gyroZ, accelX, accelY, accelZ accordingly
//     // TODO: Parse the received data from rxBuffer and update the variables accordingly

//     // Example code: assuming gyroscope data is stored in rxBuffer[0] to rxBuffer[5]
//     gyroX = static_cast<float>(reinterpret_cast<int16_t>((rxBuffer[0] << 8) | rxBuffer[1]));
//     gyroY = static_cast<float>(reinterpret_cast<int16_t>((rxBuffer[2] << 8) | rxBuffer[3]));
//     gyroZ = static_cast<float>(reinterpret_cast<int16_t>((rxBuffer[4] << 8) | rxBuffer[5]));

//     // Example code: assuming accelerometer data is stored in rxBuffer[6] to rxBuffer[11]
//     accelX = static_cast<float>(reinterpret_cast<int16_t>((rxBuffer[6] << 8) | rxBuffer[7]));
//     accelY = static_cast<float>(reinterpret_cast<int16_t>((rxBuffer[8] << 8) | rxBuffer[9]));
//     accelZ = static_cast<float>(reinterpret_cast<int16_t>((rxBuffer[10] << 8) | rxBuffer[11]));

//     return true;
// }
