#ifndef IMU_SPI_H
#define IMU_SPI_H

#include <string>

class IMU_SPI {
public:
    IMU_SPI(const std::string& device, int speed, int mode);
    ~IMU_SPI();

    bool open();
    void close();
    bool readData(float& gyroX, float& gyroY, float& gyroZ, float& accelX, float& accelY, float& accelZ);

private:
    std::string device_;
    int speed_;
    int mode_;
    int fileDescriptor_;

    bool transfer(unsigned char* txBuffer, unsigned char* rxBuffer, int length);
};

#endif // IMU_SPI_H
