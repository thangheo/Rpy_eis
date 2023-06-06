#ifndef IMU_IIO_H
#define IMU_IIO_H

#include <string>
#include <vector>

class IIO_IMU {
public:
    IIO_IMU(const std::string& device_path);
    ~IIO_IMU();

    bool initialize();
    void close();
    bool enableSensorFusionAlgorithm();
    bool read_data(std::vector<double>& gyro_data, std::vector<double>& accel_data);
    bool config(const std::string& key, const std::string& value);

private:
    std::string device_path_;
    int iio_fd_;

    bool open_device();
    void close_device();
    int read_channel(const std::string& channel);

    bool write_sysfs_file(const std::string& path, const std::string& value);

    struct iio_context* context_;
    struct iio_device* device_;

    // ... other private member functions ...
};


#endif // IMU_IIO_H
