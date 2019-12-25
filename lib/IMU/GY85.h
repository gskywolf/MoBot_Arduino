#ifndef IMU_GY85_H_
#define IMU_GY85_H_

#include <ADXL345.h>
#include <HMC5883L.h>
#include <ITG3200.h>

class IMU{
    public:
        bool init();
        void get_data(float imu_data[9]);
    private:
        ADXL345 accel;
        ITG3200 gyro;
        HMC5883L mag;

        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;
};

#endif