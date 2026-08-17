#ifndef MPU6050_DEV_H
#define MPU6050_DEV_H

#include "drivers/mpu6050.h"

namespace mpu6050_dev
{
    bool peek_latest(mpu6050_sample &sample);
    void init();
}

#endif
