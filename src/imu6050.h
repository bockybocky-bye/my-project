#ifndef MPU6050_H
#define MPU6050_H

#include "Wire.h"

extern float RateRoll, RatePitch, RateYaw;
extern float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;
extern float KalmanAngleRoll , KalmanUncertaintyAngleRoll ;
extern float KalmanAnglePitch , KalmanUncertaintyAnglePitch ;
extern float Kalman1DOutput[2];

void gyro_signals();
void TaskMPU6050(void *pvParameters);

#endif