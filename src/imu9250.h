#ifndef IMU9250_H
#define IMU9250_H

#include <Arduino.h>

// ข้อมูลหนึ่ง sample จาก IMU (ใช้ feed เข้า EKF ได้เลย)
struct Imu9250Data
{
    // accelerometer [m/s^2]
    float ax, ay, az;

    // gyroscope [rad/s]
    float gx, gy, gz;

    // magnetometer (หน่วย raw หรือ uT แล้วแต่จะสเกลต่อ)
    float mx, my, mz;

    // มุมจาก accel+mag (ไว้ debug / plot)
    float roll_deg;
    float pitch_deg;
    float yaw_deg;   // 0–360 deg
};

// เรียกครั้งเดียวใน setup()
void imu9250_init();

// อ่านค่าล่าสุดจากเซนเซอร์
// คืนค่า true ถ้าอ่านสำเร็จ (mag อาจยังไม่ ready ก็จะคงค่าก่อนหน้าไว้)
bool imu9250_read(Imu9250Data &out);

void imu9250_calibrate_gyro_accel(int samples);

void imu9250_collect_mag_minmax(int samples,
                                float &mx_min, float &mx_max,
                                float &my_min, float &my_max,
                                float &mz_min, float &mz_max);

void imu9250_compute_mag_calib(float mx_min, float mx_max,
                               float my_min, float my_max,
                               float mz_min, float mz_max);

void imu9250_set_calibration(
  float axb, float ayb, float azb,
  float gxb, float gyb, float gzb,
  float mox, float moy, float moz,
  float msx, float msy, float msz);

#endif // IMU9250_H
