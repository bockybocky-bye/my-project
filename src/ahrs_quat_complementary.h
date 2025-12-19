#ifndef AHRS_QUAT_COMPLEMENTARY_H
#define AHRS_QUAT_COMPLEMENTARY_H

#include "imu9250.h"

typedef struct {
    float q[4];      // w,x,y,z
    float kp_acc;    // accel correction gain (rad/s equivalent)
    float acc_gate_g; // gate: ถ้า |a| เบี่ยงจาก 1g มากไป จะไม่ใช้ accel (เช่น 0.25 = ±25%)
    int   inited;
} AHRSQuatComp;

void ahrs_qc_init(AHRSQuatComp *s, float kp_acc, float acc_gate_g);
void ahrs_qc_update(AHRSQuatComp *s, const Imu9250Data *d, float dt);

// helper: แปลงเป็น euler deg (ใช้ของ Quaternion.h ได้ก็ได้ แต่ให้สะดวก)
void ahrs_qc_get_euler_deg(const AHRSQuatComp *s, float *roll_deg, float *pitch_deg, float *yaw_deg);


#endif