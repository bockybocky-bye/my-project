#include <math.h>
#include "ahrs_quat_complementary.h"
#include "Quaternion.h"



static void normalize3(float v[3]) {
    float n = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1e-9f) return;
    v[0] /= n; v[1] /= n; v[2] /= n;
}

static float clampf(float x, float lo, float hi){
    if(x < lo) return lo;
    if(x > hi) return hi;
    return x;
}

void ahrs_qc_init(AHRSQuatComp *s, float kp_acc, float acc_gate_g)
{
    s->q[0] = 1.0f; s->q[1] = 0.0f; s->q[2] = 0.0f; s->q[3] = 0.0f;
    s->kp_acc = kp_acc;
    s->acc_gate_g = acc_gate_g;
    s->inited = 0;
}

void ahrs_qc_update(AHRSQuatComp *s, const Imu9250Data *d, float dt)
{
    if (dt <= 0.0f) return;

    // 1) propagate ด้วย gyro
    quat_update(s->q, d->gx, d->gy, d->gz, dt);

    // 2) accel magnitude gate (กันช่วงมีการเร่ง ไม่อยากให้ accel หลอก)
    float an = sqrtf(d->ax*d->ax + d->ay*d->ay + d->az*d->az);
    float g = 9.80665f;
    float ratio = an / g;  // ควรใกล้ 1 ตอนนิ่ง
    if (fabsf(ratio - 1.0f) > s->acc_gate_g) {
        return; // ไม่ใช้ accel correction
    }

    // 3) measured "gravity direction" จาก accel (normalize)
    float a_b[3] = { d->ax, d->ay, d->az };
    normalize3(a_b);
    a_b[0] = -a_b[0];
    a_b[1] = -a_b[1];
    a_b[2] = -a_b[2];

    // 4) predicted gravity direction จาก quaternion ผ่าน R
    float R[3][3];
    quat_to_R(s->q, R);

    // สมมติ g_n = (0,0,-1) ใน world
    // g_b = R^T * g_n  => g_b[i] = sum_j R[j][i] * g_n[j]
    float g_n[3] = {0.0f, 0.0f, -1.0f};
    float g_b[3] = {0};
    for(int i=0;i<3;i++){
        g_b[i] = 0.0f;
        for(int j=0;j<3;j++){
            g_b[i] += R[j][i] * g_n[j];
        }
    }
    normalize3(g_b);

    // 5) error = a_b x g_b  (cross)
    float e[3];
    e[0] = a_b[1]*g_b[2] - a_b[2]*g_b[1];
    e[1] = a_b[2]*g_b[0] - a_b[0]*g_b[2];
    e[2] = a_b[0]*g_b[1] - a_b[1]*g_b[0];

    // 6) small-angle correction
    float dtheta[3] = {
        s->kp_acc * e[0] * dt,
        s->kp_acc * e[1] * dt,
        s->kp_acc * e[2] * dt
    };

    // ป้องกัน correction ใหญ่เกิน (กันกระชาก)
    dtheta[0] = clampf(dtheta[0], -0.2f, 0.2f);
    dtheta[1] = clampf(dtheta[1], -0.2f, 0.2f);
    dtheta[2] = clampf(dtheta[2], -0.2f, 0.2f);

    quat_correct(s->q, dtheta);
}

void ahrs_qc_get_euler_deg(const AHRSQuatComp *s, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float r,p,y;
    quat_to_euler(s->q, &r, &p, &y);
    *roll_deg  = r * 180.0f / (float)M_PI;
    *pitch_deg = p * 180.0f / (float)M_PI;
    *yaw_deg   = y * 180.0f / (float)M_PI;
}