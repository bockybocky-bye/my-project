#include "ahrs_complementary.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float wrapPi(float a){
    while(a >  (float)M_PI) a -= 2.0f*(float)M_PI;
    while(a < -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}
static float wrap2Pi(float a){
    while(a >= 2.0f*(float)M_PI) a -= 2.0f*(float)M_PI;
    while(a <  0.0f)             a += 2.0f*(float)M_PI;
    return a;
}

// roll/pitch จาก accel
static void accelToRP(float ax, float ay, float az, float *r, float *p){
    // ใช้สูตรเดียวกับที่อิ๋มใช้ในโค้ดที่ “ไม่พัง”
    *r = atan2f(ay, sqrtf(ax*ax + az*az));
    *p = -atan2f(ax, sqrtf(ay*ay + az*az));
}

// yaw จาก mag + tilt compensation
static float magToYaw(float mx, float my, float mz, float roll, float pitch){
    float mx2 = mx * cosf(pitch) + mz * sinf(pitch);
    float my2 = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);
    float yaw = atan2f(-my2, mx2);
    return wrap2Pi(yaw);
}

void ahrs_comp_init(AhrsCompState *s, float alpha){
    s->roll = 0.0f;
    s->pitch = 0.0f;
    s->yaw = 0.0f;
    s->alpha = alpha;
    s->inited = 0;
}

void ahrs_comp_update(AhrsCompState *s,
                      float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      float dt)
{
    // 1) absolute roll/pitch from accel
    float ra, pa;
    accelToRP(ax, ay, az, &ra, &pa);

    // init ครั้งแรกไม่ให้กระชาก
    if(!s->inited){
        s->roll = ra;
        s->pitch = pa;
        s->yaw = magToYaw(mx, my, mz, s->roll, s->pitch);
        s->inited = 1;
        return;
    }

    // 2) gyro integrate
    float rg = s->roll  + gx * dt;
    float pg = s->pitch + gy * dt;
    float yg = s->yaw   + gz * dt;

    // 3) yaw absolute from mag (ใช้ roll/pitch จาก accel เป็น reference)
    float ym = magToYaw(mx, my, mz, ra, pa);

    // 4) complementary blend
    float a = s->alpha;
    s->roll  = a * rg + (1.0f - a) * ra;
    s->pitch = a * pg + (1.0f - a) * pa;

    // yaw ต้อง handle wrap
    float err = wrapPi(ym - yg);
    s->yaw = wrap2Pi(yg + (1.0f - a) * err);
}

float ahrs_rad2deg(float r){
    return r * (180.0f / (float)M_PI);
}
