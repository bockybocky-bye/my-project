#ifndef AHRS_COMPLEMENTARY_H
#define AHRS_COMPLEMENTARY_H

typedef struct {
    float roll;   // rad
    float pitch;  // rad
    float yaw;    // rad (0..2pi)
    float alpha;  // 0..1  (เช่น 0.98)
    int   inited; // 0/1
} AhrsCompState;

// init state
void ahrs_comp_init(AhrsCompState *s, float alpha);

// update ด้วย sensor (หน่วย: ax,ay,az เป็น m/s^2 | gx,gy,gz เป็น rad/s | mx,my,mz เป็นหน่วยใดก็ได้ที่สเกลคงที่)
void ahrs_comp_update(AhrsCompState *s,
                      float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      float dt);

// helper: แปลง rad->deg
float ahrs_rad2deg(float r);

#endif