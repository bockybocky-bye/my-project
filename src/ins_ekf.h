#ifndef INS_EKF_H
#define INS_EKF_H

#include <stdint.h>

typedef struct {
    float p[3];
    float v[3];
    float q[4];
    float bg[3];
    float ba[3];
    float P[15][15];
} INSState;

void ins_init(INSState *s);
void ins_predict(INSState *s, const float gyro[3], const float acc[3], float dt);
void ins_update_gps(INSState *s, const float gps_p[3], const float R_gps[3][3]);

#endif // INS_EKF_H
