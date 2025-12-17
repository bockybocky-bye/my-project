
#include <math.h>
#include "Quaternion.h"

// Quaternion propagation step
void quat_normalize(float q[4]) {
    float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    float inv = 1.0f / n;
    for(int i=0;i<4;i++) q[i]*=inv;
}

void quat_omega_matrix(const float w[3], float Om[4][4]) {
    float wx=w[0], wy=w[1], wz=w[2];
    Om[0][0]=0;   Om[0][1]=-wx; Om[0][2]=-wy; Om[0][3]=-wz;
    Om[1][0]=wx;  Om[1][1]=0;   Om[1][2]= wz; Om[1][3]=-wy;
    Om[2][0]=wy;  Om[2][1]=-wz; Om[2][2]=0;   Om[2][3]= wx;
    Om[3][0]=wz;  Om[3][1]= wy; Om[3][2]=-wx; Om[3][3]=0;
}

void quat_to_R(const float q[4], float R[3][3]) {
    float w=q[0], x=q[1], y=q[2], z=q[3];
    float ww=w*w, xx=x*x, yy=y*y, zz=z*z;
    float wx=w*x, wy=w*y, wz=w*z;
    float xy=x*y, xz=x*z, yz=y*z;

    R[0][0]= 1-2*(yy + zz); R[0][1]= 2*(xy - wz);  R[0][2]= 2*(xz + wy);
    R[1][0]= 2*(xy + wz); R[1][1]= 1-2*(xx + zz); R[1][2]= 2*(yz - wx);
    R[2][0]= 2*(xz - wy); R[2][1]= 2*(yz + wx); R[2][2]= 1-2*(xx + yy);
}

void euler_to_quat(float roll, float pitch, float yaw, float q[4])
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q[0] = cr*cp*cy + sr*sp*sy;  // w
    q[1] = sr*cp*cy - cr*sp*sy;  // x
    q[2] = cr*sp*cy + sr*cp*sy;  // y
    q[3] = cr*cp*sy - sr*sp*cy;  // z

    // normalize เผื่อเลขลอยตัวไม่เป๊ะ
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    float inv = 1.0f / n;
    for (int i=0;i<4;i++) q[i] *= inv;
}


void quat_update(float q[4], float wx, float wy, float wz, float dt)
{
    float w[3] = {wx, wy, wz};
    float Om[4][4];
    quat_omega_matrix(w, Om);

    float qdot[4] = {0.0f};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            qdot[i] += 0.5f * Om[i][j] * q[j];

    // integrate
    for (int i = 0; i < 4; i++)
        q[i] += qdot[i] * dt;

    // normalize เพื่อไม่ให้ drift
    quat_normalize(q);
}

void predict_gravity(const float q[4], float a_pred[3]) {
    float Rnb[3][3];
    quat_to_R(q, Rnb);

    float g_n[3] = {0.0f, 0.0f, -9.80665f};

    for (int i = 0; i < 3; i++) {
        a_pred[i] = 0;
        for (int j = 0; j < 3; j++)
            a_pred[i] += Rnb[j][i] * g_n[j];  // world → body
    }
}

void quat_correct(float q[4], const float dtheta[3]) {
    // small angle quaternion δq ≈ [1, 0.5*δθ]
    float dq[4];
    dq[0] = 1.0f;
    dq[1] = 0.5f * dtheta[0];
    dq[2] = 0.5f * dtheta[1];
    dq[3] = 0.5f * dtheta[2];

    // q_new = dq ⊗ q
    float q_old[4] = {q[0], q[1], q[2], q[3]};
    q[0] = dq[0]*q_old[0] - dq[1]*q_old[1] - dq[2]*q_old[2] - dq[3]*q_old[3];
    q[1] = dq[0]*q_old[1] + dq[1]*q_old[0] + dq[2]*q_old[3] - dq[3]*q_old[2];
    q[2] = dq[0]*q_old[2] - dq[1]*q_old[3] + dq[2]*q_old[0] + dq[3]*q_old[1];
    q[3] = dq[0]*q_old[3] + dq[1]*q_old[2] - dq[2]*q_old[1] + dq[3]*q_old[0];

    quat_normalize(q);
}

void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // Roll (φ)
    float sinr = 2*(w*x + y*z);
    float cosr = 1 - 2*(x*x + y*y);
    *roll = atan2f(sinr, cosr);

    // Pitch (θ)
    float sinp = 2*(w*y - z*x);
    if (sinp >= 1) sinp = 1;
    if (sinp <= -1) sinp = -1;
    *pitch = asinf(sinp);

    // Yaw (ψ)
    float siny = 2*(w*z + x*y);
    float cosy = 1 - 2*(y*y + z*z);
    *yaw = atan2f(siny, cosy);
}
