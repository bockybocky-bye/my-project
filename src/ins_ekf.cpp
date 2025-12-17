#include <math.h>
#include <string.h>
#include "Quaternion.h"

static inline float deg2rad(float d)
{
    return d * 3.14159265358979323846f / 180.0f;
}

typedef struct {
    // nominal state
    float p[3];   // position (x,y,z) - ใน local frame (เช่น ENU หรือ NED)
    float v[3];   // velocity (vx,vy,vz)
    float q[4];   // attitude quaternion [w,x,y,z]

    float bg[3];  // gyro bias (rad/s)
    float ba[3];  // accel bias (m/s^2)

    // error covariance 15x15 สำหรับ δx = [δp, δv, δθ, δbg, δba]
    float P[15][15];
} INSState;

//////////////////////////////////////////////////////////////////////////////////////////
// ================== Init ==================
void ins_init(INSState *s)
{
    memset(s, 0, sizeof(INSState));
    // initial orientation: assume level (w=1,x=y=z=0)
    s->q[0] = 1.0f; s->q[1] = s->q[2] = s->q[3] = 0.0f;

    // initialize P as diagonal
    for (int i=0;i<15;i++)
        for (int j=0;j<15;j++)
            s->P[i][j] = 0.0f;
    // pos
    s->P[0][0] = 1.0f;  
    s->P[1][1] = 1.0f;
    s->P[2][2] = 1.0f;

    // vel
    s->P[3][3] = 1.0f;
    s->P[4][4] = 1.0f;
    s->P[5][5] = 1.0f;

    // attitude error
    s->P[6][6] = 0.01f;
    s->P[7][7] = 0.01f;
    s->P[8][8] = 0.01f;

    // gyro bias
    s->P[9][9]   = 0.01f;
    s->P[10][10] = 0.01f;
    s->P[11][11] = 0.01f;

    // accel bias
    s->P[12][12] = 0.1f;
    s->P[13][13] = 0.1f;
    s->P[14][14] = 0.1f;
}

// ============ Prediction Step (IMU) =============
// gyro, acc in body frame, SI units: [rad/s], [m/s^2]
void ins_predict(INSState *s, const float gyro[3], const float acc[3], float dt)
{
    // 1) ลบ bias จาก sensor
    float wx = gyro[0] - s->bg[0];
    float wy = gyro[1] - s->bg[1];
    float wz = gyro[2] - s->bg[2];

    float ab[3] = {
        acc[0] - s->ba[0],
        acc[1] - s->ba[1],
        acc[2] - s->ba[2]
    };

    // 2) update quaternion
    quat_update(s->q, wx, wy, wz, dt);

    // 3) หา a_n = R * a_b + g
    float Rnb[3][3];
    quat_to_R(s->q, Rnb);

    float g_n[3] = {0.0f, 0.0f, -9.80665f};
    float an[3] = {0.0f, 0.0f, 0.0f};

    // an = Rnb * ab + g_n
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++)
            an[i] += Rnb[i][j] * ab[j];
        an[i] += g_n[i];
    }

    // 4) update velocity & position
    for (int i=0;i<3;i++){
        s->p[i] += s->v[i] * dt + 0.5f * an[i] * dt*dt;
        s->v[i] += an[i] * dt;
    }

    // ====== EKF PREDICTION ======
    float F[15][15] = {0};
    float Q[15][15] = {0};

    // ---------- 1) สร้าง Fc (continuous-time) ----------
    float Fc[15][15] = {0};

    // 1.1 δp_dot = δv  => Fc[p,v] = I3
    Fc[0][3] = 1.0f;  // d(δp_x)/d(δv_x)
    Fc[1][4] = 1.0f;  // d(δp_y)/d(δv_y)
    Fc[2][5] = 1.0f;  // d(δp_z)/d(δv_z)

    // 1.2 δv_dot = -R_nb [a_b]_x δθ - R_nb δb_a
    // สร้าง skew matrix ของ a_b (ab)
    float Sab[3][3] = {
        {      0.0f,  -ab[2],   ab[1]},
        {   ab[2],    0.0f,  -ab[0]},
        {  -ab[1],   ab[0],   0.0f }
    };
    // M = Rnb * Sab (3x3)
    float M[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                M[i][j] += Rnb[i][k] * Sab[k][j];
            }
        }
    }
    // Fc[v, theta] = - M
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Fc[3 + i][6 + j] = -M[i][j];
        }
    }
    // Fc[v, ba] = - Rnb
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Fc[3 + i][12 + j] = -Rnb[i][j];
        }
    }

    // 1.3 δθ_dot = -[ω]_x δθ - δb_g
    // skew ของω = [wx,wy,wz]
    float Sw[3][3] = {
        {    0.0f,  -wz,    wy},
        {    wz,    0.0f,  -wx},
        {   -wy,    wx,    0.0f}
    };
    // Fc[theta, theta] = -Sw
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Fc[6 + i][6 + j] = -Sw[i][j];
        }
    }
    // Fc[theta, bg] = -I3
    Fc[6][9]  = -1.0f;
    Fc[7][10] = -1.0f;
    Fc[8][11] = -1.0f;

    // 1.4 bg_dot, ba_dot = 0 => Fc ส่วนอื่นเป็น 0 อยู่แล้ว

    // ---------- 2) Discretize: F = I + Fc*dt ----------
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            F[i][j] = (i == j ? 1.0f : 0.0f) + Fc[i][j] * dt;
        }
    }

    // ---------- 3) Q (process noise) เหมือนเดิมไปก่อน ----------
    float sigma_acc  = 0.1f;    // process noise สำหรับความเร็ว
    float sigma_gyro = 0.01;   // process noise สำหรับ attitude error
    float sigma_bg   = 0.01f; // bias gyro
    float sigma_ba   = 0.001f;  // bias accel

    float qv  = sigma_acc  * sigma_acc;
    float qt  = sigma_gyro * sigma_gyro;
    float qbg = sigma_bg   * sigma_bg;
    float qba = sigma_ba   * sigma_ba;

    memset(Q, 0, sizeof(Q));
    // velocity noise
    Q[3][3] = qv * dt;
    Q[4][4] = qv * dt;
    Q[5][5] = qv * dt;
    // attitude error noise
    Q[6][6] = qt * dt;
    Q[7][7] = qt * dt;
    Q[8][8] = qt * dt;
    // gyro bias
    Q[9][9]   = qbg * dt;
    Q[10][10] = qbg * dt;
    Q[11][11] = qbg * dt;
    // acc bias
    Q[12][12] = qba * dt;
    Q[13][13] = qba * dt;
    Q[14][14] = qba * dt;

    // ---------- 4) P_pred = F P F^T + Q (เหมือนของเดิม) ----------
    float FP[15][15] = {0};
    float FPFt[15][15] = {0};

    // FP = F * P
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            for(int k=0;k<15;k++)
                FP[i][j] += F[i][k]*s->P[k][j];

    // FPFt = FP * F^T
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            for(int k=0;k<15;k++)
                FPFt[i][j] += FP[i][k]*F[j][k];  // F^T

    // P = FPFt + Q
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            s->P[i][j] = FPFt[i][j] + Q[i][j];
    // --- enforce symmetry ---
    for (int i=0; i<15; i++)
        for (int j=i+1; j<15; j++) {
            float sym = 0.5f * (s->P[i][j] + s->P[j][i]);
            s->P[i][j] = s->P[j][i] = sym;
        }
}

// ============ GPS Update (position only) ============
void ins_update_gps(INSState *s, const float gps_p[3], const float R_gps[3][3])
{
    // z = gps position in n-frame (สมมติแปลง lat/lon เป็น x,y,z แล้ว)
    float z[3] = { gps_p[0], gps_p[1], gps_p[2] };

    // h(x) = p_nominal
    float h[3] = { s->p[0], s->p[1], s->p[2] };

    // residual r = z - h
    float r[3] = {
        z[0] - h[0],
        z[1] - h[1],
        z[2] - h[2]
    };

    // H : 3x15, H = [I3, 0]
    float H[3][15] = {0};
    H[0][0] = 1.0f;
    H[1][1] = 1.0f;
    H[2][2] = 1.0f;

    // S = H P H^T + R_gps (3x3)
    float S[3][3] = {0};
    // H P H^T
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<15;k++)
                S[i][j] += H[i][k] * s->P[k][j];  // ใช้แค่ส่วนคอลัมน์แรก ๆ
    // บวก R_gps
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            S[i][j] += R_gps[i][j];

    // inverse S (3x3) — เขียนฟังก์ชัน invert 3x3 แยกต่างหากจะสวยกว่า
    float det =
        S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1]) -
        S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
        S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    float invS[3][3];
    float invdet = 1.0f/det;
    invS[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invdet;
    invS[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invdet;
    invS[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invdet;
    invS[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invdet;
    invS[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invdet;
    invS[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invdet;
    invS[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invdet;
    invS[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invdet;
    invS[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invdet;

    // K = P H^T invS  (15x3)
    float PHt[15][3] = {0};
    float K[15][3] = {0};

    // PHt = P * H^T
    for(int i=0;i<15;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<15;k++)
                PHt[i][j] += s->P[i][k] * H[j][k];

    // K = PHt * invS
    for(int i=0;i<15;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                K[i][j] += PHt[i][k] * invS[k][j];

    // δx = K * r  (15x1)
    float dx[15] = {0};
    for(int i=0;i<15;i++)
        for(int j=0;j<3;j++)
            dx[i] += K[i][j] * r[j];

    // ใช้ dx แก้ nominal state
    // δp
    s->p[0] += dx[0];
    s->p[1] += dx[1];
    s->p[2] += dx[2];

    // δv
    s->v[0] += dx[3];
    s->v[1] += dx[4];
    s->v[2] += dx[5];

    // δθ → quaternion correction
    float dtheta[3] = { dx[6], dx[7], dx[8] };
    quat_correct(s->q, dtheta);

    // δbg
    s->bg[0] += dx[9];
    s->bg[1] += dx[10];
    s->bg[2] += dx[11];

    // δba
    s->ba[0] += dx[12];
    s->ba[1] += dx[13];
    s->ba[2] += dx[14];

    // รีเซ็ตส่วน error attitude ใน covariance (optional: ใช้วิธี Joseph form จะดีกว่า)
    float I[15][15] = {0};
    for(int i=0;i<15;i++) I[i][i] = 1.0f;

    float KH[15][15] = {0};
    float IMKH[15][15] = {0};
    float temp[15][15] = {0};
    float KRKt[15][15] = {0};
    float newP[15][15] = {0};

    // KH = K * H  (15x3 * 3x15)
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            for(int k=0;k<3;k++)
                KH[i][j] += K[i][k]*H[k][j];

    // IMKH = I - KH
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            IMKH[i][j] = I[i][j] - KH[i][j];

    // temp = (I - K H) P
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            for(int k=0;k<15;k++)
                temp[i][j] += IMKH[i][k] * s->P[k][j];

    // newP = temp * (I - K H)^T
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            for(int k=0;k<15;k++)
                newP[i][j] += temp[i][k] * IMKH[j][k];  // transpose

    // KRKt = K * R_gps * K^T   (15x3 * 3x3 * 3x15)
    float KR[15][3] = {0};
    for(int i=0;i<15;i++)
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                KR[i][j] += K[i][k] * R_gps[k][j];

    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            for(int k=0;k<3;k++)
                KRKt[i][j] += KR[i][k] * K[j][k]; // K^T

    // P = (I - KH) P (I - KH)^T + K R K^T
    // P = newP + KRKt
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            s->P[i][j] = newP[i][j] + KRKt[i][j];

    // OPTIONAL: force symmetry
    for(int i=0;i<15;i++)
        for(int j=i+1;j<15;j++){
            float sym = 0.5f*(s->P[i][j] + s->P[j][i]);
            s->P[i][j] = s->P[j][i] = sym;
        }
}

