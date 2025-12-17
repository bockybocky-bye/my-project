/*#include <Arduino.h>
#include <Wire.h>

#include "imu9250.h"
#include "ins_ekf.h"
#include "Quaternion.h"

INSState ins;          // state ของ EKF
Imu9250Data imu;       // data จาก imu9250_read()

unsigned long lastMicros = 0;



void setup()
{
    Serial.begin(115200);

    // เลือก I2C pin ของ ESP32
    // ถ้าคุณต่อ SDA=21, SCL=22:
    Wire.begin(21, 22);
    Wire.setClock(400000);

    imu9250_init();
    ins_init(&ins);
    Imu9250Data imu0;
    for (int i = 0; i < 200; i++) {
        imu9250_read(imu0);
        delay(5);
    }

    float roll0  = imu0.roll_deg  * PI/180.0f;
    float pitch0 = imu0.pitch_deg * PI/180.0f;
    float yaw0   = imu0.yaw_deg   * PI/180.0f;

    euler_to_quat(roll0, pitch0, yaw0, ins.q);

    lastMicros = micros();

    Serial.println("IMU + EKF init done.");
}

void loop()
{
    // 1) อ่าน IMU
    imu9250_read(imu);

    // 2) คำนวณ dt จาก micros()
    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    if (dt <= 0.0f || dt > 0.1f) {
        // ป้องกัน dt แปลก ๆ (เช่นตอน reset)
        dt = 0.01f;
    }
    lastMicros = now;

    // 3) เตรียมข้อมูลส่งเข้า EKF
    float gyro[3] = { imu.gx, imu.gy, imu.gz };  // [rad/s]
    float acc[3]  = { imu.ax, imu.ay, imu.az };  // [m/s^2]

    // 4) เรียก prediction step ของ EKF
    ins_predict(&ins, gyro, acc, dt);

    // 6) อ่าน orientation จาก INSState แล้วแปลงเป็น RPY ด้วยฟังก์ชันใน Quaternion.h
    float roll_rad, pitch_rad, yaw_rad;
    quat_to_euler(ins.q, &roll_rad, &pitch_rad, &yaw_rad);  // rad

    float roll_ekf  = roll_rad  * 180.0f / PI;
    float pitch_ekf = pitch_rad * 180.0f / PI;
    float yaw_ekf   = yaw_rad   * 180.0f / PI;

    // เอา yaw ให้อยู่ช่วง 0–360°
    if (yaw_ekf <   0.0f) yaw_ekf += 360.0f;
    if (yaw_ekf >= 360.0f) yaw_ekf -= 360.0f;

 

    // 7) แสดงผลเปรียบเทียบ (IMU raw vs EKF)
    Serial.print("IMU RPY [deg] : ");
    Serial.print(imu.roll_deg);  Serial.print(", ");
    Serial.print(imu.pitch_deg); Serial.print(", ");
    Serial.print(imu.yaw_deg);   Serial.print(" | ");

    Serial.print("EKF RPY [deg] : ");
    Serial.print(roll_ekf);  Serial.print(", ");
    Serial.print(pitch_ekf); Serial.print(", ");
    Serial.println(yaw_ekf);

    Serial.print("gyro_dps: ");
    Serial.print(imu.gx * 180.0f/PI); Serial.print(", ");
    Serial.print(imu.gy * 180.0f/PI); Serial.print(", ");
    Serial.print(imu.gz * 180.0f/PI); Serial.print(" | dt = ");
    Serial.println(dt, 6);



    delay(500); //
}


*/


/*
#include <Arduino.h>
#include "imu9250.h"      // มี Imu9250Data + imu9250_init() + imu9250_read()
#include "Quaternion.h"   // มี quat_update(), quat_to_euler(), ฯลฯ

// ====== ประกาศฟังก์ชัน / struct ของ EKF ที่คุณมีอยู่แล้ว ======
typedef struct {
    float p[3];   // position
    float v[3];   // velocity
    float q[4];   // attitude quaternion [w,x,y,z]
    float bg[3];  // gyro bias
    float ba[3];  // accel bias
    float P[15][15];
} INSState;

// ฟังก์ชันที่คุณเขียนไว้ในไฟล์ EKF
void ins_init(INSState *s);
void ins_predict(INSState *s, const float gyro[3], const float acc[3], float dt);
// ถ้ามี ins_update_gps ไม่ต้องใช้ในเทสนี้

// ====== ตัวแปร global ======
INSState ins;               // EKF state
float q_gyro_only[4];       // quaternion ที่ใช้ทดสอบจาก gyro อย่างเดียว

unsigned long last_us = 0;
bool time_inited = false;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("IMU9250 + Quaternion-only + EKF attitude TEST");

    // init IMU
    imu9250_init();

    // init EKF
    ins_init(&ins);

    // init quaternion สำหรับ test (identity)
    q_gyro_only[0] = 1.0f;
    q_gyro_only[1] = 0.0f;
    q_gyro_only[2] = 0.0f;
    q_gyro_only[3] = 0.0f;

    last_us = micros();
    time_inited = true;
}

void loop() {
    Imu9250Data imu;
    if (!imu9250_read(imu)) {
        // ปกติฟังก์ชันของคุณ return true เสมอ แต่กันเหนียว
        return;
    }

    // ---------- คำนวณ dt ----------
    unsigned long now_us = micros();
    if (!time_inited) {
        last_us = now_us;
        time_inited = true;
        return;
    }

    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;

    // กันเคส dt เพี้ยนมาก (เช่น มี delay ยาว)
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.01f;  // สมมติ 100 Hz
    }

    // ---------- เตรียม gyro/acc สำหรับ EKF & quaternion ----------
    // imu.gx/gy/gz ตอนนี้ "เป็น rad/s" แล้ว (จากโค้ด imu9250 ของคุณ)
    float gyro_rad[3] = { imu.gx, imu.gy, imu.gz };

    // imu.ax/ay/az เป็น m/s^2 แล้ว
    float acc_mps2[3] = { imu.ax, imu.ay, imu.az };

    // เผื่อ debug dps ให้อ่านง่าย
    const float RAD2DEG = 180.0f / PI;
    float gx_dps = imu.gx * RAD2DEG;
    float gy_dps = imu.gy * RAD2DEG;
    float gz_dps = imu.gz * RAD2DEG;

    // ---------- 1) ทดสอบ quaternion-only (ใช้แต่ gyro) ----------
    quat_update(q_gyro_only, gyro_rad[0], gyro_rad[1], gyro_rad[2], dt);

    float roll_q, pitch_q, yaw_q;
    quat_to_euler(q_gyro_only, &roll_q, &pitch_q, &yaw_q); // rad

    float roll_q_deg  = roll_q  * RAD2DEG;
    float pitch_q_deg = pitch_q * RAD2DEG;
    float yaw_q_deg   = yaw_q   * RAD2DEG;
    if (yaw_q_deg < 0.0f) yaw_q_deg += 360.0f;

    // ---------- 2) เรียก EKF predict ----------
    ins_predict(&ins, gyro_rad, acc_mps2, dt);

    float roll_ekf, pitch_ekf, yaw_ekf;
    quat_to_euler(ins.q, &roll_ekf, &pitch_ekf, &yaw_ekf);

    float roll_ekf_deg  = roll_ekf  * RAD2DEG;
    float pitch_ekf_deg = pitch_ekf * RAD2DEG;
    float yaw_ekf_deg   = yaw_ekf   * RAD2DEG;
    if (yaw_ekf_deg < 0.0f) yaw_ekf_deg += 360.0f;

    // ---------- 3) พิมพ์เทียบ 3 ชุด: IMU, GYRO-Quat, EKF ----------
    Serial.print("dt = ");
    Serial.print(dt, 6);
    Serial.print(" | gyro_dps: ");
    Serial.print(gx_dps, 2); Serial.print(", ");
    Serial.print(gy_dps, 2); Serial.print(", ");
    Serial.print(gz_dps, 2);

    Serial.print(" | IMU RPY [deg] : ");
    Serial.print(imu.roll_deg, 2);  Serial.print(", ");
    Serial.print(imu.pitch_deg, 2); Serial.print(", ");
    Serial.print(imu.yaw_deg, 2);

    Serial.print(" | GYRO-Quat RPY [deg] : ");
    Serial.print(roll_q_deg, 2);   Serial.print(", ");
    Serial.print(pitch_q_deg, 2);  Serial.print(", ");
    Serial.print(yaw_q_deg, 2);

    Serial.print(" | EKF RPY [deg] : ");
    Serial.print(roll_ekf_deg, 2);   Serial.print(", ");
    Serial.print(pitch_ekf_deg, 2);  Serial.print(", ");
    Serial.print(yaw_ekf_deg, 2);

    Serial.println();

    delay(500); // ให้ serial อ่านง่ายหน่อย ~100 Hz
}

*/


/*
#include <Arduino.h>
#include "imu9250.h"
#include "ins_ekf.h"      // มี INSState, ins_init, ins_predict
#include "Quaternion.h"   // มี euler_to_quat(), quat_to_euler()

INSState ins;
float q_gyro_only[4];   // quaternion ที่ integrate จาก gyro อย่างเดียว

// ----------------- helper: initial alignment -----------------
void initial_alignment()
{
  delay(1000); // รอให้ IMU ตื่นนิ่ง ๆ ก่อน

  const int N = 300;
  double sum_roll = 0.0, sum_pitch = 0.0, sum_yaw = 0.0;
  double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;

  Imu9250Data imu;

  for (int i = 0; i < N; i++) {
    imu9250_read(imu);

    sum_roll  += imu.roll_deg;
    sum_pitch += imu.pitch_deg;
    sum_yaw   += imu.yaw_deg;

    sum_gx += imu.gx;   // rad/s
    sum_gy += imu.gy;
    sum_gz += imu.gz;

    delay(5);           // รวมเวลาประมาณ 1.5 วินาที
  }

  float roll0_deg  = sum_roll  / N;
  float pitch0_deg = sum_pitch / N;
  float yaw0_deg   = sum_yaw   / N;

  float roll0  = roll0_deg  * PI / 180.0f;
  float pitch0 = pitch0_deg * PI / 180.0f;
  float yaw0   = yaw0_deg   * PI / 180.0f;

  float q_init[4];
  euler_to_quat(roll0, pitch0, yaw0, q_init);

  // init EKF state
  ins_init(&ins);

  // ตั้ง quaternion เริ่มต้นให้ทั้ง EKF และ gyro-only
  for (int i = 0; i < 4; i++) {
    ins.q[i]       = q_init[i];
    q_gyro_only[i] = q_init[i];
  }

  // ตั้ง gyro bias เริ่มต้นจาก mean ช่วงนิ่ง
  ins.bg[0] = (float)(sum_gx / N);
  ins.bg[1] = (float)(sum_gy / N);
  ins.bg[2] = (float)(sum_gz / N);

  Serial.println("Initial alignment done.");
  Serial.print("Initial IMU RPY [deg] = ");
  Serial.print(roll0_deg, 2);  Serial.print(", ");
  Serial.print(pitch0_deg, 2); Serial.print(", ");
  Serial.println(yaw0_deg, 2);
}

// ----------------- setup -----------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  imu9250_init();          // init เซนเซอร์
  initial_alignment();     // ตั้ง quaternion เริ่มต้น + bias
}

// ----------------- loop (อันที่คุณขอ) -----------------
void loop()
{
  static uint32_t last_us = 0;

  // ---------- 1) อ่าน IMU ----------
  Imu9250Data imu;
  imu9250_read(imu);   // จะได้ ax,ay,az (m/s^2) + gx,gy,gz (rad/s) + roll/pitch/yaw จาก accel+mag

  // ---------- 2) คำนวณ dt ----------
  uint32_t now_us = micros();
  float dt;

  if (last_us == 0) {
    dt = 0.01f;   // สมมติเริ่มที่ 0.01s
  } else {
    dt = (now_us - last_us) * 1e-6f;
  }
  last_us = now_us;

  // กัน error ถ้า dt เพี้ยน
  if (dt < 0.001f || dt > 0.05f) {
    dt = 0.01f;
  }

  // ---------- 3) เตรียมข้อมูลส่งเข้า quat + EKF ----------
  float gyro[3] = { imu.gx, imu.gy, imu.gz };   // rad/s
  float acc[3]  = { imu.ax, imu.ay, imu.az };   // m/s^2

  // ---------- 4) อัปเดต quaternion จาก gyro อย่างเดียว ----------
  quat_update(q_gyro_only, gyro[0], gyro[1], gyro[2], dt);

  // ---------- 5) EKF prediction ----------
  ins_predict(&ins, gyro, acc, dt);

  // ---------- 6) แปลง quaternion → RPY ----------
  float r_g, p_g, y_g;
  float r_e, p_e, y_e;

  quat_to_euler(q_gyro_only, &r_g, &p_g, &y_g);   // rad
  quat_to_euler(ins.q,       &r_e, &p_e, &y_e);   // rad

  float roll_gyro_deg  = r_g * 180.0f / PI;
  float pitch_gyro_deg = p_g * 180.0f / PI;
  float yaw_gyro_deg   = y_g * 180.0f / PI;

  float roll_ekf_deg   = r_e * 180.0f / PI;
  float pitch_ekf_deg  = p_e * 180.0f / PI;
  float yaw_ekf_deg    = y_e * 180.0f / PI;

  // wrap yaw ให้ 0–360 (ทั้ง gyro และ EKF)
  if (yaw_gyro_deg < 0.0f)  yaw_gyro_deg += 360.0f;
  if (yaw_gyro_deg >= 360.0f) yaw_gyro_deg -= 360.0f;
  if (yaw_ekf_deg < 0.0f)   yaw_ekf_deg  += 360.0f;
  if (yaw_ekf_deg >= 360.0f)  yaw_ekf_deg  -= 360.0f;

  // ---------- 7) print debug ----------
  float gx_dps = imu.gx * 180.0f / PI;
  float gy_dps = imu.gy * 180.0f / PI;
  float gz_dps = imu.gz * 180.0f / PI;

  Serial.print("dt = "); Serial.print(dt, 6);
  Serial.print(" | gyro_dps: ");
  Serial.print(gx_dps, 2); Serial.print(", ");
  Serial.print(gy_dps, 2); Serial.print(", ");
  Serial.print(gz_dps, 2);

  Serial.print(" | IMU RPY [deg] : ");
  Serial.print(imu.roll_deg, 2);  Serial.print(", ");
  Serial.print(imu.pitch_deg, 2); Serial.print(", ");
  Serial.print(imu.yaw_deg, 2);

  Serial.print(" | GYRO-Quat RPY [deg] : ");
  Serial.print(roll_gyro_deg, 2);  Serial.print(", ");
  Serial.print(pitch_gyro_deg, 2); Serial.print(", ");
  Serial.print(yaw_gyro_deg, 2);

  Serial.print(" | EKF RPY [deg] : ");
  Serial.print(roll_ekf_deg, 2);  Serial.print(", ");
  Serial.print(pitch_ekf_deg, 2); Serial.print(", ");
  Serial.println(yaw_ekf_deg, 2);

  // ถ้าอยากช้าลงหน่อยให้ serial อ่านง่ายขึ้น
  delay(5);
}
*/



/*

///สำหรับเทส IMU9250 ว่าใช้งานได้ไหม

#include <Arduino.h>
#include <Wire.h>
#include "imu9250.h"

Imu9250Data imu;
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(115200);

    // ถ้า ESP32 ใช้แบบนี้:
    // Wire.begin(21, 22);  
    Wire.setClock(400000); 

    imu9250_init();
    Serial.println("IMU9250 INIT DONE");
      Serial.println("Scanning I2C...");
    for (int i = 0; i < 127; i++) {
      Wire.beginTransmission(i);
      if (Wire.endTransmission() == 0) {
        Serial.print("Found device at 0x");
        Serial.println(i, HEX);
      }
    }
    Wire.beginTransmission(0x0C);
    uint8_t err = Wire.endTransmission();
    Serial.print("AK8963 test error = ");
    Serial.println(err);
}

void loop() {
    imu9250_read(imu);

    // พิมพ์ทุก 50 ms เพื่อไม่ให้ Serial ช้าเกิน
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();

        Serial.print("ACC [m/s2]  : ");
        Serial.print(imu.ax); Serial.print(", ");
        Serial.print(imu.ay); Serial.print(", ");
        Serial.println(imu.az);

        Serial.print("GYRO [rad/s]: ");
        Serial.print(imu.gx); Serial.print(", ");
        Serial.print(imu.gy); Serial.print(", ");
        Serial.println(imu.gz);

        Serial.print("MAG (raw)   : ");
        Serial.print(imu.mx); Serial.print(", ");
        Serial.print(imu.my); Serial.print(", ");
        Serial.println(imu.mz);

        Serial.print("RPY [deg]   : ");
        Serial.print(imu.roll_deg); Serial.print(", ");
        Serial.print(imu.pitch_deg); Serial.print(", ");
        Serial.println(imu.yaw_deg);

        Serial.println("-------------");
    }
}

*/


/*
#include <Arduino.h>
#include <Wire.h>
#include "imu9250.h"
void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);

  imu9250_init();
  Serial.println("IMU9250 INIT DONE");

  // อุ่นเครื่อง: อ่านทิ้งไปรัว ๆ ให้ mag ตื่น
  Imu9250Data tmp;
  for (int i = 0; i < 100; i++) {
      imu9250_read(tmp);
      delay(10);
  }

  Serial.println("Warmup done, start mag calib...");

  // วางบอร์ดนิ่ง ๆ ระหว่างนี้
  imu9250_calibrate_gyro_accel(500);  // เก็บ 500 sample

  Serial.println("IMU + calib ready");
  
  Serial.println("Rotate the sensor in all directions for mag calib...");


  float mxmin, mxmax, mymin, mymax, mzmin, mzmax;
  imu9250_collect_mag_minmax(1000, mxmin, mxmax, mymin, mymax, mzmin, mzmax);
  imu9250_compute_mag_calib(mxmin, mxmax, mymin, mymax, mzmin, mzmax);
  
  Serial.println("Calibration done.");
}

void loop() {

}
*/

/*
#include <Arduino.h>
#include <Wire.h>
#include "imu9250.h"
void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    imu9250_init();

    delay(1000); // รอเซ็นเซอร์เซ็ตตัว

    float mx_min, mx_max, my_min, my_max, mz_min, mz_max;

    Serial.println("Rotate board in all directions...");
    delay(2000);

    imu9250_collect_mag_minmax(1000,
                               mx_min, mx_max,
                               my_min, my_max,
                               mz_min, mz_max);

    imu9250_compute_mag_calib(mx_min, mx_max,
                              my_min, my_max,
                              mz_min, mz_max);

    Serial.println("Calibration done.");
}

void loop() {
    // ว่างเปล่า หรือไม่ก็ไม่ต้องใส่อะไร
}
*/

/*
#include <Arduino.h>
#include <Wire.h>
#include "imu9250.h"

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  imu9250_init();
  Serial.println("Scanning I2C...");
  for (int i = 0; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }
  Wire.beginTransmission(0x0C);
  uint8_t err = Wire.endTransmission();
  Serial.print("AK8963 test error = ");
  Serial.println(err);

}

void loop() {}
*/


#include <Arduino.h>
#include <Wire.h>
#include "imu9250.h"

static Imu9250Data imu;

// ====== ตั้งค่า pin I2C ของ ESP32 ======
static const int SDA_PIN = 21;   // หรือ 19
static const int SCL_PIN = 22;   // หรือ 18

// ====== ใส่ค่า calibration ที่คุณได้ล่าสุด (ตัวอย่างจากของอิ๋ม) ======
// ถ้ายังไม่อยากใช้ ก็ set เป็น false ไว้ก่อน
static const bool USE_SAVED_CAL = true;

// accelBias [m/s^2] , gyroBias [rad/s], magOffset/raw , magScale
static const float AXB = 0.00f,  AYB = 0.00f,  AZB = -1.81f;
static const float GXB = 0.00f, GYB = 0.00f, GZB = 0.00f;
static const float MOX = 110.50f, MOY = 384.00f, MOZ = 54.00f;
static const float MSX = 1.10f,   MSY = 1.00f,   MSZ = 0.92f;

// ====== เลือกว่าจะคาลิเบรตทุกครั้งไหม ======
// แนะนำ: false (คาลิเบรตเมื่อสั่งเท่านั้น)
static const bool DO_CALIB_ON_BOOT = false;

void doCalibrationOnce()
{
  Serial.println("\n=== Hold still: gyro+acc calib ===");
  imu9250_calibrate_gyro_accel(2000);

  Serial.println("\n=== Rotate all directions (figure-8): mag calib ===");
  float mxmin, mxmax, mymin, mymax, mzmin, mzmax;
  imu9250_collect_mag_minmax(2000, mxmin, mxmax, mymin, mymax, mzmin, mzmax);
  imu9250_compute_mag_calib(mxmin, mxmax, mymin, mymax, mzmin, mzmax);

  Serial.println("\n=== Copy these values to main.cpp (or save to NVS later) ===");
  // ตรงนี้ “ค่าจริง” ถูกเก็บในตัวแปร static ใน imu9250.cpp แล้ว
  // วิธีง่ายสุดคือให้คุณดู Serial log ที่พิมพ์จาก compute_mag_calib + calibrate_gyro_accel
  Serial.println("Calibration done.\n");
}

void setup()
{
  Serial.begin(115200);
  delay(800);

  // 1) main เป็นคนเริ่ม I2C เอง
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // 2) init IMU
  imu9250_init();
  Serial.println("IMU9250 INIT DONE");

  // 3) warmup อ่านทิ้งให้เซนเซอร์ตื่น
  Imu9250Data tmp;
  for (int i = 0; i < 200; i++) {
    imu9250_read(tmp);
    delay(5);
  }

  // 4) โหลดค่าคาลิเบรตที่เคยได้ (โหมดใช้งานจริง)
  if (USE_SAVED_CAL) {
    imu9250_set_calibration(
      AXB, AYB, AZB,
      GXB, GYB, GZB,
      MOX, MOY, MOZ,
      MSX, MSY, MSZ
    );
    Serial.println("Loaded saved calibration.");
  }

  // 5) ถ้าต้องการคาลิเบรตตอนบูต (ไม่ค่อยแนะนำ) เปิด flag นี้
  if (DO_CALIB_ON_BOOT) {
    doCalibrationOnce();
  }

  Serial.println("Ready.");
}

void loop()
{
  imu9250_read(imu);

  static uint32_t last = 0;
  if (millis() - last > 100) {
    last = millis();

    // ตัวอย่างพิมพ์ค่าไปดู
    Serial.print("acc=");
    Serial.print(imu.ax, 2); Serial.print(",");
    Serial.print(imu.ay, 2); Serial.print(",");
    Serial.print(imu.az, 2);

    Serial.print("  gyro=");
    Serial.print(imu.gx, 3); Serial.print(",");
    Serial.print(imu.gy, 3); Serial.print(",");
    Serial.print(imu.gz, 3);

    Serial.print("  mag=");
    Serial.print(imu.mx, 1); Serial.print(",");
    Serial.print(imu.my, 1); Serial.print(",");
    Serial.print(imu.mz, 1);

    Serial.print("  rpy=");
    Serial.print(imu.roll_deg, 1); Serial.print(",");
    Serial.print(imu.pitch_deg, 1); Serial.print(",");
    Serial.println(imu.yaw_deg, 1);
  }
}
