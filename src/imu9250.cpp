#include "imu9250.h"
#include <Wire.h>
#include <math.h>

// -------------------- I2C addresses --------------------
#define MPU9250_ADDR  0x68
#define AK8963_ADDR   0x0C

// -------------------- MPU-9250 registers --------------------
#define MPU9250_PWR_MGMT_1    0x6B
#define MPU9250_CONFIG        0x1A
#define MPU9250_GYRO_CONFIG   0x1B
#define MPU9250_ACCEL_CONFIG  0x1C
#define MPU9250_ACCEL_CONFIG2 0x1D
#define MPU9250_INT_PIN_CFG   0x37

#define MPU9250_ACCEL_XOUT_H  0x3B

// -------------------- AK8963 registers --------------------
#define AK8963_ST1   0x02
#define AK8963_HXL   0x03
#define AK8963_CNTL1 0x0A

// -------------------- mag calibration (ปรับเองทีหลัง) --------------------
static float magOffsetX = 0.0f;
static float magOffsetY = 0.0f;
static float magOffsetZ = 0.0f;

static float magScaleX  = 1.0f;
static float magScaleY  = 1.0f;
static float magScaleZ  = 1.0f;


// ----- NEW: accel & gyro bias -----
static float accelBiasX = 0.0f;
static float accelBiasY = 0.0f;
static float accelBiasZ = 0.0f;  // ควรใกล้ -g หรือ +g แล้วแต่ orientation

static float gyroBiasX  = 0.0f;  // [rad/s]
static float gyroBiasY  = 0.0f;
static float gyroBiasZ  = 0.0f;


// ค่าเก็บไว้ล่าสุด (ถ้า mag ยังไม่ ready จะใช้ค่าก่อนหน้า)
static Imu9250Data g_lastData;

// -------------------- I2C helpers --------------------
static void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

static void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission();  // stop ปกติ

    if (err != 0)
    {
        // สามารถเปิด debug ตรงนี้ได้ถ้าต้องการ
        // Serial.print("I2C error "); Serial.println(err);
        return;
    }

    Wire.requestFrom((int)addr, (int)count);
    int i = 0;
    while (Wire.available() && i < count)
    {
        dest[i++] = Wire.read();
    }
}

// -------------------- init MPU9250 --------------------
static void initMPU9250()
{
    // wake up
    i2cWriteByte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00);
    delay(100);

    // low-pass filter
    i2cWriteByte(MPU9250_ADDR, MPU9250_CONFIG, 0x03);       // ~44 Hz

    // gyro ±500 dps → 65.5 LSB/dps (เหมือนโค้ด mpu6050 เดิม)
    i2cWriteByte(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x08);  // FS_SEL=1

    // accel ±8 g → 4096 LSB/g
    i2cWriteByte(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0x10); // AFS_SEL=2

    // accel DLPF
    i2cWriteByte(MPU9250_ADDR, MPU9250_ACCEL_CONFIG2, 0x03);

    // เปิด I2C bypass เพื่อให้ MCU คุยกับ AK8963 ได้ตรง ๆ
    i2cWriteByte(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);

    delay(10);
}

// -------------------- init AK8963 (mag) --------------------
static void initAK8963()
{
    // power down
    i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x00);
    delay(10);

    // 16-bit output, continuous mode 2 (100 Hz)
    // [bit4:5]=01 (16-bit), [bit0:3]=0110 (cont.2) => 0x16
    i2cWriteByte(AK8963_ADDR, AK8963_CNTL1, 0x16);
    delay(10);
}

// -------------------- read accel + gyro --------------------
static void readAccelGyro(Imu9250Data &out)
{
    uint8_t buf[14];
    i2cReadBytes(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 14, buf);

    int16_t ax_raw = (int16_t)(buf[0]  << 8 | buf[1]);
    int16_t ay_raw = (int16_t)(buf[2]  << 8 | buf[3]);
    int16_t az_raw = (int16_t)(buf[4]  << 8 | buf[5]);
    // int16_t temp_raw = (int16_t)(buf[6]  << 8 | buf[7]);
    int16_t gx_raw = (int16_t)(buf[8]  << 8 | buf[9]);
    int16_t gy_raw = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t gz_raw = (int16_t)(buf[12] << 8 | buf[13]);

    // Accel: ±8g → 4096 LSB/g → แปลงเป็น m/s^2
    const float g = 9.80665f;
    float ax_g = (float)ax_raw / 4096.0f;
    float ay_g = (float)ay_raw / 4096.0f;
    float az_g = (float)az_raw / 4096.0f;


    // NEW: ลบ bias
    out.ax = ax_g * g - accelBiasX;
    out.ay = ay_g * g - accelBiasY;
    out.az = az_g * g - accelBiasZ;

    // Gyro: ±500 dps → 65.5 LSB/dps → แปลงเป็น rad/s
    float gx_dps = (float)gx_raw / 65.5f;
    float gy_dps = (float)gy_raw / 65.5f;
    float gz_dps = (float)gz_raw / 65.5f;

    const float DEG2RAD = PI / 180.0f;

    // NEW: ลบ bias
    out.gx = gx_dps * DEG2RAD - gyroBiasX;
    out.gy = gy_dps * DEG2RAD - gyroBiasY;
    out.gz = gz_dps * DEG2RAD - gyroBiasZ;
}

// -------------------- read magnetometer --------------------
static bool readMag(Imu9250Data &out)
{
    uint8_t st1 = 0;
    i2cReadBytes(AK8963_ADDR, AK8963_ST1, 1, &st1);

    // DRDY ไม่มา = ไม่มี sample ใหม่
    if (!(st1 & 0x01)) return false;

    uint8_t buf[7] = {0};
    i2cReadBytes(AK8963_ADDR, AK8963_HXL, 7, buf);

    // ST2 overflow bit
    uint8_t st2 = buf[6];
    if (st2 & 0x08) return false;

    int16_t mx_raw = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t my_raw = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t mz_raw = (int16_t)(buf[5] << 8 | buf[4]);

    float mx_f = (float)mx_raw;
    float my_f = (float)my_raw;
    float mz_f = (float)mz_raw;

    // apply hard-iron + soft-iron
    mx_f = (mx_f - magOffsetX) * magScaleX;
    my_f = (my_f - magOffsetY) * magScaleY;
    mz_f = (mz_f - magOffsetZ) * magScaleZ;

    out.mx = mx_f;
    out.my = my_f;
    out.mz = mz_f;

    return true; // ✅ บอกว่า sample นี้ "ใหม่"
}


// -------------------- compute angles (roll, pitch, yaw) --------------------
static void computeAngles(Imu9250Data &out)
{
    // ใช้ accel หา roll/pitch
    float ax = out.ax;
    float ay = out.ay;
    float az = out.az;

    // แปลงกลับเป็น g ชั่วคราว (เพราะสูตรเดิมใช้ g)
    const float g = 9.80665f;
    float ax_g = ax / g;
    float ay_g = ay / g;
    float az_g = az / g;

    float roll  = atan2( ay_g, sqrt(ax_g * ax_g + az_g * az_g) );
    float pitch = -atan2( ax_g, sqrt(ay_g * ay_g + az_g * az_g) );

    out.roll_deg  = roll  * 180.0f / PI;
    out.pitch_deg = pitch * 180.0f / PI;

    // tilt-compensated mag
    float mx = out.mx;
    float my = out.my;
    float mz = out.mz;

    // ชดเชย pitch ก่อน
    float mx2 = mx * cos(pitch) + mz * sin(pitch);
    float my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    float yaw = atan2(-my2, mx2);
    float yaw_deg = yaw * 180.0f / PI;
    if (yaw_deg < 0.0f)
        yaw_deg += 360.0f;

    out.yaw_deg = yaw_deg;
}

// -------------------- public API --------------------
void imu9250_init()
{
    // ถ้าใช้ ESP32: อย่าลืมเรียก Wire.begin(SDA,SCL) ใน main ก่อน
    Wire.begin();
    Wire.setClock(400000);

    initMPU9250();
    initAK8963();

    // clear data
    memset(&g_lastData, 0, sizeof(g_lastData));
}

bool imu9250_read(Imu9250Data &out)
{
    out = g_lastData;

    readAccelGyro(out);

    bool magUpdated = readMag(out);   // ✅ ได้/ไม่ได้

    computeAngles(out);

    g_lastData = out;

    return true;
}


void imu9250_calibrate_gyro_accel(int samples)
{
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < samples; i++)
    {
        uint8_t buf[14];
        i2cReadBytes(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 14, buf);

        int16_t ax_raw = (int16_t)(buf[0]  << 8 | buf[1]);
        int16_t ay_raw = (int16_t)(buf[2]  << 8 | buf[3]);
        int16_t az_raw = (int16_t)(buf[4]  << 8 | buf[5]);
        int16_t gx_raw = (int16_t)(buf[8]  << 8 | buf[9]);
        int16_t gy_raw = (int16_t)(buf[10] << 8 | buf[11]);
        int16_t gz_raw = (int16_t)(buf[12] << 8 | buf[13]);

        ax_sum += ax_raw;
        ay_sum += ay_raw;
        az_sum += az_raw;
        gx_sum += gx_raw;
        gy_sum += gy_raw;
        gz_sum += gz_raw;

        delay(5);
    }

    float ax_avg = (float)ax_sum / samples;
    float ay_avg = (float)ay_sum / samples;
    float az_avg = (float)az_sum / samples;

    float gx_avg = (float)gx_sum / samples;
    float gy_avg = (float)gy_sum / samples;
    float gz_avg = (float)gz_sum / samples;

    // scale เดิม: accel ±8g => 4096 LSB/g, gyro ±500 dps => 65.5 LSB/dps
    const float g = 9.80665f;
    const float ACC_LSB = 4096.0f;
    const float GYRO_LSB = 65.5f;
    const float DEG2RAD = PI / 180.0f;

    // สมมติว่าเวลาตั้งนิ่ง แกน Z ชี้ขึ้น: az_raw ≈ +1 g
    accelBiasX = (ax_avg / ACC_LSB) * g;          // ควรใกล้ 0
    accelBiasY = (ay_avg / ACC_LSB) * g;
    accelBiasZ = ((az_avg - ACC_LSB) / ACC_LSB) * g; // ลบ 1g ออก

    gyroBiasX = (gx_avg / GYRO_LSB) * DEG2RAD;
    gyroBiasY = (gy_avg / GYRO_LSB) * DEG2RAD;
    gyroBiasZ = (gz_avg / GYRO_LSB) * DEG2RAD;

    Serial.println("IMU9250 calib done.");
    Serial.print("accelBias [m/s^2] = ");
    Serial.print(accelBiasX); Serial.print(", ");
    Serial.print(accelBiasY); Serial.print(", ");
    Serial.println(accelBiasZ);

    Serial.print("gyroBias [rad/s] = ");
    Serial.print(gyroBiasX); Serial.print(", ");
    Serial.print(gyroBiasY); Serial.print(", ");
    Serial.println(gyroBiasZ);
}

void imu9250_collect_mag_minmax(int samples,
                                float &mx_min, float &mx_max,
                                float &my_min, float &my_max,
                                float &mz_min, float &mz_max)
{
    mx_min = my_min = mz_min =  1e9;
    mx_max = my_max = mz_max = -1e9;

    Imu9250Data d;
    int got = 0;

    while (got < samples)
    {
        imu9250_read(d);

        // ✅ ถ้าค่า mag ยังเป็นศูนย์/ค้าง (เช่น DRDY ไม่มา) ให้ข้าม
        // วิธีที่ชัวร์กว่า: “เช็คว่ามีการเปลี่ยนจริง” โดยเทียบกับค่าก่อนหน้า
        static float lastMx = 123456, lastMy = 123456, lastMz = 123456;
        bool changed = (d.mx != lastMx) || (d.my != lastMy) || (d.mz != lastMz);

        if (!changed) {
            // Serial.println("skip sample (no new mag data)");
            delay(5);
            continue;
        }

        lastMx = d.mx; lastMy = d.my; lastMz = d.mz;

        // อัปเดต min/max เฉพาะตอนมี sample ใหม่
        if (d.mx < mx_min) mx_min = d.mx;
        if (d.mx > mx_max) mx_max = d.mx;

        if (d.my < my_min) my_min = d.my;
        if (d.my > my_max) my_max = d.my;

        if (d.mz < mz_min) mz_min = d.mz;
        if (d.mz > mz_max) mz_max = d.mz;

        got++;

        // debug
        //Serial.print("mag sample "); Serial.print(got);
        //Serial.print(" | mx="); Serial.print(d.mx);
        //Serial.print(" my="); Serial.print(d.my);
        //Serial.print(" mz="); Serial.println(d.mz);

        delay(20);
    }

    Serial.print("mx_min="); Serial.print(mx_min);
    Serial.print(" mx_max="); Serial.println(mx_max);
    Serial.print("my_min="); Serial.print(my_min);
    Serial.print(" my_max="); Serial.println(my_max);
    Serial.print("mz_min="); Serial.print(mz_min);
    Serial.print(" mz_max="); Serial.println(mz_max);
}


void imu9250_compute_mag_calib(float mx_min, float mx_max,
                               float my_min, float my_max,
                               float mz_min, float mz_max)
{
    magOffsetX = (mx_max + mx_min) * 0.5f;
    magOffsetY = (my_max + my_min) * 0.5f;
    magOffsetZ = (mz_max + mz_min) * 0.5f;

    float mx_range = (mx_max - mx_min) * 0.5f;
    float my_range = (my_max - my_min) * 0.5f;
    float mz_range = (mz_max - mz_min) * 0.5f;

    Serial.print("range X = "); Serial.println(mx_range);
    Serial.print("range Y = "); Serial.println(my_range);
    Serial.print("range Z = "); Serial.println(mz_range);

    // ป้องกันหาร 0
    if (mx_range < 1e-3f || my_range < 1e-3f || mz_range < 1e-3f) {
        Serial.println("WARNING: mag axis range too small! calib may be invalid.");
    }

    float avg = (mx_range + my_range + mz_range) / 3.0f;

    // ถ้าแกนไหน range เล็กเกิน ให้ไม่ scale แกนนั้นก่อน (=1.0f)
    magScaleX = (mx_range < 1e-3f) ? 1.0f : (avg / mx_range);
    magScaleY = (my_range < 1e-3f) ? 1.0f : (avg / my_range);
    magScaleZ = (mz_range < 1e-3f) ? 1.0f : (avg / mz_range);

    Serial.println("Mag calib:");
    Serial.print("offset = ");
    Serial.print(magOffsetX); Serial.print(", ");
    Serial.print(magOffsetY); Serial.print(", ");
    Serial.println(magOffsetZ);

    Serial.print("scale  = ");
    Serial.print(magScaleX); Serial.print(", ");
    Serial.print(magScaleY); Serial.print(", ");
    Serial.println(magScaleZ);
}

void imu9250_set_calibration(
  float axb, float ayb, float azb,
  float gxb, float gyb, float gzb,
  float mox, float moy, float moz,
  float msx, float msy, float msz)
{
  accelBiasX = axb; accelBiasY = ayb; accelBiasZ = azb;
  gyroBiasX  = gxb; gyroBiasY  = gyb; gyroBiasZ  = gzb;
  magOffsetX = mox; magOffsetY = moy; magOffsetZ = moz;
  magScaleX  = msx; magScaleY  = msy; magScaleZ  = msz;
}
