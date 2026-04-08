/*
 * MPU6050 -> USB CSV pre ROS 2 (imu_serial_fusion_bridge)
 * Format 20 poli: pozri hlavicku v loop() po kalibracii.
 *
 * Otvor tento priecinok v Arduino IDE ako sketch "mpu6050_serial".
 */

#include <Wire.h>
#include <math.h>

const uint8_t IMU_ADDR = 0x68;

const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

const unsigned long LOOP_US = 10000; // 100 Hz
unsigned long last_loop_us = 0;

// default scales
const float ACCEL_SCALE = 16384.0f; // +/-2g
const float GYRO_SCALE  = 131.0f;   // +/-250 dps

// gyro offsets
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

// low-pass filtered accel
float ax_f = 0.0f;
float ay_f = 0.0f;
float az_f = 0.0f;
const float ACCEL_ALPHA = 0.2f;   // 0..1, vacsie = menej filtrovania

// filtered orientation
float roll_deg  = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg   = 0.0f; // len gyro integracia, bude driftovat

const float COMP_ALPHA = 0.98f;   // viac gyro, menej accel

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t n = Wire.requestFrom(IMU_ADDR, count);
  if (n != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
  return true;
}

int16_t makeInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

bool readRaw14(
  int16_t &ax, int16_t &ay, int16_t &az,
  int16_t &temp_raw,
  int16_t &gx, int16_t &gy, int16_t &gz
) {
  uint8_t data[14];
  if (!readBytes(REG_ACCEL_XOUT_H, 14, data)) {
    return false;
  }

  ax = makeInt16(data[0],  data[1]);
  ay = makeInt16(data[2],  data[3]);
  az = makeInt16(data[4],  data[5]);
  temp_raw = makeInt16(data[6],  data[7]);
  gx = makeInt16(data[8],  data[9]);
  gy = makeInt16(data[10], data[11]);
  gz = makeInt16(data[12], data[13]);
  return true;
}

void calibrateGyro(int samples = 1000) {
  long gx_sum = 0;
  long gy_sum = 0;
  long gz_sum = 0;

  Serial.println("CALIBRATING_GYRO_KEEP_SENSOR_STILL");

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, temp_raw, gx, gy, gz;
    if (readRaw14(ax, ay, az, temp_raw, gx, gy, gz)) {
      gx_sum += gx;
      gy_sum += gy;
      gz_sum += gz;
    }
    delay(2);
  }

  gx_offset = (float)gx_sum / samples;
  gy_offset = (float)gy_sum / samples;
  gz_offset = (float)gz_sum / samples;

  Serial.print("GYRO_OFFSETS_RAW,");
  Serial.print(gx_offset); Serial.print(",");
  Serial.print(gy_offset); Serial.print(",");
  Serial.println(gz_offset);
}

void setupMPU() {
  // wake up
  writeReg(REG_PWR_MGMT_1, 0x00);
  delay(100);

  // DLPF ~44/42 Hz
  writeReg(REG_CONFIG, 0x03);

  // gyro +/-250 dps
  writeReg(REG_GYRO_CONFIG, 0x00);

  // accel +/-2g
  writeReg(REG_ACCEL_CONFIG, 0x00);

  // 1kHz / (1 + 9) = 100Hz
  writeReg(REG_SMPLRT_DIV, 0x09);

  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  setupMPU();
  calibrateGyro();

  int16_t ax, ay, az, temp_raw, gx, gy, gz;
  if (readRaw14(ax, ay, az, temp_raw, gx, gy, gz)) {
    float ax_g = ax / ACCEL_SCALE;
    float ay_g = ay / ACCEL_SCALE;
    float az_g = az / ACCEL_SCALE;

    ax_f = ax_g;
    ay_f = ay_g;
    az_f = az_g;

    roll_deg  = atan2(ay_f, az_f) * 180.0f / PI;
    pitch_deg = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * 180.0f / PI;
    yaw_deg = 0.0f;
  }

  last_loop_us = micros();

  Serial.println("ts_ms,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,temp_c,gx_dps,gy_dps,gz_dps,roll_acc,pitch_acc,roll_f,pitch_f,yaw_gyro");
}

void loop() {
  unsigned long now = micros();
  if ((unsigned long)(now - last_loop_us) < LOOP_US) return;

  float dt = (now - last_loop_us) / 1000000.0f;
  last_loop_us = now;

  int16_t ax, ay, az, temp_raw, gx, gy, gz;
  if (!readRaw14(ax, ay, az, temp_raw, gx, gy, gz)) {
    Serial.println("ERR,READ_FAIL");
    return;
  }

  // raw -> physical units
  float ax_g = ax / ACCEL_SCALE;
  float ay_g = ay / ACCEL_SCALE;
  float az_g = az / ACCEL_SCALE;

  float gx_dps = (gx - gx_offset) / GYRO_SCALE;
  float gy_dps = (gy - gy_offset) / GYRO_SCALE;
  float gz_dps = (gz - gz_offset) / GYRO_SCALE;

  float temp_c = (temp_raw / 340.0f) + 36.53f;

  // low-pass accel
  ax_f = ACCEL_ALPHA * ax_g + (1.0f - ACCEL_ALPHA) * ax_f;
  ay_f = ACCEL_ALPHA * ay_g + (1.0f - ACCEL_ALPHA) * ay_f;
  az_f = ACCEL_ALPHA * az_g + (1.0f - ACCEL_ALPHA) * az_f;

  // angles from accel
  float roll_acc  = atan2(ay_f, az_f) * 180.0f / PI;
  float pitch_acc = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * 180.0f / PI;

  // integrate gyro
  float roll_gyro  = roll_deg  + gx_dps * dt;
  float pitch_gyro = pitch_deg + gy_dps * dt;
  yaw_deg += gz_dps * dt; // bez magnetometra driftuje

  // complementary filter
  roll_deg  = COMP_ALPHA * roll_gyro  + (1.0f - COMP_ALPHA) * roll_acc;
  pitch_deg = COMP_ALPHA * pitch_gyro + (1.0f - COMP_ALPHA) * pitch_acc;

  Serial.print(millis()); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(temp_raw); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(ax_g, 4); Serial.print(",");
  Serial.print(ay_g, 4); Serial.print(",");
  Serial.print(az_g, 4); Serial.print(",");
  Serial.print(temp_c, 2); Serial.print(",");
  Serial.print(gx_dps, 4); Serial.print(",");
  Serial.print(gy_dps, 4); Serial.print(",");
  Serial.print(gz_dps, 4); Serial.print(",");
  Serial.print(roll_acc, 3); Serial.print(",");
  Serial.print(pitch_acc, 3); Serial.print(",");
  Serial.print(roll_deg, 3); Serial.print(",");
  Serial.print(pitch_deg, 3); Serial.print(",");
  Serial.println(yaw_deg, 3);
}
