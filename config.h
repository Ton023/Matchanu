#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// I2C Bus assignment (อย่าสับสน)
// Wire  (SDA:27, SCL:14) → MPU6050 + MS5611
// Wire1 (SDA:21, SCL:22) → TF-Luna LiDAR

// =============================================================
// PIDGains — จัดกลุ่มค่าทั้งหมดไว้ที่เดียว save/load flash ง่าย
// ค่า default = ค่าจาก main6 config.h
// หมายเหตุ: main6 ใช้ Kp_outer/Ki_outer ร่วมกัน roll+pitch
//          ในโค้ดนี้แยก roll/pitch แต่ตั้งค่าเท่ากันเพื่อให้พฤติกรรมตรงกับ main6
// =============================================================
struct PIDGains {
    // Outer angle loop (main6 ใช้ร่วม roll+pitch = 4.3 / 1.00)
    float Kp_outer_roll  = 4.20f;
    float Ki_outer_roll  = 1.20f;
    float Kp_outer_pitch = 4.20f;
    float Ki_outer_pitch = 1.20f;

    // Inner rate loop — Roll (main6)
    float Kp_roll = 1.45f;
    float Ki_roll = 0.781f;
    float Kd_roll = 1.331f;

    // Inner rate loop — Pitch (main6)
    float Kp_pitch = 1.65f;
    float Ki_pitch = 0.707f;
    float Kd_pitch = 1.331f;

    // Rate loop — Yaw (main6)
    float Kp_yaw = 8.00f;
    float Ki_yaw = 1.598f;
    float Kd_yaw = 0.00f;

    // Altitude Hold (main6 = main55 เดิม)
    float Kp_alt = 1.50f;
    float Ki_alt = 0.10f;
    float Kd_alt = 1.00f;
};

// =============================================================
// FlightControl
// =============================================================
class FlightControl {
  public:
    FlightControl();
    void init();
    bool update();
    void calibrate();
    void resetOuterIntegral();

    // [NEW 20 ต.ค.] ใช้กับปุ่ม Calibrate Level (ตั้งศูนย์ gyro แบบไม่บล็อก loop — ทำใน HAUV_FINAL.ino)
    //   getRawGyro     = ค่า gyro ดิบจาก DMP packet ล่าสุด (LSB, ±2000dps -> 16.4 LSB ต่อ deg/s)
    //   get/setGyroOffsets = register offset ในชิป (หน่วย 1/32.8 deg/s) — DMP ใช้ค่าหลังหัก offset นี้
    void getRawGyro(int16_t o[3]) const { o[0] = gyro.x; o[1] = gyro.y; o[2] = gyro.z; }
    void getGyroOffsets(int16_t o[3]) { o[0] = mpu.getXGyroOffset(); o[1] = mpu.getYGyroOffset(); o[2] = mpu.getZGyroOffset(); }
    void setGyroOffsets(const int16_t o[3]) { mpu.setXGyroOffset(o[0]); mpu.setYGyroOffset(o[1]); mpu.setZGyroOffset(o[2]); }

    // save/load ค่า PID ลง flash (คงอยู่หลังรีบูต)
    void loadGains();
    void saveGains();
    void resetGains();   // กลับเป็นค่า default ใน struct

    float ypr[3];
    float gyroRate[3];

    PIDGains g;   // ค่า PID ทั้งหมดอยู่ใน struct เดียว

    VectorFloat  gravity;
    Quaternion   q;

    // integral states (outer)
    float integral_outer_roll  = 0.0f;
    float integral_outer_pitch = 0.0f;

    // --- Altitude Hold state ---
    float integral_alt    = 0.0f;
    float prev_error_alt  = 0.0f;
    float alt_pid_output  = 0.0f;
    float setpoint_alt    = 0.0f;
    // [FIX] 1440 -> 1495: ค่า hover จริงที่จูนได้จากการบิน (แบตเต็ม ~63%)
    // ถ้าตั้งต่ำกว่าจริง ตอนกด ALT HOLD โดรนจะวูบลงก่อนแล้วค่อยไต่กลับ
    float hover_throttle  = 1495.0f;
    bool  altHoldActive   = false;

    // 🌊 Submarine mode ใช้ direct thrust (บังคับมือผ่านปุ่ม) ไม่มี PID ใต้น้ำ

    // --- Throttle / limits ---
    float throttle        = 1000.0f;
    float manual_throttle = 1000.0f;
    // [FIX] 1200 -> 2000
    // ค่านี้ถูกใช้เฉพาะ motor mix ของ AERIAL_MODE เท่านั้น
    // โหมดใต้น้ำมีลิมิตของตัวเองอยู่แล้ว (int maxThrust = 1200 ใน main11)
    // ถ้าตั้ง 1200 ที่นี่ -> โหมดบินโดนตัดที่ 1200 ทั้งที่ hover ต้องใช้ ~1440-1495
    //   = ยกตัวไม่ขึ้น + PID คุมมุมไม่ได้ (ทุกมอเตอร์ชนเพดานพร้อมกัน ผลต่างหายหมด)
    float maxThrottle     = 2000.0f;
    float minThrottle     = 1000.0f;

    // --- State flags ---
    bool          armed           = false;
    unsigned long lastCommandTime = 0;
    bool          failsafeActive  = false;

    // --- Tuning limits (public เผื่อปรับ runtime) — ค่าจาก main6 ---
    float outer_integral_limit = 20.0f;    // outer angle integral clamp (main6)
    float rate_integral_limit  = 30.0f;    // inner rate integral clamp (main6)
    float outer_rate_limit     = 200.0f;   // max target rate deg/s (main6)
    float pid_output_limit     = 350.0f;   // max motor mix contribution (main6)

  private:
    MPU6050      mpu;
    uint8_t      devStatus;
    uint16_t     packetSize;
    // [FIX เก่าจาก main55] เพิ่มจาก 64 → 128 bytes กัน stack smash ตอน FIFO overflow
    uint8_t      fifoBuffer[128];
    VectorInt16  gyro;

    float gyroLPF[3] = {0.0f, 0.0f, 0.0f};
};

// =============================================================
// BLDC_PWM — สัญญาณ ESC มอเตอร์บิน 4 ตัว
// [NEW 21 ต.ค.] เลือกโปรโตคอลได้ (ESC BLHeli รองรับทั้งสองแบบ ตรวจจับเองตอนเสียบแบต)
//   1 = OneShot125 : พัลส์ 125-250µs @1000Hz (แบบเดียวกับ main5 ที่เพื่อนบินผ่านแล้ว)
//                    ESC ได้ค่าใหม่ทุก 1ms (PWM เดิมทุก 4ms) -> ตอบสนองไวขึ้น
//   0 = PWM เดิม   : พัลส์ 1000-2000µs @250Hz
//   โค้ดส่วนอื่นยังคิดเป็น 1000-2000 เหมือนเดิมทั้งหมด (PID / MOTOR_MIN_ARMED / แบต) — แปลงที่นี่ที่เดียว
//   ⚠️ เปลี่ยนโปรโตคอลแล้ว ทดสอบโดย "ถอดใบพัด" ก่อนเสมอ
// =============================================================
#define ESC_ONESHOT125  1

class BLDC_PWM {
  private:
    uint8_t _pins[4];
  public:
    BLDC_PWM(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4);
    void begin();
    void write(uint8_t motorIndex, uint16_t pulse_us);
    void writeAll(uint16_t pulse_us);
    void controlmotor(int motorIndex, float speed);
};

#endif