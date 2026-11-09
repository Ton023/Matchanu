#include "config.h"
#include <Preferences.h>

static Preferences prefs;

// =============================================================
// Constructor — ค่า PID มาจาก struct PIDGains (config.h) แล้ว
// =============================================================
FlightControl::FlightControl() {
    integral_outer_roll  = 0.0f;
    integral_outer_pitch = 0.0f;
    for (int i = 0; i < 3; i++) gyroRate[i] = 0.0f;
    lastCommandTime = millis();
}

// =============================================================
// Flash persistence — ค่า PID คงอยู่หลังรีบูต
// (ใช้ namespace "pid" ใน NVS)
// =============================================================
void FlightControl::loadGains() {
    prefs.begin("pid", true);   // read-only
    if (prefs.isKey("saved")) {
        g.Kp_outer_roll  = prefs.getFloat("kor", g.Kp_outer_roll);
        g.Ki_outer_roll  = prefs.getFloat("kir_o", g.Ki_outer_roll);
        g.Kp_outer_pitch = prefs.getFloat("kop", g.Kp_outer_pitch);
        g.Ki_outer_pitch = prefs.getFloat("kip_o", g.Ki_outer_pitch);

        g.Kp_roll  = prefs.getFloat("kpr", g.Kp_roll);
        g.Ki_roll  = prefs.getFloat("kir", g.Ki_roll);
        g.Kd_roll  = prefs.getFloat("kdr", g.Kd_roll);

        g.Kp_pitch = prefs.getFloat("kpp", g.Kp_pitch);
        g.Ki_pitch = prefs.getFloat("kip", g.Ki_pitch);
        g.Kd_pitch = prefs.getFloat("kdp", g.Kd_pitch);

        g.Kp_yaw   = prefs.getFloat("kpy", g.Kp_yaw);
        g.Ki_yaw   = prefs.getFloat("kiy", g.Ki_yaw);
        g.Kd_yaw   = prefs.getFloat("kdy", g.Kd_yaw);

        g.Kp_alt   = prefs.getFloat("kpa", g.Kp_alt);
        g.Ki_alt   = prefs.getFloat("kia", g.Ki_alt);
        g.Kd_alt   = prefs.getFloat("kda", g.Kd_alt);
        Serial.println(F("PID: loaded from flash"));
    } else {
        Serial.println(F("PID: no saved data, using defaults"));
    }
    prefs.end();
}

void FlightControl::saveGains() {
    prefs.begin("pid", false);
    prefs.putFloat("kor",   g.Kp_outer_roll);
    prefs.putFloat("kir_o", g.Ki_outer_roll);
    prefs.putFloat("kop",   g.Kp_outer_pitch);
    prefs.putFloat("kip_o", g.Ki_outer_pitch);

    prefs.putFloat("kpr", g.Kp_roll);
    prefs.putFloat("kir", g.Ki_roll);
    prefs.putFloat("kdr", g.Kd_roll);

    prefs.putFloat("kpp", g.Kp_pitch);
    prefs.putFloat("kip", g.Ki_pitch);
    prefs.putFloat("kdp", g.Kd_pitch);

    prefs.putFloat("kpy", g.Kp_yaw);
    prefs.putFloat("kiy", g.Ki_yaw);
    prefs.putFloat("kdy", g.Kd_yaw);

    prefs.putFloat("kpa", g.Kp_alt);
    prefs.putFloat("kia", g.Ki_alt);
    prefs.putFloat("kda", g.Kd_alt);
    prefs.putBool("saved", true);
    prefs.end();
    Serial.println(F("PID: saved to flash"));
}

void FlightControl::resetGains() {
    g = PIDGains();   // กลับเป็นค่า default ใน struct
    prefs.begin("pid", false);
    prefs.clear();
    prefs.end();
    Serial.println(F("PID: reset to default"));
}

// =============================================================
// calibrate
// =============================================================
void FlightControl::calibrate() {
    mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
}

// =============================================================
// init
// =============================================================
void FlightControl::init() {
    // Wire (SDA:27, SCL:14) → MPU6050 + MS5611
    Wire.begin(27, 14);
    Wire.setClock(400000);

    mpu.initialize();
    mpu.setDLPFMode(3);   // hardware LPF ~44Hz

    // [main6] setRate(0) = 1kHz internal → DMP output สูงสุด 200Hz (ตรงกับ PID timer)
    mpu.setRate(0);

    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        Serial.println(F("Auto-Calibrating GY-86..."));
        calibrate();
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println(F("DMP Ready!"));
    } else {
        Serial.print(F("ERROR: DMP init failed! Code: "));
        Serial.println(devStatus);
        while (true) {
            Serial.println(F("ERROR: Reboot required."));
            delay(3000);
        }
    }
}

// =============================================================
// update — อ่าน DMP + gyroRate ผ่าน LPF
// alpha 0.8 (main6) เบากว่า main55 เดิม (0.5) เพราะ DLPF hw กรอง 44Hz แล้ว
// ค่าเก่า 0.5 = กรองซ้ำแรงไป ทำให้ D-term ช้า → oscillate/overshoot ง่าย
// =============================================================
bool FlightControl::update() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGyro(&gyro, fifoBuffer);

        float rawYaw   =  gyro.z / 16.4f;
        float rawPitch = -(gyro.y / 16.4f);
        float rawRoll  =  gyro.x / 16.4f;

        gyroLPF[0] = (0.8f * rawYaw)   + (0.2f * gyroLPF[0]);
        gyroLPF[1] = (0.8f * rawPitch) + (0.2f * gyroLPF[1]);
        gyroLPF[2] = (0.8f * rawRoll)  + (0.2f * gyroLPF[2]);
        gyroRate[0] = gyroLPF[0];  // Yaw
        gyroRate[1] = gyroLPF[1];  // Pitch
        gyroRate[2] = gyroLPF[2];  // Roll

        return true;
    }
    return false;
}

// =============================================================
// resetOuterIntegral
// =============================================================
void FlightControl::resetOuterIntegral() {
    integral_outer_roll  = 0.0f;
    integral_outer_pitch = 0.0f;
}

// =============================================================
// BLDC_PWM
// [NEW 21 ต.ค.] OneShot125 (ESC_ONESHOT125 = 1 ใน config.h) หรือ PWM เดิม (= 0)
//   OneShot125: 1000Hz (คาบ 1000µs) 14 bit -> ละเอียด ~0.061µs/ขั้น
//     คำสั่ง 1000-2000 -> พัลส์ 125-250µs (หาร 8)
//     คำสั่งดับ (<=1000) -> 122µs ต่ำกว่า 125 เล็กน้อย ให้ ESC เห็นว่า "ดับแน่นอน" ไม่กระตุก
//     (แบบเดียวกับ main5: duty 2000 = 122µs)
//   PWM เดิม: 250Hz (คาบ 4000µs) พัลส์ 1000-2000µs
//   LEDC: ledcAttach จองช่อง 0-3 (timer 0,1) — มอเตอร์เรือใช้ช่อง 6,7 (timer 3) ไม่ชนกัน
// =============================================================
#if ESC_ONESHOT125
  #define ESC_FREQ_HZ     1000
  #define ESC_STOP_US     122.0f
#else
  #define ESC_FREQ_HZ     250
  #define ESC_STOP_US     1000.0f
#endif
#define ESC_RES_BITS      14
#define ESC_DUTY_MAX      ((1UL << ESC_RES_BITS) - 1)          // 16383
static const float ESC_PERIOD_US = 1000000.0f / ESC_FREQ_HZ;

static inline uint32_t escDutyFromUs(float us) {
    return (uint32_t)(us * (float)ESC_DUTY_MAX / ESC_PERIOD_US + 0.5f);
}

BLDC_PWM::BLDC_PWM(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
    _pins[0] = p1; _pins[1] = p2; _pins[2] = p3; _pins[3] = p4;
}

void BLDC_PWM::begin() {
    for (int i = 0; i < 4; i++) {
        if (!ledcAttach(_pins[i], ESC_FREQ_HZ, ESC_RES_BITS)) {
            Serial.printf("ESC: LEDC attach FAILED pin %d\n", _pins[i]);
        }
        write(i, 1000);
    }
    Serial.println(ESC_ONESHOT125 ? "ESC: OneShot125 (125-250us @1000Hz)" : "ESC: PWM (1000-2000us @250Hz)");
}

void BLDC_PWM::write(uint8_t motorIndex, uint16_t pulse_us) {
    if (motorIndex >= 4) return;
    pulse_us = constrain(pulse_us, 1000, 2000);
    float out_us;
    if (pulse_us <= 1000) {
        out_us = ESC_STOP_US;                                   // ดับ
    } else {
        out_us = ESC_ONESHOT125 ? (pulse_us / 8.0f) : (float)pulse_us;
    }
    ledcWrite(_pins[motorIndex], escDutyFromUs(out_us));
}

void BLDC_PWM::writeAll(uint16_t pulse_us) {
    for (int i = 0; i < 4; i++) write(i, pulse_us);
}

void BLDC_PWM::controlmotor(int motorIndex, float speed) {
    write((uint8_t)motorIndex, (uint16_t)speed);
}