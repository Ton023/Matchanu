#ifndef BOAT_ESC_H
#define BOAT_ESC_H

#include <Arduino.h>

// =============================================================
// BoatESC — ขับมอเตอร์เรือ 2 ตัว (ซ้าย/ขวา) ผ่าน ESC แบบกลับทางได้ (Bidirectional)
//
// สัญญาณ: servo PWM มาตรฐาน 50Hz (คาบ 20ms)
//
// [FIX 15 ต.ค.] ค่าจากการเทสบนโต๊ะกับ ESC จริง (Bidirectional ESC 40A, BE4108):
//   1) จุดหยุดจริง = 1480µs (ไม่ใช่ 1500) — มอเตอร์นิ่งช่วง 1475-1485 เท่านั้น (±5 แคบมาก)
//        ส่ง 1500 = ESC มองเป็น "เดินหน้าเบาๆ" -> ปล่อยสติ๊กแล้วมอเตอร์หมุนเอง
//   2) ESC ต้อง "ได้ไฟก่อน แล้วค่อยเห็นสัญญาณ" ถึงจะ arm
//        ถ้าเห็นสัญญาณตั้งแต่ตอนเสียบแบต -> เงียบ ไม่ arm ไม่หมุน
//        -> ตอนเริ่ม begin() "ยังไม่ส่งพัลส์" (สาย LOW) รอ enable() ค่อยส่งจุดหยุด
//   3) ESC ไม่ยอมออกตัวถ้าค่ากระโดด (sensorless + มอเตอร์ขั้วเยอะ) -> ต้องไล่ค่าช้าๆ เสมอ
//   ⚠️ ESC ชุดใหม่/เปลี่ยน ESC -> หาจุดหยุดใหม่ด้วย BoatMotorTest (คำสั่ง P / Z)
//
//   ห้ามเรียก motors.writeAll(1000) กับ ESC ตัวนี้ (1000 = ถอยเต็มแรง)
//
// LEDC channel: ใช้ channel 6,7 (timer 3) แยกจากมอเตอร์บิน (channel 0-3 ที่ 250Hz)
// =============================================================
#ifndef BOAT_NEUTRAL_US
#define BOAT_NEUTRAL_US 1480
#endif

class BoatESC {
  private:
    uint8_t _pinL, _pinR;
    uint8_t _chL, _chR;
    static const uint32_t FREQ_HZ   = 50;
    static const uint8_t  RES_BITS  = 14;                     // 20000µs / 16383 = ~1.22µs ต่อขั้น
    static const uint32_t PERIOD_US = 1000000UL / FREQ_HZ;     // 20000µs

    float _outL = BOAT_NEUTRAL_US;   // ค่าที่ส่งออกจริงตอนนี้ (หลังผ่าน slew)
    float _outR = BOAT_NEUTRAL_US;
    bool  _ok   = false;
    bool  _enabled = false;          // false = ไม่ส่งพัลส์เลย (สาย LOW) — ESC ยังไม่ arm

    // ---- [SAFETY] Reverse protection ----
    // เปลี่ยนทิศหมุน ต้อง: ลดลงถึงจุดหยุดก่อน -> พักครบ REVERSE_DWELL_MS -> ค่อยไปอีกทาง
    static const uint32_t REVERSE_DWELL_MS = 200;
    int8_t   _lastDirL = 0,  _lastDirR = 0;
    uint32_t _neutralSinceL = 0, _neutralSinceR = 0;

    static int8_t dirOf(float us) {
        if (us > BOAT_NEUTRAL_US + 0.5f) return  1;
        if (us < BOAT_NEUTRAL_US - 0.5f) return -1;
        return 0;
    }

    static float stepMotor(float out, float target, float slew,
                           int8_t &lastDir, uint32_t &neutralSince, uint32_t now) {
        int8_t odir = dirOf(out);
        int8_t tdir = dirOf(target);
        if (odir != 0 && tdir != 0 && tdir != odir) {
            target = BOAT_NEUTRAL_US;                         // ยังหมุนอยู่อีกทาง -> ลงจุดหยุดก่อน
        } else if (odir == 0 && tdir != 0 && lastDir != 0 && tdir != lastDir
                   && (now - neutralSince) < REVERSE_DWELL_MS) {
            target = BOAT_NEUTRAL_US;                         // อยู่ที่จุดหยุดแล้วแต่ยังพักไม่ครบ
        }
        out += constrain(target - out, -slew, slew);
        int8_t ndir = dirOf(out);
        if (ndir == 0) {
            out = BOAT_NEUTRAL_US;                            // snap กันเศษทศนิยมค้าง
            if (odir != 0) { lastDir = odir; neutralSince = now; }
        } else {
            lastDir = ndir;
        }
        return out;
    }

    void writeUs(uint8_t pin, float us) {
        if (!_enabled) { ledcWrite(pin, 0); return; }         // ไม่มีพัลส์
        us = constrain(us, 1000.0f, 2000.0f);
        uint32_t duty = (uint32_t)(us * (float)((1UL << RES_BITS) - 1) / (float)PERIOD_US + 0.5f);
        ledcWrite(pin, duty);
    }

  public:
    BoatESC(uint8_t pinL, uint8_t pinR, uint8_t chL, uint8_t chR)
        : _pinL(pinL), _pinR(pinR), _chL(chL), _chR(chR) {}

    // เรียกครั้งเดียวใน setup() — attach แต่ "ยังไม่ส่งพัลส์" (ESC รุ่นนี้ต้องได้ไฟก่อนเห็นสัญญาณ)
    bool begin() {
        bool a = ledcAttachChannel(_pinL, FREQ_HZ, RES_BITS, _chL);
        bool b = ledcAttachChannel(_pinR, FREQ_HZ, RES_BITS, _chR);
        _ok = a && b;
        _enabled = false;
        neutral();
        return _ok;
    }

    // เริ่มส่งจุดหยุด -> ESC arm (บี๊บ) — เรียกหลังจากแบตเข้า ESC แล้ว
    void enable()  { _enabled = true;  neutral(); }
    // หยุดส่งพัลส์ (แบตหลุด) -> พอเสียบแบตใหม่ จะ enable() ใหม่ให้ ESC arm อีกรอบ
    void disable() { neutral(); _enabled = false; writeUs(_pinL, 0); writeUs(_pinR, 0); }
    bool isEnabled() const { return _enabled; }

    // ไล่ค่าออกเข้าหาเป้า (µs) โดยเปลี่ยนได้ไม่เกิน slewUs ต่อการเรียก 1 ครั้ง
    void drive(float targetL, float targetR, float slewUs) {
        targetL = constrain(targetL, 1000.0f, 2000.0f);
        targetR = constrain(targetR, 1000.0f, 2000.0f);
        uint32_t now = millis();
        _outL = stepMotor(_outL, targetL, slewUs, _lastDirL, _neutralSinceL, now);
        _outR = stepMotor(_outR, targetR, slewUs, _lastDirR, _neutralSinceR, now);
        writeUs(_pinL, _outL);
        writeUs(_pinR, _outR);
    }

    // หยุดทันที (ไม่ผ่าน slew) — ใช้ตอนล็อก / failsafe / สลับโหมด
    void neutral() {
        uint32_t now = millis();
        if (dirOf(_outL) != 0) { _lastDirL = dirOf(_outL); _neutralSinceL = now; }
        if (dirOf(_outR) != 0) { _lastDirR = dirOf(_outR); _neutralSinceR = now; }
        _outL = BOAT_NEUTRAL_US;
        _outR = BOAT_NEUTRAL_US;
        writeUs(_pinL, _outL);
        writeUs(_pinR, _outR);
    }

    bool  isOK() const { return _ok; }
    float getL() const { return _outL; }
    float getR() const { return _outR; }
};

#endif
