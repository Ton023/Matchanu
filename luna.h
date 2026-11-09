#ifndef LUNA_H
#define LUNA_H

#include <Wire.h>

// Wire1 (SDA:21, SCL:22) → TF-Luna LiDAR (แยกจาก Wire ที่ใช้กับ MPU6050)
//
// [FIX 21 ต.ค.] ค่ากระโดด 4-5-8-10-4-5-2-3 cm
//   main5 มี LPF (0.3 ใหม่ + 0.7 เก่า) แต่ main11/HAUV_FINAL ส่งค่าดิบตรงๆ -> ค่าไม่นิ่ง
//   แก้: (1) อ่าน 4 byte = ระยะ + ความแรงสัญญาณ (Amp) -> ทิ้งค่าที่สัญญาณอ่อน
//        (2) median 3 ค่าล่าสุด -> ตัดค่ากระโดดครั้งเดียว
//        (3) LPF แบบเดียวกับ main5 (alpha 0.3)
//        (4) อ่านพลาดสั้นๆ ไม่เกิน LUNA_HOLD_MS -> ใช้ค่าเดิมต่อ (เดิม: พลาดครั้งเดียว = 0 ทันที)
//   หมายเหตุ: TF-Luna วัดแม่นตั้งแต่ ~20 cm ขึ้นไป — วางบนพื้น (<20 cm) ค่าจะไม่ค่อยแม่นอยู่แล้ว

// ---- ค่าปรับแต่ง ----
#define LUNA_ALPHA      0.30f   // LPF: ↑ ตอบสนองไว แต่สั่น | ↓ นิ่ง แต่ตามช้า (main5 = 0.3)
#define LUNA_MIN_AMP    100     // ความแรงสัญญาณต่ำกว่านี้ = ไม่เชื่อ (ตามคู่มือ TF-Luna)
#define LUNA_HOLD_MS    150     // อ่านไม่ได้ติดกันไม่เกินนี้ = ใช้ค่าเดิม | เกิน = 0 (ไม่มีค่า)
#define LUNA_MAX_CM     800

class LunaSensor {
private:
    const uint8_t TFLUNA_ADDR = 0x10;
    int      distance = 0;        // ระยะหลังกรอง (cm) — 0 = ไม่มีค่าที่เชื่อถือได้
    float    filt = 0.0f;
    int      hist[3] = {0, 0, 0};
    uint8_t  histN = 0, histI = 0;
    uint16_t amp = 0;             // ความแรงสัญญาณล่าสุด (ไว้ดูตอนไล่ปัญหา)
    int      rawLast = 0;         // ค่าดิบล่าสุด
    uint32_t lastGoodMs = 0;
    uint8_t  errCount = 0;
    int _sda = 21, _scl = 22;

    // อ่านดิบ 4 byte: Dist_L, Dist_H, Amp_L, Amp_H (register 0x00-0x03)
    // [FIX เดิม] endTransmission(true) = ส่ง STOP เสมอ ไม่ยึดบัสค้าง
    bool readRaw(int &dist, uint16_t &a) {
        Wire1.beginTransmission(TFLUNA_ADDR);
        Wire1.write(0x00);
        if (Wire1.endTransmission(true) != 0) return false;
        if (Wire1.requestFrom(TFLUNA_ADDR, (uint8_t)4) != 4) return false;
        uint8_t dl = Wire1.read(), dh = Wire1.read();
        uint8_t al = Wire1.read(), ah = Wire1.read();
        dist = dl | (dh << 8);
        a    = al | (ah << 8);
        return true;
    }

    int median3() {
        if (histN == 1) return hist[0];
        if (histN == 2) return (hist[0] + hist[1]) / 2;
        int a = hist[0], b = hist[1], c = hist[2];
        if (a > b) { int t = a; a = b; b = t; }
        if (b > c) { int t = b; b = c; c = t; }
        if (a > b) { int t = a; a = b; b = t; }
        return b;
    }

    void clearFilter() { filt = 0.0f; histN = 0; histI = 0; distance = 0; }

public:
    bool begin(int sda = 21, int scl = 22) {
        _sda = sda; _scl = scl;
        Wire1.begin(sda, scl);
        Wire1.setClock(100000);
        // [PERF] default timeout 50ms — ถ้าบัสค้าง loop() จะหยุดรอ 50ms ต่อครั้ง
        //   transaction ปกติบนบัสนี้ใช้ < 1ms ตั้ง 5ms ก็เหลือเฟือ
        Wire1.setTimeOut(5);
        Wire1.beginTransmission(TFLUNA_ADDR);
        return (Wire1.endTransmission() == 0);
    }

    // เรียก ~50Hz (ดู loop() ใน HAUV_FINAL.ino)
    void update() {
        int d; uint16_t a;
        bool ok = readRaw(d, a);
        if (!ok) {
            if (++errCount > 20) { errCount = 0; recoverBus(); }
        } else {
            errCount = 0;
            rawLast = d;
            amp = a;
        }

        bool valid = ok && d > 0 && d <= LUNA_MAX_CM && a >= LUNA_MIN_AMP;
        if (valid) {
            hist[histI] = d;
            histI = (histI + 1) % 3;
            if (histN < 3) histN++;
            float m = (float)median3();
            filt = (filt <= 0.0f) ? m : (LUNA_ALPHA * m + (1.0f - LUNA_ALPHA) * filt);
            distance = (int)(filt + 0.5f);
            lastGoodMs = millis();
        } else if (millis() - lastGoodMs > LUNA_HOLD_MS) {
            clearFilter();      // ไม่มีค่าดีนานเกินไป -> 0 (alt hold จะไม่ใช้ค่านี้)
        }
        // พลาดสั้นๆ (< LUNA_HOLD_MS) -> distance คงค่าเดิม
    }

    // ปลดบัสที่ค้าง: สั่ง SCL 9 pulse ให้ slave ปล่อย SDA แล้วส่ง STOP
    void recoverBus() {
        Wire1.end();
        pinMode(_scl, OUTPUT);
        pinMode(_sda, INPUT_PULLUP);
        for (int i = 0; i < 9; i++) {
            digitalWrite(_scl, HIGH); delayMicroseconds(5);
            digitalWrite(_scl, LOW);  delayMicroseconds(5);
        }
        digitalWrite(_scl, HIGH); delayMicroseconds(5);
        pinMode(_sda, OUTPUT);
        digitalWrite(_sda, LOW);  delayMicroseconds(5);
        digitalWrite(_sda, HIGH);           // STOP condition
        delayMicroseconds(5);
        Wire1.begin(_sda, _scl);
        Wire1.setClock(100000);
        Wire1.setTimeOut(5);
    }

    int getDistance() {                     // ระยะหลังกรอง (cm), 0 = ไม่มีค่า
        if (distance < 0 || distance > LUNA_MAX_CM) return 0;
        return distance;
    }
    float    getDistanceF() const { return filt; }    // แบบทศนิยม (cm)
    int      getRaw()       const { return rawLast; } // ค่าดิบล่าสุด (ไล่ปัญหา)
    uint16_t getAmp()       const { return amp; }     // ความแรงสัญญาณล่าสุด
};

#endif
