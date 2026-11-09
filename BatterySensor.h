#ifndef BATTERY_SENSOR_H
#define BATTERY_SENSOR_H

#include <Arduino.h>

class BatterySensor {
private:
  uint8_t pin;

  // [FIX จาก main5] แบตจริงเป็น 4S (16.8V เต็ม) ไม่ใช่ 3S แบบเดิม
  // main5 ใช้ ratio=9.00 (R1=12k/R2=1.5k) เป็นค่าที่ผ่านการ calibrate จริงบนบอร์ดนี้
  // ratio จริงของวงจร: R1=12k (ฝั่งแบต) + R2=1.5k (ฝั่ง GND) -> (12+1.5)/1.5 = 9.00
  // ที่ 4S เต็ม 16.8V -> ขา ADC ได้ 1.87V (ปลอดภัย ห่างจากเพดาน 3.3V เยอะ)
  float vDividerRatio = 9.00;

  // [FIX] 1.0V -> 0.4V
  // ค่านี้คือ "แรงดันที่คาดว่าจะตกตอนคันเร่งสุด" ใช้บวกกลับเพื่อประมาณแรงดันตอนพัก
  // ถ้าตั้งสูงกว่าที่ตกจริง -> บวกเกิน -> % เด้งขึ้นตอนมอเตอร์หมุน (ผิดธรรมชาติ)
  // 4S ที่ hover (PWM~1450) แบตดีๆ ตกราว 0.2-0.4V เท่านั้น
  // วิธีหาค่าจริง: วัด V ตอนนิ่ง -> ดันคันเร่งค้าง -> วัดอีกครั้ง -> ผลต่าง / throttlePct
  float maxSagVoltage = 0.4;

  // เก็บเป็น "มิลลิโวลต์" (ไม่ใช่ค่าดิบ 0-4095 แบบเดิม)
  float emaMilliVolts = 1800.0;
  float actualVoltage = 0.0;
  float compensatedVoltage = 0.0;
  float displayedPercentage = -1.0;
  float minSeenVoltage = 99.0;   // แรงดันต่ำสุดที่เคยเห็น — ใช้ตรวจว่าเปลี่ยน/ชาร์จแบตหรือยัง

  unsigned long lastUpdate = 0;

  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

public:
  void begin(uint8_t batteryPin) {
    pin = batteryPin;
    pinMode(pin, INPUT);
    analogSetPinAttenuation(pin, ADC_11db);   // ช่วง 0-3.3V
    emaMilliVolts = analogReadMilliVolts(pin);
  }

  void setRatio(float newRatio) { vDividerRatio = newRatio; }
  void setMaxSag(float v) { maxSagVoltage = constrain(v, 0.0f, 2.0f); }

  // ค่าที่ใช้ไล่ปัญหา: ดูว่า compensation บวกไปเท่าไหร่จริงๆ
  float getCompensated() { return compensatedVoltage; }

  void update(int currentAvgPWM = 1000) {
    if (millis() - lastUpdate >= 10) {
      lastUpdate = millis();

      // [FIX] analogRead() -> analogReadMilliVolts()
      // ESP32 แต่ละตัวมี Vref จริงไม่เท่ากัน (3.0-3.6V ไม่ใช่ 3.3V เป๊ะ) และเส้นตอบสนอง ADC โค้ง
      // โรงงานวัดค่าจริงของชิปแต่ละตัวแล้วเบิร์นลง eFuse ไว้
      // analogReadMilliVolts() ดึงข้อมูลนั้นมาชดเชยให้ -> ได้ "มิลลิโวลต์จริง"
      // ถ้าใช้ analogRead() + สมมติ 3.3V จะอ่านต่ำกว่าจริง 10-14% (16.2V -> โชว์ 14.5V)
      //
      // MEDIAN-OF-5 ตัด spike จาก ESC/มอเตอร์ (median ทนกว่า average: spike เดียวไม่ดึงค่าเพี้ยน)
      uint32_t s[5];
      for (int i = 0; i < 5; i++) s[i] = analogReadMilliVolts(pin);
      for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 5; j++)
          if (s[j] < s[i]) { uint32_t t = s[i]; s[i] = s[j]; s[j] = t; }
      uint32_t mv = s[2];

      float ema_alpha = 0.20;
      emaMilliVolts = (emaMilliVolts * (1.0 - ema_alpha)) + (mv * ema_alpha);

      actualVoltage = (emaMilliVolts / 1000.0f) * vDividerRatio;

      // [FIX จาก main5] clamp upper bound: 4S LiPo ไม่เกิน 16.8V (4.2V x 4)
      // ป้องกัน ADC noise หรือสาย ADC หลุดทำให้คำนวณออกมาเกินจริง
      actualVoltage = constrain(actualVoltage, 0.0f, 17.5f);

      if (actualVoltage < 3.0) {
        actualVoltage = 0.0;
      }

      float throttlePct = mapFloat(currentAvgPWM, 1000, 2000, 0.0, 1.0);
      if (throttlePct < 0.0) throttlePct = 0.0;
      if (throttlePct > 1.0) throttlePct = 1.0;

      compensatedVoltage = actualVoltage + (throttlePct * maxSagVoltage);

      // [FIX จาก main5] เส้นโค้ง % ต่อเซลล์ (4.2/3.9/3.7/3.5/3.2 V) คูณ 4 เซลล์ = 16.8/15.6/14.8/14.0/12.8V
      float rawPercentage = 0.0;
      if (compensatedVoltage >= 16.8) {
        rawPercentage = 100.0;
      } else if (compensatedVoltage >= 15.6) {
        rawPercentage = mapFloat(compensatedVoltage, 15.6, 16.8, 70.0, 100.0);
      } else if (compensatedVoltage >= 14.8) {
        rawPercentage = mapFloat(compensatedVoltage, 14.8, 15.6, 50.0, 70.0);
      } else if (compensatedVoltage >= 14.0) {
        rawPercentage = mapFloat(compensatedVoltage, 14.0, 14.8, 20.0, 50.0);
      } else if (compensatedVoltage >= 12.8) {
        rawPercentage = mapFloat(compensatedVoltage, 12.8, 14.0, 0.0, 20.0);
      } else {
        rawPercentage = 0.0;
      }

      // [FIX] เดิม: ขึ้น = กระโดดทันที / ลง = ช้ามาก (0.95/0.05)
      //   -> พอ sag compensation ดัน % ขึ้นชั่วขณะ มันจะค้างที่ค่าสูงไม่ยอมลง
      //   แก้เป็น 2 กฎ:
      //     (ก) ฟิลเตอร์สมมาตร ขึ้น-ลงช้าเท่ากัน (ไม่มีทางกระโดด)
      //     (ข) ระหว่างใช้งาน % ห้ามเพิ่ม — แบตมีแต่ลดลง ถ้าเพิ่ม = การวัดเพี้ยน
      //         ยกเว้นแรงดันขึ้นเกิน 0.8V จากค่าต่ำสุดที่เคยเห็น = เปลี่ยน/ชาร์จแบต -> รีเซ็ต
      if (displayedPercentage < 0.0 || rawPercentage == 0.0) {
        displayedPercentage = rawPercentage;
        minSeenVoltage = actualVoltage;
      } else {
        // ตรวจการเปลี่ยน/ชาร์จแบต: แรงดันดีดขึ้นจากจุดต่ำสุดที่เคยเห็นมากผิดปกติ
        if (actualVoltage > minSeenVoltage + 0.8) {
          displayedPercentage = rawPercentage;   // เริ่มนับใหม่
          minSeenVoltage = actualVoltage;
        } else {
          if (actualVoltage < minSeenVoltage) minSeenVoltage = actualVoltage;
          float filtered = (displayedPercentage * 0.95) + (rawPercentage * 0.05);
          // กฎ (ข): ยอมให้ลดลงเท่านั้น
          displayedPercentage = (filtered < displayedPercentage) ? filtered : displayedPercentage;
        }
      }
    }
  }

  float getVoltage() {
    return actualVoltage;
  }

  int getPercentage() {
    return (int)(displayedPercentage + 0.5);
  }
};

#endif