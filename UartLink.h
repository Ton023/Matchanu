#ifndef UARTLINK_H
#define UARTLINK_H
#include <ArduinoJson.h>

class UartLink {
  private:
    char rxBuffer[256];
    uint16_t rxIndex = 0;

  public:
    void begin() {
      // [PERF] ต้องเรียกก่อน begin() — ค่า default TX buffer = 0
      //   = Serial2.print() บล็อกรอจนข้อมูลลง FIFO (128 byte) หมด
      //   telemetry ~230 byte -> loop ค้าง ~9ms ทุก 50ms -> PID tick หาย 1-2 รอบ
      //   มี buffer 512 แล้ว print() คืนทันที ฮาร์ดแวร์ทยอยส่งเอง
      Serial2.setTxBufferSize(512);
      Serial2.begin(115200, SERIAL_8N1, 23, 19); // RXD2=23, TXD2=19
    }

    bool receive(JsonDocument& doc) {
      while (Serial2.available()) {
        char c = Serial2.read();

        if (c == '\n') {
          rxBuffer[rxIndex] = '\0';
          rxIndex = 0;

          doc.clear();
          DeserializationError error = deserializeJson(doc, rxBuffer);
          return !error;

        } else {
          if (rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = c;
          } else {
            // [FIX] Buffer overflow: ล้าง index, buffer, และ flush hardware Serial FIFO
            // ถ้าไม่ flush hardware buffer bytes ที่ overflow จะค้างใน Serial FIFO
            // และถูกอ่านต่อในรอบหน้าปนกับข้อมูลใหม่ ทำให้ JSON parse error ต่อเนื่อง
            rxIndex = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
            while (Serial2.available()) Serial2.read(); // flush hardware buffer
          }
        }
      }
      return false;
    }

    void send(const JsonDocument& doc) {
      serializeJson(doc, Serial2);
      Serial2.println();
    }
};

extern UartLink rpiLink;
#endif