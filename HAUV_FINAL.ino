// =============================================================
//  Drone Flight Controller — Hybrid (Aerial + Boat)
//  [BOAT] โหมด SUBMARINE เปลี่ยนเป็น "เรือผิวน้ำ" แล้ว
//    - มอเตอร์บิน 4 ตัวดับหมด
//    - ใช้มอเตอร์เรือ 2 ตัว (BE4108 720KV) ผ่าน ESC กลับทางได้ ที่ GPIO16 (ซ้าย) / GPIO17 (ขวา)
//    - ถอด MS5837 (เซนเซอร์ความลึก) ออกแล้ว
//    - ชื่อโหมดใน protocol ยังเป็น "SUBMARINE" เหมือนเดิม -> RPi/GCS ไม่ต้องแก้
//  ยกของดีจาก main6 มาใส่: 200Hz PID timer / D-on-measurement
//  / D LPF / IMU freeze failsafe / flash save-load
//  แต่คงโครง main55: JSON joystick, alt-hold, unlock 888/999, BatterySensor class
//  [SAFETY] เพิ่ม: arm interlock / A+B ไม่ตัดมอเตอร์กลางอากาศ / ห้ามสลับโหมดตอนปลดล็อก
//           / motor slew reset / watchdog 500ms / boat reverse protection
// =============================================================
#include <ArduinoJson.h>
#include "UartLink.h"
#include "luna.h"
#include "BatterySensor.h"
#include "config.h"
// [FIX 15 ต.ค.] จุดหยุดจริงของ ESC เรือ (เทสบนโต๊ะ: นิ่งช่วง 1475-1485) — ต้อง define ก่อน include BoatESC.h
#define BOAT_NEUTRAL_US 1480
#include "BoatESC.h"
#include <esp_task_wdt.h>
#include <esp_system.h>   // esp_reset_reason()
// ==========================================
// 1. PIN CONFIGURATION & HARDWARE
// ==========================================
const int pinLF = 33;
const int pinLB = 4;
const int pinRB = 5;
const int pinRF = 25;
const int pinBat = 32;
// [BOAT] มอเตอร์เรือ — ESC กลับทางได้ (servo PWM 50Hz, BOAT_NEUTRAL_US = หยุด)
//   ⚠️ GPIO16/17 ใช้ได้กับโมดูล ESP32-WROOM (ESP32-S / NodeMCU-32S)
//      ถ้าเป็น ESP32-WROVER สองขานี้ต่อกับ PSRAM อยู่ ใช้ไม่ได้
const int pinBoatL = 16;
const int pinBoatR = 17;

UartLink rpiLink;
LunaSensor myLuna;
BatterySensor myBatt;
FlightControl fc;
BLDC_PWM motors(pinLF, pinLB, pinRB, pinRF);
BoatESC  boat(pinBoatL, pinBoatR, 6, 7);   // LEDC channel 6,7 (timer แยกจากมอเตอร์บิน)

// ==========================================
// [BOAT] ค่าปรับแต่งโหมดเรือ
// ==========================================
// การบังคับ (differential / tank steering):
//   สติ๊กซ้าย ขึ้น-ลง (p)  = เดินหน้า / ถอยหลัง
//   สติ๊กขวา ซ้าย-ขวา (rx) = เลี้ยวซ้าย / ขวา (มอเตอร์สองข้างหมุนต่างกัน)
//   ปล่อยสติ๊กทั้งคู่ = หยุด (BOAT_NEUTRAL_US = 1480)
#define BOAT_RANGE_US    250.0f   // แรงสูงสุด = จุดหยุด ± 250 (= 50% ของเต็ม) เริ่มจากเบาๆ ก่อน
                                  //   ไหวแล้วค่อยเพิ่ม: 350 = 70% | 500 = 100% (1000-2000)
#define BOAT_TURN_GAIN   0.7f     // ความไวการเลี้ยว (1.0 = เต็ม) ลดลงถ้าเลี้ยวไวเกินจนหมุนติ้ว
#define BOAT_DEADBAND    0.15f    // สติ๊กเบี่ยงน้อยกว่า 15% = ถือว่าปล่อย
// [FIX 15 ต.ค.] 4.0 -> 1.25: ESC เรือออกตัวไม่ได้ถ้าค่าขึ้นเร็ว (เทสบนโต๊ะ: กระโดดค่า = ไม่หมุนเลย)
//   ความเร็วไล่ที่เทสแล้วว่าออกตัวได้ = 250µs/วิ -> 1.25µs ต่อรอบ (200Hz)
//   0 -> เต็มแรง (250) ใช้ ~1 วิ | เดินหน้าเต็ม -> ถอยเต็ม ~2.2 วิ (รวมพักกลับทิศ)
//   อยากให้ไวขึ้นค่อยๆ เพิ่ม (2.0 = 400µs/วิ) แล้วเทสว่ายังออกตัวได้
#define BOAT_SLEW_US     1.25f
#define BOAT_SLEW_US_PER_S  250.0f
#define BOAT_SLEW_MAX_DT_S  0.10f
// [FIX 15 ต.ค.] ข้ามช่วงรอบต่ำที่มอเตอร์สะดุด: ดันสติ๊กปุ๊บ เป้าเริ่มที่ จุดหยุด ± MIN ทันที
//   (slew ยังไล่ขึ้นให้ช้าๆ อยู่ — ไม่กระโดด) 0 = ปิด
//   หาค่าด้วย BoatMotorTest คำสั่ง F: ค่าที่ "เริ่มหมุนเรียบ" ลบจุดหยุด
#define BOAT_MIN_FWD_US  0.0f
#define BOAT_MIN_REV_US  0.0f
// [FIX 15 ต.ค.] ESC ต้อง "ได้ไฟก่อน แล้วค่อยเห็นสัญญาณ" ถึงจะ arm
// [FIX 20 ต.ค.] ส่งสัญญาณให้ ESC เรือ "ตั้งแต่สลับเป็นโหมดเรือ" (ยังล็อกอยู่ก็ส่ง)
//      -> ESC บี๊บตอนเลือกโหมดเรือ -> กด A+B ปลดล็อกแล้วบังคับได้ทันที ไม่ต้องรอ
//      ตอนล็อก = ส่งจุดหยุด (1480) ค้างไว้ มอเตอร์นิ่ง
//      โหมดบิน -> หยุดส่งพัลส์ (ESC เรือเงียบ)
//   ถ้าปลดล็อกเร็วกว่า BOAT_ARM_WAIT_MS หลังสลับโหมด จะรอให้ครบก่อน (ESC ยัง arm ไม่เสร็จ)
//   (ไม่เกี่ยวกับ BatterySensor — แบตใช้แสดงผลอย่างเดียว)
#define BOAT_ARM_WAIT_MS  3000
uint32_t boatEnabledAt = 0;
#define BOAT_NEUTRAL_BAND 3       // ต้องเห็นสติ๊ก |p|,|rx| <= 3 (จาก 30) ก่อน มอเตอร์เรือถึงจะทำงาน
// ทิศการหมุน: ถ้าดันสติ๊กไปหน้าแล้วมอเตอร์ข้างไหนดันถอยหลัง ให้เปลี่ยนข้างนั้นเป็น -1.0f
//   (หรือสลับสายมอเตอร์ 2 เส้นจาก 3 เส้น ก็ได้ผลเหมือนกัน)
#define BOAT_DIR_L       (+1.0f)
#define BOAT_DIR_R       (+1.0f)

// ==========================================
// 2. SENSORS & FUSION
// ==========================================
// [SENSOR ISOLATION] แหล่งความจริงเดียวของค่าเซนเซอร์ที่ส่งออก
//   AERIAL   : lidar_dist = ค่าจริง
//   SUBMARINE: lidar_dist = 0 เสมอ (โหมดเรือไม่ใช้ LiDAR)
// เขียนค่าทุกลูปไม่ว่าอยู่โหมดไหน -> ไม่มีทางค้างค่าเก่าข้ามโหมด
float estimated_altitude = 0.0f;
float lidar_dist = 0.0f;
uint32_t loopPerSec = 0;   // [DIAG] รอบ loop/วินาที
uint32_t imuPerSec  = 0;   // [DIAG 21 ต.ค.] จำนวน packet จาก DMP ต่อวินาที (ส่งใน telemetry "imu_hz")

// ==========================================
// 3. SYSTEM STATE
// ==========================================
String currentStatus = "SYSTEM_LOCKED";
bool isSystemUnlocked = false;
enum Mode { AERIAL_MODE, SUBMARINE_MODE };
Mode currentMode = AERIAL_MODE;

int current_r = 0, current_p = 0, current_rx = 0, current_ry = 0;
int current_a = 0, current_b = 0;
float val_lt = 0.0, val_rt = 0.0;   // LT/RT = yaw ในโหมดบิน
bool isHoldingUnlock = false;
unsigned long holdUnlockStartTime = 0;
// [SAFETY] Arm interlock — หลังปลดล็อก มอเตอร์ยังไม่หมุนจนกว่าสติ๊กจะอยู่ในตำแหน่งปลอดภัย 1 ครั้ง
//   AERIAL : ต้องเห็นคันเร่ง "ลงสุด" (สติ๊กกลาง = 1500 = โดรนพุ่งขึ้นเอง)
//   BOAT   : ต้องเห็นสติ๊ก "อยู่กลาง" (กันปลดล็อกตอนดันสติ๊กค้าง แล้วเรือพุ่ง)
bool armInterlockOk = false;
// ต้องเป็นค่าสติ๊ก "จริง" ที่มาหลังปลดล็อกเท่านั้น
//   ระหว่างยังกด A+B ค้าง ข้อความ 888 ถูกเมิน -> current_p/rx เป็น 0 ที่เรารีเซ็ตไว้
//   ถ้าไม่เช็คตัวนี้ โหมดเรือจะนึกว่า "สติ๊กอยู่กลางแล้ว" ทั้งที่ผู้ใช้ยังไม่ได้ปล่อยปุ่ม
//   -> ปล่อยปุ่มพร้อมดันสติ๊กค้าง = เรือพุ่งทันที (เจอตอนทดสอบจำลอง)
bool stickFresh = false;
#define THROTTLE_LOW_P (-25)

// [SAFETY] ค่า PWM รอบก่อนของ motor slew — ต้องรีเซ็ตเป็น 1000 ทุกครั้งที่มอเตอร์หยุด
//   เดิมเป็น static ในฟังก์ชัน ไม่เคยรีเซ็ต -> สตาร์ทใหม่แล้วกระโดดไปค่าเก่าของเที่ยวบินก่อน
float prevLF = 1000, prevRF = 1000, prevLB = 1000, prevRB = 1000;
void resetMotorSlew() { prevLF = prevRF = prevLB = prevRB = 1000.0f; }

unsigned long lastRecvTime = 0;
const unsigned long TIMEOUT_MS = 2000;
int current_avg_pwm = 1000;
float smooth_throttle = 1000.0f;

// ==========================================
// 4. PID VARIABLES (AERIAL ONLY)
// ==========================================
float Roll = 0.0f, Pitch = 0.0f, Yaw = 0.0f;

// [NEW 20 ต.ค.] Level calibration (ปุ่ม "Calibrate Level" บน GCS)
//   วางโดรนบนพื้นราบ กดปุ่ม -> มุมที่อ่านได้ตอนนั้นถูกเก็บเป็น trim แล้วหักออก -> Pitch/Roll = 0
//   ใช้ "หักมุมในซอฟต์แวร์" ไม่ใช่ calibrate MPU ใหม่ (calibrate ใช้หลายวิ -> loop ค้าง -> watchdog รีบูต)
//   ไม่บันทึกลง flash: ทุกครั้งที่บูต ESP32 calibrate MPU ใหม่ (ถือท่าตอนบูต = ระดับ) -> trim เก่าใช้ไม่ได้
//   ผลกับโหมดบิน: PID ใช้มุมหลังหัก trim -> โดรนพยายามทรงตัวตามระดับใหม่ (ซึ่งคือจุดประสงค์)
float trimPitch = 0.0f, trimRoll = 0.0f;       // มุมที่หักออก (องศา)
float rawPitchF = 0.0f, rawRollF = 0.0f;       // มุมดิบผ่าน LPF (ใช้ตอนกดปุ่ม กันค่ากระตุก)
bool  rawFiltInit = false;
uint8_t  calResult = 0;   // 0=ยังไม่เคยกด 1=สำเร็จ 2=ปฏิเสธ:ปลดล็อกอยู่ 3=ปฏิเสธ:กำลังขยับ 4=ปฏิเสธ:เอียงเกิน 5=รีเซ็ตแล้ว 6=ปฏิเสธ:IMU ไม่พร้อม
                          // 7=กำลังคาลิเบรต (gyro) 8=gyro ไม่นิ่ง/คาลิเบรตไม่ผ่าน (คืนค่าเดิม) 9=หมดเวลา (คืนค่าเดิม)
uint16_t calSeq = 0;      // เพิ่มทุกครั้งที่กด -> GCS รู้ว่ามีผลใหม่
#define LEVEL_CAL_MAX_DEG   15.0f   // เอียงเกินนี้ไม่ยอม (ติดตั้งผิด/วางไม่ราบ)
#define LEVEL_CAL_MAX_RATE  5.0f    // หมุนเร็วเกินนี้ (deg/s) = ยังขยับอยู่ ไม่ยอม

// [NEW 20 ต.ค.] ตั้งศูนย์ gyro ใหม่ตอนกด Calibrate Level (ทำก่อนตั้งระดับ)
//   ทำแบบ "ไม่บล็อก loop": เก็บค่า gyro ดิบจาก DMP ทีละ packet (200Hz) ครบ 1 วิ -> หาค่าเฉลี่ย (= bias)
//   -> ปรับ register gyro offset ในชิป -> วัดซ้ำเพื่อยืนยัน (สูงสุด 4 รอบ ~4-5 วิ)
//   ถ้าวัดแล้วแย่ลง / โดรนขยับ / หมดเวลา -> คืน offset เดิมทั้งหมด (ไม่ทำให้แย่กว่าก่อนกด)
//   ระหว่างคาลิเบรต: ห้ามปลดล็อก (A+B ไม่นับ) + มอเตอร์ไม่หมุนเพราะล็อกอยู่
// [FIX 21 ต.ค.] เดิมนับรอบเป็น "200 packet" โดยสมมติว่า DMP ส่ง 200Hz
//   แต่ dmpInitialize() ตั้ง rate ของตัวเอง (บอร์ดจริงอาจได้ ~100Hz) -> 1 รอบกลายเป็น ~2 วิ
//   ครบ 4-5 รอบ = เกิน timeout 8 วิ -> ขึ้น "IMU ไม่พร้อม" ทั้งที่ IMU ปกติ
//   ตอนนี้: นับรอบเป็น "เวลา" 1 วิ (ไม่ขึ้นกับ rate) + timeout 15 วิ + แยกรหัส timeout (9)
#define GYRO_CAL_ROUND_MS   1000    // เก็บค่า 1 วิต่อรอบ
// [FIX 21 ต.ค. #2] บอร์ดจริงได้ < 40 packet ใน 1 วิ -> เดิมตัดจบ "too few IMU samples"
//   ตอนนี้: รอบยืดออกได้จนกว่าจะได้ค่าพอ (ไม่เกิน GYRO_CAL_ROUND_MAX_MS) ไม่ตัดจบทันที
#define GYRO_CAL_WANT_SAMPLES 50    // อยากได้ต่อรอบ (ครบ 1 วิ + ได้เท่านี้ = จบรอบ)
#define GYRO_CAL_MIN_SAMPLES  10    // ขั้นต่ำจริงๆ (ยืดรอบจนสุดแล้วยังไม่ถึง = IMU มีปัญหา)
#define GYRO_CAL_ROUND_MAX_MS 3000  // รอบยาวสุด
#define GYRO_CAL_SETTLE_MS  100     // ทิ้งค่าหลังเปลี่ยน offset (ค่าใน FIFO ยังเป็นค่าเก่า)
#define GYRO_CAL_OK_DPS     0.2f    // bias เหลือต่ำกว่านี้ (deg/s) = ผ่าน
#define GYRO_CAL_ACCEPT_DPS 0.5f    // ครบรอบแล้วยังไม่ถึง OK แต่ต่ำกว่านี้ = รับได้
#define GYRO_CAL_NOISE_LSB  60      // ค่าสวิงในรอบเกินนี้ (~3.7 deg/s) = โดรนขยับ
#define GYRO_CAL_MAX_ROUND  4
#define GYRO_CAL_TIMEOUT_MS 25000
#define GYRO_LSB_PER_DPS    16.4f   // ±2000dps (ค่าที่ DMP ตั้ง)
#define GYRO_OFS_PER_LSB    2.0f    // register offset หน่วย 1/32.8 deg/s = 2 เท่าของ LSB ±2000dps
bool     gyroCalBusy = false;
uint8_t  gcRound = 0;
uint16_t gcN = 0;
uint32_t gcRoundStartMs = 0;  // เริ่มเก็บรอบนี้เมื่อไหร่ (หลัง settle)
int32_t  gcSum[3];
int16_t  gcMin[3], gcMax[3];
int16_t  gcOrigOff[3];      // offset ก่อนกด (ไว้คืนค่า)
float    gcPrevBias = 0.0f; // bias สูงสุดของรอบก่อน (deg/s)
uint32_t gcStartMs = 0;
float setpoint_pitch_angle = 0.0f;
float setpoint_roll_angle  = 0.0f;
float setpoint_yaw_rate    = 0.0f;

float error_pitch_rate, integral_pitch_rate, pid_output_pitch;
float error_roll_rate,  integral_roll_rate,  pid_output_roll;
float error_yaw_rate,   integral_yaw_rate,   pid_output_yaw;
float prev_error_pitch = 0.0f, prev_error_roll = 0.0f, prev_error_yaw = 0.0f;

// [main6] Derivative-on-measurement — diff จาก gyro โดยตรง
// กัน "derivative kick" ตอนเปลี่ยน setpoint angle แบบ step (โยกสติ๊ก)
// ทำให้ tgt_rate กระโดด → error กระโดด → D-term ระเบิด ถ้าใช้ diff จาก error
float prev_gyro_pitch = 0.0f, prev_gyro_roll = 0.0f, prev_gyro_yaw = 0.0f;

// [main6] D-term ที่กรอง LPF แล้ว (แยกจาก raw) — กัน noise ของ gyro
// ไม่ให้ขยายเป็นมอเตอร์กระตุกความถี่สูง
// alpha 0.3 = ค่าใหม่ 30% + ค่าเก่า 70%
float d_filt_pitch = 0.0f, d_filt_roll = 0.0f, d_filt_yaw = 0.0f;
#define D_LPF_ALPHA 0.3f

// [main5] Angle deadband (deg) / yaw rate deadband (deg/s) — กัน hover twitch + integral windup
float angle_deadband     = 0.5f;
float yaw_rate_deadband  = 3.0f;

unsigned long lastTime = 0;
float dt;

// [main5] Altitude setpoint slew + bumpless alt-hold engage — กัน throttle กระโดด/กระชากตอนสั่ง hold
float target_alt = 0.0f;
float alt_slew_rate = 30.0f;          // cm/วินาที
float max_climb_rate_cms = 500.0f;    // อัตราขึ้น/ลงสูงสุดตอนสติ๊กเบี่ยงสุดคัน (cm/s)
float altHoldEngageThrottle = 1000.0f;
unsigned long altHoldEngageTime = 0;
#define ALT_HOLD_BLEND_MS 500          // ไล่ระดับเข้าหาค่า PID ภายใน 500ms
float d_alt_filtered = 0.0f;           // [main5] ย้ายจาก static ในฟังก์ชันมาเป็น global เพื่อให้ reset จาก hold ได้

#define MOTOR_MIN_ARMED   1020
// [main5] Motor slew limit — จำกัดการเปลี่ยน PWM ต่อรอบ (200Hz) กันสไปก์กระตุก
#define MOTOR_SLEW 35.0f

// [option B] ลบ PID_LIMIT_AERIAL define ทิ้ง — ใช้ fc.pid_output_limit จาก config.h
// (source of truth เดียว → ปรับผ่าน flash ได้)

// ==========================================
// 5. 200Hz PID TIMER (main6)
// ==========================================
hw_timer_t* pidTimer = NULL;
volatile bool pidFlag = false;
void IRAM_ATTR onPidTimer() { pidFlag = true; }

// ==========================================
// 6. FAILSAFE STATE MACHINE (main6)
// ==========================================
enum FailsafeState { FS_NORMAL, FS_SIGNAL_LOSS, FS_IMU_FAIL };
FailsafeState fsState = FS_NORMAL;
unsigned long lastDmpUpdate = 0;
#define IMU_FREEZE_MS 200   // DMP ควรมาทุก 5ms — เกิน 200ms = ค้างแน่นอน

// ==========================================
// Prototypes
// ==========================================
void updateMotors();
void stopMotors();
void driveBoat();
void updateBoatSignal();
bool boatArmed();
void update_angle();
void pid_loop_aerial();
void resetPID();
bool checkImuFreeze();
void gyroCalSample();

// ==========================================
// [SAFETY] WATCHDOG
// ==========================================
// ปัญหา: PWM มาจากวงจร LEDC (ฮาร์ดแวร์) ที่ส่ง pulse เองต่อเนื่องไม่ต้องรอ CPU
//   ถ้า loop() ค้าง (I2C รอไม่จบ / ไลบรารีวนไม่ออก / บั๊ก) failsafe ทุกตัวที่อยู่ใน loop() จะไม่ทำงาน
//   แต่ LEDC ยังส่งค่าเดิม -> มอเตอร์หมุนต่อไม่มีกำหนด กด A+X ก็ไม่หยุด
// แก้: Task Watchdog เฝ้า loop() — ต้อง esp_task_wdt_reset() ทุกรอบ
//   ไม่มีใคร reset เกิน WDT_TIMEOUT_MS -> panic -> ESP32 รีบูต
//   -> ขา GPIO หยุดส่ง PWM -> ESC ไม่เห็นสัญญาณ -> มอเตอร์หยุด -> บูตใหม่ในสถานะล็อก
// loop() ปกติใช้ < 10ms ต่อรอบ (ดู lps ใน telemetry) 500ms จึงเผื่อเยอะ ไม่ทริกมั่ว
#define WDT_TIMEOUT_MS 500
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;

void setupWatchdog() {
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms     = WDT_TIMEOUT_MS;
    cfg.idle_core_mask = 0;      // เฝ้าเฉพาะ loop() ของเรา ไม่เฝ้า idle task (โปรเจกต์นี้ไม่ใช้ WiFi)
    cfg.trigger_panic  = true;   // ครบเวลา = รีบูต (ถ้า false จะแค่พิมพ์เตือน ไม่ได้ช่วยอะไร)
    // ESP32 Arduino core 3.x เปิด TWDT ไว้ตั้งแต่บูต (5 วิ, ไม่รีบูต) -> reconfigure ทับ
    // ถ้ายังไม่ถูกเปิด (บาง config) -> init ใหม่
    esp_err_t err = esp_task_wdt_reconfigure(&cfg);
    if (err == ESP_ERR_INVALID_STATE) err = esp_task_wdt_init(&cfg);
    // NULL = task ปัจจุบัน (loopTask) — ถ้าถูกเพิ่มไว้แล้ว (เช่นเคยเรียก enableLoopWDT) ไม่ต้องเพิ่มซ้ำ
    //   (add ซ้ำจะคืน error ทั้งที่ watchdog ทำงานอยู่ ทำให้ log หลอกว่า FAILED)
    if (err == ESP_OK && esp_task_wdt_status(NULL) != ESP_OK) err = esp_task_wdt_add(NULL);
    if (err == ESP_OK) { Serial.print("Watchdog: ON ("); Serial.print(WDT_TIMEOUT_MS); Serial.println(" ms)"); }
    else               { Serial.print("Watchdog: FAILED err="); Serial.println((int)err); }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    // [SAFETY] บอกว่ารอบที่แล้วดับเพราะอะไร — ถ้าเป็น watchdog/panic แปลว่า loop() เคยค้างหรือโปรแกรมพัง
    bootResetReason = esp_reset_reason();
    if (bootResetReason == ESP_RST_TASK_WDT || bootResetReason == ESP_RST_INT_WDT ||
        bootResetReason == ESP_RST_WDT      || bootResetReason == ESP_RST_PANIC ||
        bootResetReason == ESP_RST_BROWNOUT) {
        Serial.print("!!! WARNING: previous run ended abnormally (reason=");
        Serial.print((int)bootResetReason);
        Serial.println(") -> loop() hung / crashed / power dipped last time");
    }
    analogReadResolution(12);
    rpiLink.begin();
    myBatt.begin(pinBat);

    if (myLuna.begin(21, 22)) Serial.println("TF-Luna: Initialized");
    else Serial.println("TF-Luna: Failed!");

    delay(1000);
    motors.begin();
    // [BOAT] attach แต่ยังไม่ส่งพัลส์ — จะเริ่มส่งจุดหยุดตอนปลดล็อกโหมดเรือ (ดู updateBoatSignal)
    if (boat.begin()) Serial.println("Boat ESC: attached (GPIO16/17, 50Hz) - signal off until unlock");
    else              Serial.println("Boat ESC: LEDC attach FAILED!");
    delay(2000);

    Serial.println("Initializing Flight Control MPU...");
    fc.init();
    fc.loadGains();   // [main6] โหลดค่า PID ที่เคย save ไว้ (ถ้ามี)
    stopMotors();
    lastTime = micros();
    lastRecvTime = millis();
    lastDmpUpdate = millis();   // [main6] กัน IMU freeze false-trigger ตอนเพิ่งบูต

    // [main6] 200Hz PID timer (ESP32 core v3.x API)
    pidTimer = timerBegin(1000000);
    timerAttachInterrupt(pidTimer, &onPidTimer);
    timerAlarm(pidTimer, 5000, true, 0);   // 5000µs = 200Hz

    // [SAFETY] เปิด watchdog เป็น "ขั้นสุดท้าย" ของ setup เท่านั้น
    //   ก่อนหน้านี้มี delay() + calibrate IMU รวมหลายวินาที ถ้าเปิดก่อนจะรีบูตวนไม่จบ
    setupWatchdog();

    Serial.println("Hybrid Node Started! System Locked.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    esp_task_wdt_reset();   // [SAFETY] บอก watchdog ว่า loop() ยังวิ่งอยู่ (ต้องอยู่บรรทัดแรกเสมอ)

    // [DIAG] นับรอบ loop ต่อวินาที ส่งใน telemetry ("lps") — ต่ำผิดปกติ = มีอะไรบล็อก
    static uint32_t loopCount = 0, lastLpsMs = 0;
    loopCount++;
    if (millis() - lastLpsMs >= 1000) {
        loopPerSec = loopCount;
        loopCount = 0;
        lastLpsMs = millis();
    }

    // ---- (A) งานเร็ว: sensors + comm + state machine ----
    
    // เลือกอ่านเซนเซอร์ตามโหมดที่กำลังใช้งาน (ลดภาระบอร์ด)
    if (currentMode == AERIAL_MODE) {
        // [PERF] TF-Luna ออกข้อมูลจริง 100Hz — อ่านทุกรอบ loop (พันๆ ครั้ง/วิ) ได้ค่าซ้ำ
        // แต่กิน I2C ~0.5ms/ครั้ง = ครองเวลา loop เกือบหมด  จำกัด 50Hz ก็เกินพอ
        static uint32_t lastLuna = 0;
        if (millis() - lastLuna >= 20) {
            lastLuna = millis();
            myLuna.update();
        }
    }
    // [BOAT] โหมดเรือไม่อ่านเซนเซอร์ระยะใดๆ
    
    myBatt.update(current_avg_pwm);

    // --- 1. Sensor Fusion ---
    if (currentMode == AERIAL_MODE) {
        // [FIX 21 ต.ค.] ใช้ค่าที่กรองแล้วจาก luna.h (median + LPF + ทิ้งค่าสัญญาณอ่อน)
        lidar_dist = myLuna.getDistanceF();   // cm, 0 = ไม่มีค่าที่เชื่อถือได้
        if (lidar_dist > 2.0f && lidar_dist < 800.0f) {
            // [FIX 21 ต.ค.] ชดเชยการเอียง: LiDAR วัดตามแนวเฉียง -> ความสูงจริง = d · cos(pitch) · cos(roll)
            //   เอียง 10° ค่าเดิมเกินจริง ~1.5% / 20° ~6% -> alt hold ไต่ลงเองตอนเอียงไปหน้า
            //   จำกัด k >= 0.7 (~35°) — เอียงเกินนั้นค่า LiDAR ไม่น่าเชื่อถืออยู่แล้ว
            float k = cosf(Pitch * DEG_TO_RAD) * cosf(Roll * DEG_TO_RAD);
            if (k < 0.7f) k = 0.7f;
            estimated_altitude = lidar_dist * k;
        } else {
            estimated_altitude = 0.0f;
        }
    } else {
        lidar_dist = 0.0f;                // <- โหมดเรือ: lidar ต้องเป็น 0 เสมอ
        estimated_altitude = 0.0f;
    }

    // --- 2. อ่าน DMP เร็วสุด (non-blocking) ---
    // [main6] มาร์ค lastDmpUpdate ทุกครั้งที่ได้ packet → ใช้จับ IMU freeze
    static uint32_t imuCount = 0, lastImuMs = 0;   // [DIAG 21 ต.ค.] นับ packet IMU ต่อวินาที
    if (millis() - lastImuMs >= 1000) { imuPerSec = imuCount; imuCount = 0; lastImuMs = millis(); }
    if (fc.update()) {
        imuCount++;
        update_angle();
        lastDmpUpdate = millis();
        if (gyroCalBusy) gyroCalSample();   // [NEW 20 ต.ค.] ตั้งศูนย์ gyro (ทีละ packet ไม่บล็อก)
    }

    // --- 3. Receive Command (JSON) ---
    // [PERF] static = จองหน่วยความจำครั้งเดียว (เดิมจอง/คืน heap ทุกรอบ loop
    // หลายพันครั้ง/วินาที = heap แตกเป็นเสี่ยง -> อาการหน่วง/ค้างสะสม)
    static JsonDocument rxDoc;
    if (rpiLink.receive(rxDoc)) {
        String cmd = rxDoc["cmd"].as<String>();
        if (cmd == "J") {
            lastRecvTime = millis();
            int new_r = rxDoc["r"] | 0;
            int new_p = rxDoc["p"] | 0;
            // [SAFETY] กด A+B (888) ตอนปลดล็อกอยู่แล้ว = ไม่ต้องทำอะไร
            //   เดิม: 888 ระหว่างบิน -> stopMotors() = ร่วงกลางอากาศ
            //   และถ้าปล่อยให้ p=888 เข้า updateMotors() จะถูก constrain เป็น 30 = คันเร่งเต็ม
            //   ตอนนี้: ตอนปลดล็อกอยู่ ไม่รับค่า r/p ที่เป็นรหัส 888 (คงค่าสติ๊กเดิมไว้)
            bool isUnlockCode = (new_r == 888 && new_p == 888);
            if (!(isUnlockCode && isSystemUnlocked)) {
                current_r = new_r;
                current_p = new_p;
                stickFresh = true;   // ได้ค่าสติ๊กจริงแล้ว
            }
            current_rx = rxDoc["rx"] | 0;
            current_ry = rxDoc["ry"] | 0;
            current_a  = rxDoc["a"] | 0;
            current_b  = rxDoc["b"] | 0;

            // Force Lock
            bool isForceLock = (current_r == 999 && current_p == 999);
            if (isForceLock) {
                isSystemUnlocked = false;
                currentStatus = "SYSTEM_LOCKED";
                fc.altHoldActive = false;
                stopMotors();
                resetPID();
            }

            // [FIX] ข้อความล็อก 999 ไม่ใช่คำสั่งจากปุ่ม — ห้ามอ่านโหมด/ปุ่มจากมัน
            //   RPi (ตอนเน็ตหลุด) และ server (ตอนคนคุมหลุด/ปล่อยสิทธิ์) สร้าง 999 เองโดยใส่ btns = 0 ทั้งหมด
            //   เดิม: ล็อกก่อน แล้วอ่าน btns[8]=0 เป็น "ขอโหมดบิน" -> สลับเป็น AERIAL ได้เพราะเพิ่งล็อก
            //   -> เรือที่ลอยอยู่ในน้ำกลายเป็นโหมดบินเงียบๆ + กล้อง/AI สลับโหมดไปมา (เจอตอนทดสอบ end-to-end)
            JsonArray btns = rxDoc["btns"];
            if (!isForceLock && !btns.isNull() && btns.size() > 8) {
                val_lt = btns[6].as<float>();
                val_rt = btns[7].as<float>();

                // สลับโหมด
                float modeFlag = btns[8].as<float>();
                Mode requestedMode = (modeFlag > 0.5) ? SUBMARINE_MODE : AERIAL_MODE;
                // [SAFETY] สลับโหมดได้เฉพาะตอนล็อกอยู่เท่านั้น
                //   เดิม: กด SEL กลางอากาศ -> เปลี่ยนเป็น direct thrust ใต้น้ำ -> มอเตอร์ 1000 -> ร่วง
                if (requestedMode != currentMode && !isSystemUnlocked) {
                    resetPID();
                    currentMode = requestedMode;
                    setpoint_pitch_angle = 0.0f;
                    setpoint_roll_angle  = 0.0f;
                    // [FIX] เคลียร์ค่าเซนเซอร์ทั้งคู่ทันทีที่สลับโหมด
                    // กันเฟรมสุดท้ายของโหมดเก่าหลุดไปโชว์บน GCS ก่อนค่าใหม่มา
                    lidar_dist = 0.0f;
                    estimated_altitude = 0.0f;
                    fc.altHoldActive = false;   // alt hold ใช้ได้เฉพาะโหมดบิน
                    stopMotors();               // [BOAT] ทั้งมอเตอร์บิน (1000) และเรือ (จุดหยุด)
                }

                // สลับ Alt Hold (เฉพาะ Aerial)
                static bool last_hold_btn = false;
                bool current_hold_btn = (btns[3].as<float>() > 0.5);
                if (current_hold_btn && !last_hold_btn && currentMode == AERIAL_MODE) {
                    if (isSystemUnlocked && estimated_altitude > 5.0f) {
                        fc.altHoldActive = !fc.altHoldActive;
                        if (fc.altHoldActive) {
                            target_alt = estimated_altitude;
                            fc.setpoint_alt = estimated_altitude;
                            fc.integral_alt = 0.0f;
                            fc.prev_error_alt = 0.0f;
                            fc.alt_pid_output = 0.0f;
                            d_alt_filtered = 0.0f;
                            // [main5] bumpless transfer — จำ throttle ปัจจุบันไว้ตอนกด HOLD ON
                            altHoldEngageThrottle = fc.manual_throttle;
                            altHoldEngageTime = millis();
                        }
                    }
                }
                last_hold_btn = current_hold_btn;
            }
        } else if (cmd == "CAL") {
            // [NEW 20 ต.ค.] Level calibration — ทำได้เฉพาะตอนล็อก + นิ่ง + IMU ทำงาน
            String v = rxDoc["value"].as<String>();
            calSeq++;
            if (gyroCalBusy) {
                Serial.println("LEVEL CAL: busy - ignored");   // กำลังคาลิเบรตอยู่ (calResult ยังเป็น 7)
            } else if (isSystemUnlocked) {                       // ทั้ง set และ reset ต้องล็อกก่อน (กันมุมกระโดดตอนบิน)
                calResult = 2;
                Serial.println("LEVEL CAL: rejected - unlocked");
            } else if (v == "reset") {
                trimPitch = trimRoll = 0.0f;
                calResult = 5;
                resetPID();
                Serial.println("LEVEL CAL: reset (trim = 0)");
            } else if (millis() - lastDmpUpdate > IMU_FREEZE_MS || !rawFiltInit) {
                calResult = 6;
                Serial.println("LEVEL CAL: rejected - IMU not ready");
            } else if (fabsf(fc.gyroRate[1]) > LEVEL_CAL_MAX_RATE || fabsf(fc.gyroRate[2]) > LEVEL_CAL_MAX_RATE) {
                calResult = 3;
                Serial.println("LEVEL CAL: rejected - moving");
            } else if (fabsf(rawPitchF) > LEVEL_CAL_MAX_DEG || fabsf(rawRollF) > LEVEL_CAL_MAX_DEG) {
                calResult = 4;
                Serial.printf("LEVEL CAL: rejected - tilt too large (P %.1f R %.1f)\n", rawPitchF, rawRollF);
            } else {
                // ผ่านเงื่อนไข -> เริ่มตั้งศูนย์ gyro ก่อน (~1-5 วิ) เสร็จแล้วค่อยตั้งระดับใน gyroCalSample()
                fc.getGyroOffsets(gcOrigOff);
                gyroCalBusy = true;
                gcRound = 0; gcN = 0; gcPrevBias = 0.0f;
                gcStartMs = millis();
                gcRoundStartMs = gcStartMs;
                calResult = 7;
                Serial.printf("LEVEL CAL: gyro calibrating... (offset now %d %d %d) imu_hz %lu loop/s %lu - keep still\n",
                              gcOrigOff[0], gcOrigOff[1], gcOrigOff[2], (unsigned long)imuPerSec, (unsigned long)loopPerSec);
            }
        }
    }

    // ---- (B) งานความถี่คงที่ 200Hz: PID + motor mix + telemetry ----
    if (pidFlag) {
        pidFlag = false;
        updateBoatSignal();   // [FIX 15 ต.ค.] เปิด/ปิดสัญญาณ ESC เรือตามสถานะปลดล็อก+โหมด

        // --- 4.5 IMU Freeze Failsafe ---
        // [main6] อันตรายสุด — บินต่อด้วยมุมที่ค้าง = พลิกทันที
        if (checkImuFreeze()) {
            stopMotors();
            resetPID();
            lastTime = micros();   // กัน dt พุ่งตอนกลับมา
            // ยังส่ง telemetry ต่อได้ (ไม่ return ตอนนี้)
        } else {
            // --- 5. PID (aerial only) + Motor Output ---
            if (isSystemUnlocked) {
                if (currentMode == AERIAL_MODE) {
                    pid_loop_aerial();
                }

                // [SAFETY] Arm interlock — หลังปลดล็อก มอเตอร์ยังไม่หมุนจนกว่าสติ๊กอยู่ตำแหน่งปลอดภัย 1 ครั้ง
                if (!armInterlockOk && stickFresh) {
                    if (currentMode == AERIAL_MODE) {
                        if (current_p <= THROTTLE_LOW_P) armInterlockOk = true;          // คันเร่งลงสุด
                    } else {
                        if (abs(current_p)  <= BOAT_NEUTRAL_BAND &&
                            abs(current_rx) <= BOAT_NEUTRAL_BAND &&
                            boatArmed()) armInterlockOk = true; // สติ๊กอยู่กลาง + ESC เรือ arm เสร็จแล้ว
                    }
                }

                // 999 ถูกจัดการตอนรับคำสั่งแล้ว (ล็อกทันที) — 888 ตอนปลดล็อกอยู่ถูกเมินไปแล้ว
                if ((int)current_r == 999 || !armInterlockOk) {
                    stopMotors();
                    currentStatus = "SYSTEM_READY";   // ปลดล็อกแล้ว รอสติ๊กอยู่ตำแหน่งปลอดภัย
                } else {
                    currentStatus = (currentMode == AERIAL_MODE) ? "AERIAL_ACTIVE" : "SUBMARINE_ACTIVE";
                    updateMotors();
                }
            } else {
                // ระบบล็อกอยู่ — ตรวจการปลดล็อก 888 ค้าง 2 วินาที
                currentStatus = "SYSTEM_LOCKED";
                stopMotors();

                if ((int)current_r == 888 && (int)current_p == 888 && !gyroCalBusy) {   // [NEW 20 ต.ค.] ห้ามปลดล็อกระหว่างคาลิเบรต
                    if (!isHoldingUnlock) {
                        isHoldingUnlock = true;
                        holdUnlockStartTime = millis();
                    } else if (millis() - holdUnlockStartTime >= 2000) {
                        isSystemUnlocked = true;
                        currentStatus = "SYSTEM_READY";
                        isHoldingUnlock = false;
                        smooth_throttle = 1000.0f;
                        resetPID();
                        // [SAFETY] ล้างค่า 888 ที่ค้างอยู่ทิ้ง + บังคับให้ดึงคันเร่งลงสุดก่อนมอเตอร์จะหมุน
                        current_r = 0;
                        current_p = 0;
                        armInterlockOk = false;
                        stickFresh = false;   // รอค่าสติ๊กจริงข้อความถัดไป (ไม่ใช่ 888)
                    }
                } else {
                    isHoldingUnlock = false;
                }
            }
        }

        // --- 6. Telemetry (50ms) — วิ่งใน pidFlag block เพื่อ timing สม่ำเสมอ ---
        static unsigned long lastSend = 0;
        if (millis() - lastSend > 50) {
            lastSend = millis();
            static JsonDocument txDoc;
            txDoc.clear();   // ใช้ซ้ำ ไม่จองใหม่
            txDoc["status"]  = currentStatus;
            txDoc["pitch"]   = Pitch;
            txDoc["roll"]    = Roll;
            txDoc["batt"]    = myBatt.getVoltage();        // แรงดันจริงที่วัดได้ (ยังไม่ชดเชย sag)
            txDoc["bat_pct"] = myBatt.getPercentage();
            // ไว้ไล่ปัญหา sag: ถ้า b_cmp โดดขึ้นตอนมอเตอร์หมุน = maxSagVoltage สูงเกิน
            txDoc["b_cmp"]   = myBatt.getCompensated();     // หลังบวก sag compensation
            txDoc["pwm"]     = current_avg_pwm;             // AERIAL: PWM เฉลี่ย 4 ตัว | BOAT: เทียบเท่าจากแรงเรือ (ใช้คูณ sag)
            txDoc["fs"]      = (int)fsState; 

            txDoc["mode"]  = (currentMode == AERIAL_MODE) ? "AERIAL" : "SUBMARINE";
            txDoc["lidar"] = (int)lroundf(lidar_dist);   // AERIAL = ค่าที่กรองแล้ว (cm) | BOAT = 0
            txDoc["la"]    = (currentMode == AERIAL_MODE) ? myLuna.getAmp() : 0;   // [DIAG] ความแรงสัญญาณ LiDAR (<100 = ถูกทิ้ง)
            // [BOAT] PWM ที่ส่งเข้า ESC เรือจริง (1480 = หยุด) — ไว้ดูตอนทดสอบทิศ/แรง
            txDoc["bl"]    = (int)boat.getL();
            txDoc["br"]    = (int)boat.getR();
            txDoc["bon"]   = boat.isEnabled() ? 1 : 0;   // [FIX 15 ต.ค.] 1 = ส่งสัญญาณให้ ESC เรือแล้ว
            // [NEW 20 ต.ค.] Level calibration: trim ที่ใช้อยู่ + ผลการกดล่าสุด
            txDoc["tp"]    = trimPitch;
            txDoc["tr"]    = trimRoll;
            txDoc["cal"]   = calResult;
            txDoc["cal_n"] = calSeq;
            txDoc["lps"]   = loopPerSec;   // [DIAG] loop/วินาที
            txDoc["imu_hz"] = imuPerSec;   // [DIAG 21 ต.ค.] packet IMU/วินาที (ปกติ ~100)
            // [SAFETY] สาเหตุการรีบูตครั้งล่าสุด (esp_reset_reason): 1=เปิดเครื่องปกติ
            //   4=panic/crash  5=INT_WDT  6=TASK_WDT (loop ค้าง)  7=WDT อื่น  9=BROWNOUT (ไฟเลี้ยงตก)
            txDoc["rst"]   = (int)bootResetReason;
            txDoc["ah"]    = (currentMode == AERIAL_MODE) ? fc.altHoldActive : false;

            rpiLink.send(txDoc);
        }
    }
}

// ==========================================
// FAILSAFE STATE MACHINE (main5) — รวม IMU freeze + signal-loss ไว้ที่เดียว
// ลำดับความสำคัญ: IMU_FAIL > SIGNAL_LOSS > NORMAL
// return true = ต้องตัดมอเตอร์ทันที
// ==========================================
bool checkImuFreeze() {
    if (!isSystemUnlocked) {
        fsState = FS_NORMAL;
        fc.failsafeActive = false;
        return false;
    }

    unsigned long now = millis();
    bool imuFrozen  = (now - lastDmpUpdate > IMU_FREEZE_MS);
    bool signalLost = (now - lastRecvTime  > TIMEOUT_MS);

    if (imuFrozen) {
        if (fsState != FS_IMU_FAIL) {
            fsState = FS_IMU_FAIL;
            fc.failsafeActive = true;
            isSystemUnlocked = false;   // บังคับล็อกทันที
            fc.altHoldActive = false;
            currentStatus = "IMU_FROZEN";
            Serial.println("FAILSAFE: IMU FROZEN - motors cut!");
        }
        return true;
    }

    if (signalLost) {
        if (fsState != FS_SIGNAL_LOSS) {
            fsState = FS_SIGNAL_LOSS;
            fc.failsafeActive = true;
            isSystemUnlocked = false;   // บังคับล็อกทันที
            fc.altHoldActive = false;
            currentStatus = "SYSTEM_LOCKED";
            Serial.println("FAILSAFE: Signal Lost - motors cut!");
        }
        return true;
    }

    fsState = FS_NORMAL;
    fc.failsafeActive = false;
    return false;
}

// ==========================================
// MOTOR MIXING
// (คงจาก main55 — motor mix นี้ผ่านการ validate สัญญาณ yaw แล้ว)
// ==========================================
void updateMotors() {
    float outLF_val, outRF_val, outLB_val, outRB_val;

    if (currentMode == AERIAL_MODE) {
        // --- ✈️ AERIAL: Cascaded PID & Alt Hold ---
        // [FIX] เดิมไม่ constrain current_p ก่อน map()
        //   ถ้า Pi ส่งค่าเพี้ยน/นอกช่วง (เช่นรหัสปลดล็อก 888) -> throttle = 16300
        //   smooth_throttle จะไล่ขึ้นไปหาค่านั้น พอปลดล็อกปุ๊บมอเตอร์พุ่งทันที
        //   (โหมดใต้น้ำ constrain อยู่แล้ว โหมดบินลืม)
        float target_throttle = map(constrain(current_p, -30, 30), -30, 30, 1000, 2000);
        // [FIX 21 ต.ค.] ทิศ yaw กลับ: เดิม (rt - lt) -> กด LT หมุนขวา / RT หมุนซ้าย
        //   gyroRate[0] = gyro.z (แกน Z ชี้ขึ้น) -> ค่า + = หมุนทวนเข็ม (ซ้าย) มองจากด้านบน
        //   เดิม RT ให้ค่า + = สั่งหมุนซ้าย -> สลับเป็น (lt - rt): LT = ซ้าย, RT = ขวา
        //   แก้ที่ "คำสั่ง" อย่างเดียว ห้ามแก้เครื่องหมายใน motor mix / gyro
        //   (ลูปตอนนี้เสถียรอยู่แล้ว ถ้าไปกลับตรงนั้น = positive feedback -> โดรนหมุนติ้ว)
        setpoint_yaw_rate    = (val_lt - val_rt) * 90.0f;
        // [FIX] เดิม *1.0 = จอยสุด(±30) -> เอียงเป้าหมาย 30 องศา = เยอะเกินไป
        //   ดันสติ๊กปุ๊บโดรนก้ม 30° กระแทกพื้น -> คว่ำ
        //   ลดเป็น *0.25 = เอียงสุดแค่ 7.5° (ปลอดภัยสำหรับเริ่มต้น)
        //   ปรับเพิ่มได้ทีหลัง: 0.33=10° | 0.5=15°(sport)
        // ⚠️ ถ้าดันหน้าแล้วยังคว่ำ = "ทิศกลับ" ไม่ใช่ค่าเยอะ -> สลับเครื่องหมาย
        //   pitch: ลอง -current_ry เป็น +current_ry
        //   roll:  ลอง  current_rx เป็น -current_rx
        float ANGLE_SCALE = 0.25f;
        setpoint_pitch_angle = -current_ry * ANGLE_SCALE;
        setpoint_roll_angle  =  current_rx * ANGLE_SCALE;

        smooth_throttle = (smooth_throttle * 0.85f) + (target_throttle * 0.15f);
        fc.throttle = target_throttle;

        if (fc.altHoldActive) {
            // [main5] Climb-rate mode — สติ๊กเบี่ยงจากกลาง = อัตราขึ้น/ลง (cm/s) ต่อเนื่อง
            float dev = 0.0f;
            if (fc.throttle > 1520)      dev = fc.throttle - 1500.0f;
            else if (fc.throttle < 1480) dev = fc.throttle - 1500.0f;
            if (dev != 0.0f) {
                float climb_rate = (dev / 500.0f) * max_climb_rate_cms;
                target_alt += climb_rate * dt;
            }
            target_alt = constrain(target_alt, 5.0f, 800.0f);

            // [main5] ไล่ setpoint_alt เข้าหา target_alt แบบนุ่มๆ กัน error กระโดดวูบเดียว
            float alt_step = alt_slew_rate * dt;
            fc.setpoint_alt += constrain(target_alt - fc.setpoint_alt, -alt_step, alt_step);

            float alt_target_throttle = fc.hover_throttle + fc.alt_pid_output;
            unsigned long elapsed = millis() - altHoldEngageTime;
            if (elapsed < ALT_HOLD_BLEND_MS) {
                // [main5] bumpless transfer — ไล่จาก throttle ตอนกด HOLD ไปหาค่า PID ภายใน 500ms
                float blend = (float)elapsed / (float)ALT_HOLD_BLEND_MS;
                fc.manual_throttle = altHoldEngageThrottle + (alt_target_throttle - altHoldEngageThrottle) * blend;
            } else {
                fc.manual_throttle = alt_target_throttle;
            }
        } else {
            fc.manual_throttle = smooth_throttle;
        }

        if (fc.manual_throttle < 1050) {
            outLF_val = outRF_val = outRB_val = outLB_val = 1000;
            resetPID();
            resetMotorSlew();   // [SAFETY] รอบหน้าไต่จาก 1000 ไม่กระโดดไปค่าเก่า
        } else {
            float p_pitch = constrain(pid_output_pitch, -fc.pid_output_limit, fc.pid_output_limit);
            float p_roll  = constrain(pid_output_roll,  -fc.pid_output_limit, fc.pid_output_limit);
            float p_yaw   = constrain(pid_output_yaw,   -fc.pid_output_limit, fc.pid_output_limit);

            outLF_val = fc.manual_throttle + p_pitch + p_roll + p_yaw;
            outRF_val = fc.manual_throttle + p_pitch - p_roll - p_yaw;
            outLB_val = fc.manual_throttle - p_pitch + p_roll - p_yaw;
            outRB_val = fc.manual_throttle - p_pitch - p_roll + p_yaw;

            // [main5] clamp ก่อนแล้วจำกัดการเปลี่ยน PWM ต่อรอบ กันสไปก์/สั่นถี่ตอน hover
            outLF_val = constrain(outLF_val, MOTOR_MIN_ARMED, fc.maxThrottle);
            outRF_val = constrain(outRF_val, MOTOR_MIN_ARMED, fc.maxThrottle);
            outLB_val = constrain(outLB_val, MOTOR_MIN_ARMED, fc.maxThrottle);
            outRB_val = constrain(outRB_val, MOTOR_MIN_ARMED, fc.maxThrottle);

            // prevLF..prevRB ย้ายไปเป็น global แล้ว (รีเซ็ตได้จาก stopMotors)
            outLF_val = prevLF + constrain(outLF_val - prevLF, -MOTOR_SLEW, MOTOR_SLEW); prevLF = outLF_val;
            outRF_val = prevRF + constrain(outRF_val - prevRF, -MOTOR_SLEW, MOTOR_SLEW); prevRF = outRF_val;
            outLB_val = prevLB + constrain(outLB_val - prevLB, -MOTOR_SLEW, MOTOR_SLEW); prevLB = outLB_val;
            outRB_val = prevRB + constrain(outRB_val - prevRB, -MOTOR_SLEW, MOTOR_SLEW); prevRB = outRB_val;
        }

    } else {
        // --- 🚤 BOAT (SUBMARINE_MODE): มอเตอร์บินดับ ใช้มอเตอร์เรือ 2 ตัวแทน ---
        motors.writeAll(1000);
        resetMotorSlew();
        driveBoat();          // ตั้ง current_avg_pwm ให้เองข้างใน
        return;               // ไม่ผ่านส่วนเขียนมอเตอร์บินข้างล่าง
    }

    // เขียนลงมอเตอร์บิน (เฉพาะ AERIAL — โหมดเรือ return ไปก่อนแล้ว)
    //   clamp เฉพาะตัวที่ตั้งใจให้หมุน (>1000) ส่วนที่สั่งดับคง 1000
    if (outLF_val > 1000) outLF_val = constrain(outLF_val, MOTOR_MIN_ARMED, 2000);
    if (outRF_val > 1000) outRF_val = constrain(outRF_val, MOTOR_MIN_ARMED, 2000);
    if (outLB_val > 1000) outLB_val = constrain(outLB_val, MOTOR_MIN_ARMED, 2000);
    if (outRB_val > 1000) outRB_val = constrain(outRB_val, MOTOR_MIN_ARMED, 2000);

    current_avg_pwm = (outLF_val + outRF_val + outLB_val + outRB_val) / 4;

    motors.controlmotor(0, outLF_val);
    motors.controlmotor(1, outLB_val);
    motors.controlmotor(2, outRB_val);
    motors.controlmotor(3, outRF_val);
}

void stopMotors() {
    motors.writeAll(1000);    // มอเตอร์บิน: 1000 = ดับ
    boat.neutral();           // [BOAT] มอเตอร์เรือ: 1480 = หยุด (ห้ามใช้ 1000 = ถอยเต็มแรง)
    smooth_throttle = 1000.0f;
    current_avg_pwm = 1000;   // ให้ battery sag compensation รู้ว่ามอเตอร์หยุดแล้ว
    resetMotorSlew();
}

// ==========================================
// [FIX 15 ต.ค.] เปิด/ปิดสัญญาณ ESC เรือตามโหมด (ไม่เกี่ยวกับแบต)
//   ESC รุ่นนี้ arm ได้เฉพาะเมื่อ "ได้ไฟก่อน แล้วค่อยเห็นสัญญาณ"
// [FIX 20 ต.ค.] เดิมเปิดตอนปลดล็อก -> ต้องรอ ESC บี๊บอีก 3 วิหลังกด A+B
//   ตอนนี้เปิดทันทีที่สลับเป็นโหมดเรือ (สลับโหมดได้เฉพาะตอนล็อก -> แบตเสียบอยู่แล้ว ESC arm ได้)
//   -> ESC บี๊บตอนเลือกโหมด / ปลดล็อกแล้วบังคับได้เลย / สลับกลับโหมดบิน = หยุดส่ง
// ==========================================
void updateBoatSignal() {
    bool want = (currentMode == SUBMARINE_MODE);
    if (want && !boat.isEnabled()) {
        boat.enable();
        boatEnabledAt = millis();
        Serial.printf("Boat ESC: signal ON (boat mode, neutral %d) - arming %d ms\n", BOAT_NEUTRAL_US, BOAT_ARM_WAIT_MS);
    } else if (!want && boat.isEnabled()) {
        boat.disable();
        Serial.println("Boat ESC: signal OFF (aerial mode)");
    }
}

// ESC เรือได้สัญญาณนานพอให้ arm เสร็จแล้วหรือยัง
bool boatArmed() {
    return boat.isEnabled() && (millis() - boatEnabledAt >= BOAT_ARM_WAIT_MS);
}

// ==========================================
// [BOAT] Differential (tank) mixing
//   left  = เดินหน้า + เลี้ยว
//   right = เดินหน้า - เลี้ยว
//   ตัวอย่าง (BOAT_TURN_GAIN=1): สติ๊กหน้าสุด            -> L=+1, R=+1 (ตรงไป)
//                               สติ๊กขวาสุดอย่างเดียว      -> L=+1, R=-1 (หมุนตัวอยู่กับที่ ตามเข็ม)
//                               หน้าสุด + ขวาสุด          -> L=+2, R=0 -> normalize -> L=+1, R=0 (โค้งขวา)
// ==========================================
void driveBoat() {
    float thr  = constrain(current_p,  -30, 30) / 30.0f;   // สติ๊กซ้าย ขึ้น = +1 (เดินหน้า)
    float turn = constrain(current_rx, -30, 30) / 30.0f;   // สติ๊กขวา ขวา = +1 (เลี้ยวขวา)
    if (fabs(thr)  < BOAT_DEADBAND) thr  = 0.0f;
    if (fabs(turn) < BOAT_DEADBAND) turn = 0.0f;
    turn *= BOAT_TURN_GAIN;

    float l = thr + turn;
    float r = thr - turn;
    // normalize: ถ้าข้างใดเกิน 1 ย่อทั้งคู่ด้วยอัตราส่วนเดียวกัน (คงทิศการเลี้ยวไว้ ไม่ตัดทิ้ง)
    float m = fmaxf(fabsf(l), fabsf(r));
    if (m > 1.0f) { l /= m; r /= m; }

    // [FIX 15 ต.ค.] ฐาน = จุดหยุดจริง (1480) + ข้ามช่วงรอบต่ำ (ถ้าตั้ง MIN ไว้)
    //   หมายเหตุ: MIN ใช้ตาม "ทิศของสัญญาณ" (>จุดหยุด = เดินหน้าของ ESC) หลังคูณ BOAT_DIR แล้ว
    auto shape = [](float x) -> float {
        if (fabsf(x) < 0.02f) return 0.0f;
        float mn = (x > 0) ? BOAT_MIN_FWD_US : BOAT_MIN_REV_US;
        float s  = (x > 0) ? 1.0f : -1.0f;
        return s * (mn + fabsf(x) * (BOAT_RANGE_US - mn));
    };
    float targetL = (float)BOAT_NEUTRAL_US + shape(BOAT_DIR_L * l);
    float targetR = (float)BOAT_NEUTRAL_US + shape(BOAT_DIR_R * r);
    static uint32_t lastBoatUs = 0;
    uint32_t nowUs = micros();
    float dt = (lastBoatUs == 0) ? 0.005f : (nowUs - lastBoatUs) * 1e-6f;
    lastBoatUs = nowUs;
    if (dt > BOAT_SLEW_MAX_DT_S) dt = 0.005f;
    boat.drive(targetL, targetR, BOAT_SLEW_US_PER_S * dt);

    // battery sag compensation ใช้ current_avg_pwm (1000=หยุด ... 2000=เต็ม)
    // แปลงจาก "ระยะห่างจากจุดหยุด" ของมอเตอร์เรือ: เบี่ยง 0..500 -> 1000..2000
    float dev = (fabs(boat.getL() - BOAT_NEUTRAL_US) + fabs(boat.getR() - BOAT_NEUTRAL_US)) * 0.5f;
    current_avg_pwm = 1000 + (int)(dev * 2.0f);
}

// ==========================================
// MPU & PID FUNCTIONS
// ==========================================
float consToDegree(float mpu_u) { return (mpu_u * 180.0f / M_PI); }

void update_angle() {
    Yaw   = consToDegree(fc.ypr[0]);
    float rawPitch = consToDegree(fc.ypr[1]);
    float rawRoll  = consToDegree(fc.ypr[2]);
    // [NEW 20 ต.ค.] LPF มุมดิบไว้ใช้ตอนกด Calibrate Level (~0.25 วิ ที่ 200Hz)
    if (!rawFiltInit) { rawPitchF = rawPitch; rawRollF = rawRoll; rawFiltInit = true; }
    rawPitchF += 0.02f * (rawPitch - rawPitchF);
    rawRollF  += 0.02f * (rawRoll  - rawRollF);
    // หัก trim (ค่าเริ่มต้น 0 = เหมือนเดิมทุกอย่าง)
    Pitch = rawPitch - trimPitch;
    Roll  = rawRoll  - trimRoll;
}

// [NEW 20 ต.ค.] จบการคาลิเบรต: ok=true -> ตั้งระดับต่อ | ok=false -> คืน offset เดิม
static void gyroCalFinish(bool ok, uint8_t failCode, const char* why) {
    gyroCalBusy = false;
    calSeq++;
    if (!ok) {
        fc.setGyroOffsets(gcOrigOff);
        calResult = failCode;
        Serial.printf("LEVEL CAL: gyro FAILED (%s) - restored offset %d %d %d\n",
                      why, gcOrigOff[0], gcOrigOff[1], gcOrigOff[2]);
        return;
    }
    // gyro ผ่าน -> ตั้งระดับ (เหมือนเดิม)
    trimPitch = rawPitchF;
    trimRoll  = rawRollF;
    resetPID();
    calResult = 1;
    int16_t now[3]; fc.getGyroOffsets(now);
    Serial.printf("LEVEL CAL: OK gyro bias %.2f deg/s (offset %d %d %d) | trim P %.2f R %.2f\n",
                  gcPrevBias, now[0], now[1], now[2], trimPitch, trimRoll);
}

void gyroCalSample() {
    if (isSystemUnlocked)                         { gyroCalFinish(false, 2, "unlocked"); return; }
    uint32_t now = millis();
    if (now - gcStartMs > GYRO_CAL_TIMEOUT_MS) { gyroCalFinish(false, 9, "timeout"); return; }
    if ((int32_t)(now - gcRoundStartMs) < 0) return;   // ยังอยู่ช่วง settle หลังเปลี่ยน offset

    int16_t g[3]; fc.getRawGyro(g);
    if (gcN == 0) {
        for (int i = 0; i < 3; i++) { gcSum[i] = 0; gcMin[i] = g[i]; gcMax[i] = g[i]; }
    }
    for (int i = 0; i < 3; i++) {
        gcSum[i] += g[i];
        if (g[i] < gcMin[i]) gcMin[i] = g[i];
        if (g[i] > gcMax[i]) gcMax[i] = g[i];
    }
    gcN++;
    uint32_t el = now - gcRoundStartMs;
    if (el < GYRO_CAL_ROUND_MS) return;                                         // ยังไม่ครบ 1 วิ
    if (gcN < GYRO_CAL_WANT_SAMPLES && el < GYRO_CAL_ROUND_MAX_MS) return;     // ค่ายังน้อย -> เก็บต่อ
    if (gcN < GYRO_CAL_MIN_SAMPLES) {
        Serial.printf("LEVEL CAL: only %u IMU samples in %lu ms (imu_hz %lu)\n", gcN, (unsigned long)el, (unsigned long)imuPerSec);
        gyroCalFinish(false, 6, "too few IMU samples"); return;
    }

    // ---- ครบ 1 รอบ ----
    uint16_t n = gcN;
    gcN = 0;
    for (int i = 0; i < 3; i++) {
        if (gcMax[i] - gcMin[i] > GYRO_CAL_NOISE_LSB) { gyroCalFinish(false, 3, "moving"); return; }
    }
    float avg[3], worst = 0.0f;
    for (int i = 0; i < 3; i++) {
        avg[i] = (float)gcSum[i] / n;
        worst = fmaxf(worst, fabsf(avg[i]) / GYRO_LSB_PER_DPS);
    }
    Serial.printf("LEVEL CAL: gyro round %d (%u samples / %lu ms) bias %.3f deg/s  [x %.2f y %.2f z %.2f]\n",
                  gcRound, n, (unsigned long)el, worst, avg[0] / GYRO_LSB_PER_DPS, avg[1] / GYRO_LSB_PER_DPS, avg[2] / GYRO_LSB_PER_DPS);

    if (gcRound > 0 && worst > gcPrevBias + 0.05f) {   // ปรับแล้วแย่ลง
        gcPrevBias = worst;
        if (worst < GYRO_CAL_ACCEPT_DPS) gyroCalFinish(true, 0, "");          // แค่ noise แกว่ง ยังรับได้
        else                             gyroCalFinish(false, 8, "worse after adjust");
        return;
    }
    gcPrevBias = worst;
    if (worst < GYRO_CAL_OK_DPS)                  { gyroCalFinish(true, 0, ""); return; }
    if (gcRound >= GYRO_CAL_MAX_ROUND) {
        if (worst < GYRO_CAL_ACCEPT_DPS) gyroCalFinish(true, 0, "");
        else                             gyroCalFinish(false, 8, "bias still high");
        return;
    }
    // ปรับ offset: offset ใหม่ = offset เดิม − bias (แปลงหน่วย LSB -> หน่วย register)
    int16_t off[3]; fc.getGyroOffsets(off);
    for (int i = 0; i < 3; i++) {
        long n = (long)off[i] - lroundf(avg[i] * GYRO_OFS_PER_LSB);
        off[i] = (int16_t)constrain(n, -32768L, 32767L);
    }
    fc.setGyroOffsets(off);
    gcRound++;
    gcRoundStartMs = now + GYRO_CAL_SETTLE_MS;   // รอให้ค่าใหม่มาถึงก่อนเริ่มเก็บรอบถัดไป
}

void resetPID() {
    integral_pitch_rate = 0.0f;
    integral_roll_rate  = 0.0f;
    integral_yaw_rate   = 0.0f;
    prev_error_pitch    = 0.0f;
    prev_error_roll     = 0.0f;
    prev_error_yaw      = 0.0f;
    // [main6] reset D-on-measurement + filtered D
    prev_gyro_pitch = 0.0f; prev_gyro_roll = 0.0f; prev_gyro_yaw = 0.0f;
    d_filt_pitch = 0.0f;    d_filt_roll = 0.0f;    d_filt_yaw = 0.0f;

    fc.integral_alt     = 0.0f;
    fc.prev_error_alt   = 0.0f;
    fc.alt_pid_output   = 0.0f;
    d_alt_filtered      = 0.0f;   // [main5]
    fc.resetOuterIntegral();
    // [FIX] ล้าง output ด้วย ไม่งั้นรอบแรกหลังปลดล็อกจะใช้ค่า PID ค้างจากเที่ยวบินก่อน
    pid_output_pitch = 0.0f;
    pid_output_roll  = 0.0f;
    pid_output_yaw   = 0.0f;
}

// ==========================================
// Cascade PID — aerial (math เหมือน main6)
// เปลี่ยนจาก main55 เดิม:
//   - ใช้ fc.g.Kp_* (struct-nested) แทน fc.Kp_*
//   - เรียกจาก 200Hz timer แทนวิ่งใน loop() (timing สม่ำเสมอกว่า)
//   - D-term เปลี่ยนเป็น D-on-measurement + LPF alpha 0.3 (main6)
//     → กัน derivative kick + noise amplification
// ค่า clamp ใช้จาก config.h: outer_integral_limit / rate_integral_limit / outer_rate_limit / pid_output_limit
// ==========================================
void pid_loop_aerial() {
    unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    // [FIX] เดิม return ทิ้งโดยปล่อยให้ global dt ค้างค่าผิดปกติไว้
    //   updateMotors() ใช้ dt ต่อ (target_alt += climb_rate*dt / alt_step = slew*dt)
    //   -> dt=0.5 ค้างอยู่ = เป้าความสูงกระโดดทีเดียว 0.5 วินาที
    //   แก้: บังคับ dt ให้อยู่ในช่วงปกติก่อน return ทุกกรณี
    if (dt <= 0.000001f || dt > 0.1f) { dt = 0.005f; return; }

    // --- Pitch: Outer angle → Inner rate ---
    float error_angle_pitch = setpoint_pitch_angle - Pitch;
    if (fabs(error_angle_pitch) < angle_deadband) error_angle_pitch = 0.0f;   // [main5] กัน hover twitch
    float P_outer_pitch = fc.g.Kp_outer_pitch * error_angle_pitch;
    fc.integral_outer_pitch = constrain(fc.integral_outer_pitch + (error_angle_pitch * dt),
                                        -fc.outer_integral_limit, fc.outer_integral_limit);
    float target_rate_pitch = constrain(P_outer_pitch + (fc.g.Ki_outer_pitch * fc.integral_outer_pitch),
                                        -fc.outer_rate_limit, fc.outer_rate_limit);

    error_pitch_rate = target_rate_pitch - fc.gyroRate[1];
    integral_pitch_rate = constrain(integral_pitch_rate + (error_pitch_rate * dt),
                                    -fc.rate_integral_limit, fc.rate_integral_limit);

    // [main6] D-on-measurement + LPF (diff จาก gyro ตรง แทน diff จาก error)
    float d_raw_pitch = -(fc.gyroRate[1] - prev_gyro_pitch) / dt;
    prev_gyro_pitch = fc.gyroRate[1];
    d_filt_pitch = D_LPF_ALPHA * d_raw_pitch + (1.0f - D_LPF_ALPHA) * d_filt_pitch;

    pid_output_pitch = (fc.g.Kp_pitch * error_pitch_rate)
                     + (fc.g.Ki_pitch * integral_pitch_rate)
                     + (fc.g.Kd_pitch * d_filt_pitch);
    prev_error_pitch = error_pitch_rate;

    // --- Roll: Outer angle → Inner rate ---
    float error_angle_roll = setpoint_roll_angle - Roll;
    if (fabs(error_angle_roll) < angle_deadband) error_angle_roll = 0.0f;   // [main5] กัน hover twitch
    float P_outer_roll = fc.g.Kp_outer_roll * error_angle_roll;
    fc.integral_outer_roll = constrain(fc.integral_outer_roll + (error_angle_roll * dt),
                                       -fc.outer_integral_limit, fc.outer_integral_limit);
    float target_rate_roll = constrain(P_outer_roll + (fc.g.Ki_outer_roll * fc.integral_outer_roll),
                                       -fc.outer_rate_limit, fc.outer_rate_limit);

    error_roll_rate = target_rate_roll - fc.gyroRate[2];
    integral_roll_rate = constrain(integral_roll_rate + (error_roll_rate * dt),
                                   -fc.rate_integral_limit, fc.rate_integral_limit);

    // [main6] D-on-measurement + LPF
    float d_raw_roll = -(fc.gyroRate[2] - prev_gyro_roll) / dt;
    prev_gyro_roll = fc.gyroRate[2];
    d_filt_roll = D_LPF_ALPHA * d_raw_roll + (1.0f - D_LPF_ALPHA) * d_filt_roll;

    pid_output_roll = (fc.g.Kp_roll * error_roll_rate)
                    + (fc.g.Ki_roll * integral_roll_rate)
                    + (fc.g.Kd_roll * d_filt_roll);
    prev_error_roll = error_roll_rate;

    // --- Yaw: Rate-only ---
    error_yaw_rate = setpoint_yaw_rate - fc.gyroRate[0];
    // [main5] yaw rate deadband ก่อนเข้า integral — กัน integral windup จาก noise/vibration
    float yaw_rate_for_integral = (fabs(error_yaw_rate) < yaw_rate_deadband) ? 0.0f : error_yaw_rate;
    integral_yaw_rate = constrain(integral_yaw_rate + (yaw_rate_for_integral * dt),
                                  -fc.rate_integral_limit, fc.rate_integral_limit);

    // [main6] D-on-measurement + LPF
    float d_raw_yaw = -(fc.gyroRate[0] - prev_gyro_yaw) / dt;
    prev_gyro_yaw = fc.gyroRate[0];
    d_filt_yaw = D_LPF_ALPHA * d_raw_yaw + (1.0f - D_LPF_ALPHA) * d_filt_yaw;

    pid_output_yaw = (fc.g.Kp_yaw * error_yaw_rate)
                   + (fc.g.Ki_yaw * integral_yaw_rate)
                   + (fc.g.Kd_yaw * d_filt_yaw);
    prev_error_yaw = error_yaw_rate;
    pid_output_yaw = constrain(pid_output_yaw, -fc.pid_output_limit, fc.pid_output_limit);

    // --- Altitude PID ---
    if (fc.altHoldActive && estimated_altitude > 0.0f) {
        float error_alt = fc.setpoint_alt - estimated_altitude;
        if (fabs(error_alt) < 3.0f) error_alt = 0.0f;

        fc.integral_alt = constrain(fc.integral_alt + (error_alt * dt), -100.0f, 100.0f);
        float d_alt_raw = (error_alt - fc.prev_error_alt) / dt;
        d_alt_filtered = (0.2f * d_alt_raw) + (0.8f * d_alt_filtered);
        fc.prev_error_alt = error_alt;

        fc.alt_pid_output = (fc.g.Kp_alt * error_alt)
                          + (fc.g.Ki_alt * fc.integral_alt)
                          + (fc.g.Kd_alt * d_alt_filtered);
        fc.alt_pid_output = constrain(fc.alt_pid_output, -150.0f, 150.0f);
    } else {
        fc.alt_pid_output = 0.0f;
        fc.integral_alt   = 0.0f;
        d_alt_filtered    = 0.0f;
    }
}
