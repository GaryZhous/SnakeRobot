// #include <Arduino.h>
// #include <SPI.h>

// #include "Bitcraze_PMW3901.h"
// #include <Adafruit_BNO08x.h>

// // ---------- Pins (from your top-level code) ----------
// #define BNO08X_CS    15
// #define BNO08X_INT   22
// #define BNO08X_RESET 4

// #define FLOW_CS      5

// // SPI bus pins on ESP32 DOIT (your note)
// static const int PIN_SCK  = 18;
// static const int PIN_MISO = 19;
// static const int PIN_MOSI = 23;

// // ---------- Sensor objects ----------
// Bitcraze_PMW3901 flow(FLOW_CS);
// Adafruit_BNO08x  bno08x(BNO08X_RESET);
// sh2_SensorValue_t sensorValue;

// sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
// long reportIntervalUs = 5000;

// // ---------- Euler helper ----------
// struct euler_t { float yaw, pitch, roll; } ypr;

// static void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* out, bool degrees=false) {
//   float sqr = qr*qr, sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;

//   out->yaw   = atan2f(2.0f * (qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
//   out->pitch = asinf(-2.0f * (qi*qk - qj*qr) / (sqi + sqj + sqk + sqr));
//   out->roll  = atan2f(2.0f * (qj*qk + qi*qr), (-sqi - sqj + sqk + sqr));

//   if (degrees) {
//     out->yaw   *= 180.0f / PI;
//     out->pitch *= 180.0f / PI;
//     out->roll  *= 180.0f / PI;
//   }
// }

// // Position

// int posX_counts = 0;
// int posY_counts = 0;

// static void setReports(sh2_SensorId_t rt, long intervalUs) {
//   if (!bno08x.enableReport(rt, intervalUs)) {
//     Serial.println("BNO08x: enableReport failed");
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   delay(200);


//   // Init shared SPI bus (important on ESP32)
//   SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

//   // Make sure CS lines idle HIGH
//   pinMode(FLOW_CS, OUTPUT);
//   pinMode(BNO08X_CS, OUTPUT);
//   digitalWrite(FLOW_CS, HIGH);
//   digitalWrite(BNO08X_CS, HIGH);
//   delay(50);

//   // ---- Init PMW3901 ----
//   if (!flow.begin()) {
//     Serial.println("PMW3901 init FAILED");
//     while (1) delay(100);
//   }
//   Serial.println("PMW3901 init OK");

//   // ---- Init BNO085 over SPI ----
//   if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
//     Serial.println("BNO08x init FAILED");
//     while (1) delay(100);
//   }
//   Serial.println("BNO08x init OK");

//   setReports(reportType, reportIntervalUs);
// }

// void loop() {
//   // ---- Optical flow ----
//   int16_t dx = 0, dy = 0;


//   flow.readMotionCount(&dx, &dy);

//   posX_counts += dx;
//   posY_counts += dy;

//   // ---- IMU ----
//   float yaw = NAN, pitch = NAN, roll = NAN;

//   if (bno08x.wasReset()) {
//     setReports(reportType, reportIntervalUs);
//   }

//   if (bno08x.getSensorEvent(&sensorValue)) {
//     if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
//       quaternionToEuler(
//         sensorValue.un.arvrStabilizedRV.real,
//         sensorValue.un.arvrStabilizedRV.i,
//         sensorValue.un.arvrStabilizedRV.j,
//         sensorValue.un.arvrStabilizedRV.k,
//         &ypr,
//         true
//       );
//       yaw = ypr.yaw; pitch = ypr.pitch; roll = ypr.roll;
//     }
//   }

//   // Change from ticks to meters


//   // ---- Print ----
//   Serial.print("Flow dX: "); Serial.print(posX_counts/60);
//   Serial.print(" dY: "); Serial.print(posY_counts/60);
//   Serial.print(" | YPR: ");
//   Serial.print(yaw);   Serial.print(", ");
//   Serial.print(pitch); Serial.print(", ");
//   Serial.println(roll);

//   delay(100);
// }

// #include <Arduino.h>
// #include <SPI.h>

// #include "Bitcraze_PMW3901.h"
// #include <Adafruit_BNO08x.h>

// // ---------- Pins (from your top-level code) ----------
// #define BNO08X_CS    15
// #define BNO08X_INT   22
// #define BNO08X_RESET 4

// #define FLOW_CS      5

// // SPI bus pins on ESP32 DOIT (your note)
// static const int PIN_SCK  = 18;
// static const int PIN_MISO = 19;
// static const int PIN_MOSI = 23;

// // ---------- Sensor objects ----------
// Bitcraze_PMW3901 flow(FLOW_CS);
// Adafruit_BNO08x  bno08x(BNO08X_RESET);
// sh2_SensorValue_t sensorValue;

// sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
// long reportIntervalUs = 5000;

// // ---------- Euler helper ----------
// struct euler_t { float yaw, pitch, roll; } ypr;

// static void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* out, bool degrees=false) {
//   float sqr = qr*qr, sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;

//   out->yaw   = atan2f(2.0f * (qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
//   out->pitch = asinf(-2.0f * (qi*qk - qj*qr) / (sqi + sqj + sqk + sqr));
//   out->roll  = atan2f(2.0f * (qj*qk + qi*qr), (-sqi - sqj + sqk + sqr));

//   if (degrees) {
//     out->yaw   *= 180.0f / PI;
//     out->pitch *= 180.0f / PI;
//     out->roll  *= 180.0f / PI;
//   }
// }

// static void setReports(sh2_SensorId_t rt, long intervalUs) {
//   if (!bno08x.enableReport(rt, intervalUs)) {
//     Serial.println("BNO08x: enableReport failed");
//   }
// }

// // ---------- Angle wrapping ----------
// static float wrapDeg(float a) {
//   while (a > 180.0f) a -= 360.0f;
//   while (a < -180.0f) a += 360.0f;
//   return a;
// }

// // ---------- Position (world frame) ----------
// float xw_counts = 0.0f;
// float yw_counts = 0.0f;

// // yaw handling
// bool yawInitialized = false;
// float yaw0_deg = 0.0f;       // initial yaw
// float lastYawDeg = 0.0f;     // last valid yaw
// float theta_rad = 0.0f;      // relative yaw in radians

// void setup() {
//   Serial.begin(115200);
//   delay(200);

//   // Init shared SPI bus
//   SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

//   // Make sure CS lines idle HIGH
//   pinMode(FLOW_CS, OUTPUT);
//   pinMode(BNO08X_CS, OUTPUT);
//   digitalWrite(FLOW_CS, HIGH);
//   digitalWrite(BNO08X_CS, HIGH);
//   delay(50);

//   // ---- Init PMW3901 ----
//   if (!flow.begin()) {
//     Serial.println("PMW3901 init FAILED");
//     while (1) delay(100);
//   }
//   Serial.println("PMW3901 init OK");

//   // ---- Init BNO085 over SPI ----
//   if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
//     Serial.println("BNO08x init FAILED");
//     while (1) delay(100);
//   }
//   Serial.println("BNO08x init OK");

//   setReports(reportType, reportIntervalUs);

//   Serial.println("Starting... will capture initial yaw on first valid IMU sample.");
// }

// void loop() {
//   // ---- Read optical flow increment (body frame) ----
//   int16_t dx = 0, dy = 0;
//   flow.readMotionCount(&dx, &dy);

//   // ---- Read IMU yaw (degrees) ----
//   float yawDeg = NAN;

//   if (bno08x.wasReset()) {
//     setReports(reportType, reportIntervalUs);
//   }

//   if (bno08x.getSensorEvent(&sensorValue)) {
//     if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
//       quaternionToEuler(
//         sensorValue.un.arvrStabilizedRV.real,
//         sensorValue.un.arvrStabilizedRV.i,
//         sensorValue.un.arvrStabilizedRV.j,
//         sensorValue.un.arvrStabilizedRV.k,
//         &ypr,
//         true
//       );
//       yawDeg = ypr.yaw;
//     }
//   }

//   // Keep last valid yaw
//   if (!isnan(yawDeg)) {
//     lastYawDeg = yawDeg;

//     // Capture initial yaw at first valid sample
//     if (!yawInitialized) {
//       yaw0_deg = lastYawDeg;
//       yawInitialized = true;
//     }
//   }

//   // If we don't have yaw yet, don't rotate/integrate (keeps it simple & safe)
//   if (!yawInitialized) {
//     Serial.println("Waiting for initial yaw...");
//     delay(50);
//     return;
//   }

//   // Relative heading theta = current - initial (wrapped)
//   float theta_deg = wrapDeg(lastYawDeg - yaw0_deg);
//   theta_rad = theta_deg * (PI / 180.0f);

//   // ---- Rotation matrix from theta ----
//   // R = [ cos -sin
//   //       sin  cos ]
//   float c = cosf(theta_rad);
//   float s = sinf(theta_rad);

//   // Rotate increment into world frame
//   float dXw = c * (float)dx - s * (float)dy;
//   float dYw = s * (float)dx + c * (float)dy;

//   // Integrate world position (still in "counts")
//   xw_counts += dXw;
//   yw_counts += dYw;

//   // ---- Placeholder scale (replace after calibration) ----
//   const float meters_per_count = 1.0f / 60.0f; // your placeholder
//   float xw_m = xw_counts * meters_per_count;
//   float yw_m = yw_counts * meters_per_count;

//   // ---- Print ----
//   Serial.print("dx,dy: "); Serial.print(dx); Serial.print(","); Serial.print(dy);
//   Serial.print(" | yaw0: "); Serial.print(yaw0_deg, 2);
//   Serial.print(" | yaw: "); Serial.print(lastYawDeg, 2);
//   Serial.print(" | theta: "); Serial.print(theta_deg, 2);
//   Serial.print(" | Xw,Yw(m): ");
//   Serial.print(xw_m, 4); Serial.print(", ");
//   Serial.println(yw_m, 4);

//   delay(100);
// }

#include <Arduino.h>
#include <SPI.h>

#include "Bitcraze_PMW3901.h"
#include <Adafruit_BNO08x.h>

// ---------- Pins ----------
#define BNO08X_CS    15
#define BNO08X_INT   22
#define BNO08X_RESET 4

#define FLOW_CS      5

// SPI bus pins on ESP32 DOIT
static const int PIN_SCK  = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;

// ---------- Sensor objects ----------
Bitcraze_PMW3901 flow(FLOW_CS);
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ---------- Use the SAME report as your example ----------
static const sh2_SensorId_t reportType = SH2_GAME_ROTATION_VECTOR;

// ---------- Angle wrapping ----------
static float wrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ---------- Yaw from quaternion (GAME ROTATION VECTOR) ----------
static float yawFromGameQuatDeg(const sh2_SensorValue_t& sv) {
  // quaternion components from GAME rotation vector
  const float qr = sv.un.gameRotationVector.real;
  const float qi = sv.un.gameRotationVector.i;
  const float qj = sv.un.gameRotationVector.j;
  const float qk = sv.un.gameRotationVector.k;

  // Yaw (Z) using atan2 (degrees)
  // Same convention as your earlier helper:
  // yaw = atan2(2(qi qj + qk qr), (qi^2 - qj^2 - qk^2 + qr^2))
  const float sqr = qr*qr, sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;
  float yaw = atan2f(2.0f * (qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
  yaw *= 180.0f / PI;
  return yaw;
}

static void setReports() {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType)) {
    Serial.println("Could not enable game rotation vector");
  }
}

// ---------- Position (world frame) ----------
float xw_counts = 0.0f;
float yw_counts = 0.0f;

// yaw handling
bool yawInitialized = false;
float yaw0_deg = 0.0f;
float lastYawDeg = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Init shared SPI bus
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // CS lines idle HIGH
  pinMode(FLOW_CS, OUTPUT);
  pinMode(BNO08X_CS, OUTPUT);
  digitalWrite(FLOW_CS, HIGH);
  digitalWrite(BNO08X_CS, HIGH);
  delay(50);

  // ---- Init PMW3901 ----
  if (!flow.begin()) {
    Serial.println("PMW3901 init FAILED");
    while (1) delay(100);
  }
  Serial.println("PMW3901 init OK");

  // ---- Init BNO085 over SPI ----
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("BNO08x init FAILED");
    while (1) delay(100);
  }
  Serial.println("BNO08x init OK");

  setReports();

  Serial.println("Starting... will capture initial yaw on first valid IMU sample.");
}

void loop() {
  // ---- Read optical flow increment (body frame) ----
  int16_t dx = 0, dy = 0;
  flow.readMotionCount(&dx, &dy);

  // ---- Read IMU yaw (degrees) using GAME ROTATION VECTOR ----
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float yawDeg = yawFromGameQuatDeg(sensorValue);
      lastYawDeg = yawDeg;

      if (!yawInitialized) {
        yaw0_deg = lastYawDeg;
        yawInitialized = true;
      }
    }
  }

  if (!yawInitialized) {
    Serial.println("Waiting for initial yaw...");
    delay(50);
    return;
  }

  // ---- Relative heading theta ----
  float theta_deg = wrapDeg(lastYawDeg - yaw0_deg);
  float theta_rad = theta_deg * (PI / 180.0f);

  // ---- Rotation matrix from theta ----
  float c = cosf(theta_rad);
  float s = sinf(theta_rad);

  // Rotate increment into world frame and integrate
  float dXw = c * (float)dx - s * (float)dy;
  float dYw = s * (float)dx + c * (float)dy;

  xw_counts += dXw;
  yw_counts += dYw;

  // ---- Placeholder scale (replace after calibration) ----
  const float meters_per_count = 1.0f / 60.0f; // your placeholder
  float xw_m = xw_counts * meters_per_count;
  float yw_m = yw_counts * meters_per_count;

  // ---- Print ----
  Serial.print("dx,dy: "); Serial.print(dx); Serial.print(","); Serial.print(dy);
  Serial.print(" | yaw0: "); Serial.print(yaw0_deg, 2);
  Serial.print(" | yaw: "); Serial.print(lastYawDeg, 2);
  Serial.print(" | theta: "); Serial.print(theta_deg, 2);
  Serial.print(" | Xw,Yw(m): ");
  Serial.print(xw_m, 4); Serial.print(", ");
  Serial.println(yw_m, 4);

  delay(100);
}
