/*  This is the first project I've ever done 2022
&This code was developed in collaboration with trainer and engineer Murad At Orange, the programming training, design, and connections were all handled by me (Rama Alhiyari)
  Scorpion_AgriBot – Field Explorer & Soil/Plant Logger
  Platform : Arduino MEGA 2560
  Motors   : 4x DC motors via 2x L298N (or similar 4‑channel driver)
  Sensors  : HC‑SR04 ultrasonic (front), Analog soil moisture probe (on servo arm)
  Actuator : SG90/MG90S servo to lower/raise moisture probe
  Camera   : (OPTIONAL) ArduCAM Mini (OV2640) + microSD for leaf photos
  Storage  : (OPTIONAL) microSD card for logs and images

  Features:
  - Autonomous roaming with obstacle avoidance
  - Soil sampling routine: lower probe -> read -> raise
  - Plant imaging via camera (if enabled)
  - Data logging to CSV (time, position*, moisture, distance, notes)

  Compile‑time feature flags (uncomment as needed):
  - USE_NEW_PING: use NewPing lib for ultrasonic (else pulseIn)
  - ENABLE_CAMERA: use ArduCAM Mini OV2640 + SD to capture JPEGs
  - ENABLE_RTC: use DS3231 RTC for timestamps (else millis)

  *Position is odometry‑lite using time * speed estimate (no encoders). For real odometry,
   add wheel encoders and replace estimatePosition().
*/

// ======== FEATURE FLAGS ========
//#define USE_NEW_PING
//#define ENABLE_CAMERA
//#define ENABLE_RTC

// ======== LIBRARIES ========
#include <Servo.h>
#ifdef USE_NEW_PING
  #include <NewPing.h>
#endif
#ifdef ENABLE_CAMERA
  #include <SPI.h>
  #include <SD.h>
  #include <ArduCAM.h>
  // Select your camera module here:
  #define OV2640_MINI_2MP
  const int CAM_CS = 7;            // Chip Select for ArduCAM (adjust if wired differently)
  ArduCAM myCAM(OV2640, CAM_CS);
#endif
#ifdef ENABLE_RTC
  #include <Wire.h>
  #include <RTClib.h>
  RTC_DS3231 rtc;
#endif

// ======== PIN MAP (MEGA) ========
// L298N #1 – Left Side (Motors M1 front-left, M2 rear-left)
const uint8_t ENA = 5;      // PWM enable for M1 (uses Timer3/5 safe PWM)
const uint8_t IN1 = 22;     // M1 dir1
const uint8_t IN2 = 23;     // M1 dir2
const uint8_t ENB = 6;      // PWM enable for M2
const uint8_t IN3 = 24;     // M2 dir1
const uint8_t IN4 = 25;     // M2 dir2

// L298N #2 – Right Side (Motors M3 front-right, M4 rear-right)
const uint8_t ENC = 9;      // PWM enable for M3 (also servo default, but we use 10 for servo)
const uint8_t IN5 = 26;     // M3 dir1
const uint8_t IN6 = 27;     // M3 dir2
const uint8_t END_ = 10;    // PWM enable for M4
const uint8_t IN7 = 28;     // M4 dir1
const uint8_t IN8 = 29;     // M4 dir2

// Ultrasonic (HC‑SR04)
const uint8_t US_TRIG = 31;
const uint8_t US_ECHO = 30;

// Servo for moisture arm
const uint8_t SERVO_PIN = 11;   // use 11 to avoid PWM conflicts

// Soil moisture analog input
const uint8_t MOISTURE_PIN = A0;

// SD Card (for logs/images) – uses hardware SPI (50/51/52/53 on MEGA)
const uint8_t SD_CS = 53;   // SD Chip Select (adjust to your module)

// ======== ROBOT PARAMS ========
const int SERVO_UP = 20;          // degrees – adjust to your arm geometry
const int SERVO_DOWN = 110;       // degrees – adjust to poke soil
const int SERVO_SPEED_MS = 500;   // settle time after move

const uint8_t BASE_SPEED = 170;   // 0..255 PWM for cruising
const uint8_t TURN_SPEED = 160;   // turning PWM
const uint8_t SLOW_SPEED = 120;

const uint16_t OBSTACLE_CM = 25;  // stop/avoid threshold
const uint16_t CLEAR_CM    = 40;  // resume forward when farther than this

// Soil moisture thresholds (0‑1023 raw if analog). Calibrate in your soil.
int moistAir   = 800;  // reading in air (dry)
int moistWater = 350;  // reading in water (wet)
int dryThresh  = 65;   // %
int wetThresh  = 35;   // %

// ======== GLOBALS ========
Servo arm;
bool sdReady = false;

#ifdef USE_NEW_PING
  NewPing sonar(US_TRIG, US_ECHO, 300);
#endif

// ======== UTILITIES ========
void motorPinsInit() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT); pinMode(END_, OUTPUT);
}

void setMotorRaw(uint8_t inA, uint8_t inB, uint8_t en, int16_t pwm) {
  if (pwm > 0) {
    digitalWrite(inA, HIGH); digitalWrite(inB, LOW);
    analogWrite(en, constrain(pwm, 0, 255));
  } else if (pwm < 0) {
    digitalWrite(inA, LOW); digitalWrite(inB, HIGH);
    analogWrite(en, constrain(-pwm, 0, 255));
  } else {
    digitalWrite(inA, LOW); digitalWrite(inB, LOW);
    analogWrite(en, 0);
  }
}

void drive(int16_t leftPWM, int16_t rightPWM) {
  // Left side: M1, M2 share same direction -> average
  setMotorRaw(IN1, IN2, ENA, leftPWM);
  setMotorRaw(IN3, IN4, ENB, leftPWM);
  // Right side: M3, M4
  setMotorRaw(IN5, IN6, ENC, rightPWM);
  setMotorRaw(IN7, IN8, END_, rightPWM);
}

void stopAll() { drive(0,0); }

void forward(uint8_t spd=BASE_SPEED) { drive(spd, spd); }
void backward(uint8_t spd=SLOW_SPEED) { drive(-spd, -spd); }
void turnLeft(uint8_t spd=TURN_SPEED) { drive(-spd, spd); }
void turnRight(uint8_t spd=TURN_SPEED) { drive(spd, -spd); }

uint16_t readDistanceCM() {
#ifdef USE_NEW_PING
  return sonar.ping_cm();
#else
  digitalWrite(US_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  unsigned long d = pulseIn(US_ECHO, HIGH, 30000UL); // 30ms -> ~5m
  return (uint16_t)(d / 58UL);
#endif
}

int readMoistureRaw() {
  const int N=10; long sum=0;
  for (int i=0;i<N;i++){ sum += analogRead(MOISTURE_PIN); delay(5);} 
  return sum/N;
}

int rawToMoistPercent(int raw) {
  // Map raw to 0..100% moisture (100=wet). Clamp to range.
  int clamped = constrain(raw, moistWater, moistAir);
  float pct = 100.0f * (float)(moistAir - clamped) / (float)(moistAir - moistWater);
  return (int)constrain((int)pct, 0, 100);
}

String nowString() {
#ifdef ENABLE_RTC
  DateTime t = rtc.now();
  char buf[25];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
  return String(buf);
#else
  return String(millis());
#endif
}

#ifdef ENABLE_CAMERA
bool initCamera() {
  pinMode(CAM_CS, OUTPUT); digitalWrite(CAM_CS, HIGH);
  SPI.begin();
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_640x480); // VGA default
  delay(100);
  // Quick probe
  uint8_t vid, pid;
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(0x0A, &vid);
  myCAM.rdSensorReg8_8(0x0B, &pid);
  return (vid != 0x00 && pid != 0x00);
}

String captureJpegToSD() {
  // Create filename with time
  static uint32_t shot = 0;
  String name = String("IMG_") + String(shot++) + String(".JPG");
  File out = SD.open(name.c_str(), FILE_WRITE);
  if (!out) return String("");

  // Start capture
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  // Wait done
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) { /* spin */ }

  // Read FIFO in chunks
  uint8_t temp = 0, temp_last = 0;
  bool isHeader = false;
  uint32_t len = myCAM.read_fifo_length();
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while (len--) {
    temp_last = temp;
    temp = SPI.transfer(0x00);
    if (isHeader) out.write(temp);
    else if (temp == 0xD8 && temp_last == 0xFF) { // SOI
      isHeader = true; out.write(temp_last); out.write(temp);
    }
    if (temp == 0xD9 && temp_last == 0xFF) break; // EOI
  }
  myCAM.CS_HIGH();
  out.close();
  myCAM.clear_fifo_flag();
  return name;
}
#endif

// ======== LOGGING ========
void ensureCSVHeader() {
  if (!sdReady) return;
  if (!SD.exists("log.csv")) {
    File f = SD.open("log.csv", FILE_WRITE);
    if (f) {
      f.println("time,moisture_pct,moisture_raw,distance_cm,action,note,image");
      f.close();
    }
  }
}

void logRow(const String& action, const String& note, int moistPct, int moistRaw, int dist, const String& img) {
  if (!sdReady) return;
  File f = SD.open("log.csv", FILE_WRITE);
  if (f) {
    f.print(nowString()); f.print(',');
    f.print(moistPct); f.print(',');
    f.print(moistRaw); f.print(',');
    f.print(dist); f.print(',');
    f.print(action); f.print(',');
    f.print(note); f.print(',');
    f.println(img);
    f.close();
  }
}

// ======== SOIL SAMPLING ========
struct SampleResult { int raw; int pct; };

SampleResult takeSoilSample() {
  arm.write(SERVO_DOWN); delay(SERVO_SPEED_MS);
  int raw = readMoistureRaw();
  arm.write(SERVO_UP); delay(SERVO_SPEED_MS);
  SampleResult r; r.raw = raw; r.pct = rawToMoistPercent(raw);
  return r;
}

// ======== EXPLORATION ========
void avoidObstacle() {
  stopAll(); delay(150);
  // simple: back up, then random left/right turn
  backward(SLOW_SPEED); delay(400);
  stopAll(); delay(100);
  if (millis() & 0x01) { turnLeft(); } else { turnRight(); }
  delay(350);
  stopAll(); delay(100);
}

// ======== SETUP ========
void setup() {
  Serial.begin(115200);
  Serial.println("Scorpion_AgriBot booting...");

  pinMode(US_TRIG, OUTPUT); pinMode(US_ECHO, INPUT);
  motorPinsInit(); stopAll();

  arm.attach(SERVO_PIN);
  arm.write(SERVO_UP); delay(500);

#ifdef ENABLE_RTC
  if (!rtc.begin()) Serial.println("RTC not found – using millis timestamps");
#endif

#ifdef ENABLE_CAMERA
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed – images/logging disabled");
    sdReady = false;
  } else {
    sdReady = true; ensureCSVHeader();
  }
  if (sdReady) {
    if (initCamera()) Serial.println("Camera OK");
    else Serial.println("Camera init failed – check wiring and CS pin");
  }
#else
  if (SD.begin(SD_CS)) { sdReady = true; ensureCSVHeader(); }
#endif

  Serial.println("READY");
}

// ======== LOOP (FSM‑lite) ========
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_EVERY_MS = 20UL * 1000UL; // sample every 20s

void loop() {
  // 1) Navigate
  uint16_t d = readDistanceCM();
  if (d > 0 && d < OBSTACLE_CM) {
    avoidObstacle();
  } else if (d == 0 || d > CLEAR_CM) {
    forward(BASE_SPEED);
  } else {
    // in between – slow
    forward(SLOW_SPEED);
  }

  // 2) Periodic soil sample
  if (millis() - lastSampleMs > SAMPLE_EVERY_MS) {
    lastSampleMs = millis();
    stopAll(); delay(200);

    SampleResult s = takeSoilSample();
    String note;
    if (s.pct <= wetThresh) note = "WET";
    else if (s.pct >= dryThresh) note = "DRY";
    else note = "OK";

    String imgName = "";
#ifdef ENABLE_CAMERA
    if (sdReady) {
      imgName = captureJpegToSD();
      if (imgName.length()) Serial.print("Captured "), Serial.println(imgName);
    }
#endif

    logRow("SAMPLE", note, s.pct, s.raw, d, imgName);
    Serial.print("Moisture: "); Serial.print(s.pct); Serial.print("% (raw "); Serial.print(s.raw); Serial.println(")");
    Serial.print("Distance: "); Serial.print(d); Serial.println(" cm");

    // Basic response: blink behavior (or drive to next patch)
    if (note == "DRY") {
      // e.g., slow zig‑zag to keep searching for irrigation spots
      turnLeft(); delay(250); turnRight(); delay(250);
    }

    forward(BASE_SPEED);
  }
}

/* ================== WIRING QUICK GUIDE ==================
ULTRASONIC HC‑SR04:  TRIG->31, ECHO->30, VCC 5V, GND GND
SERVO (SG90):        SIG->11, VCC 5–6V (separate BEC recommended), GND common
SOIL MOISTURE:       AO->A0, VCC 5V, GND GND (or 3.3V depending on module)
L298N #1 (Left):     ENA->5 (PWM), IN1->22, IN2->23; ENB->6 (PWM), IN3->24, IN4->25
L298N #2 (Right):    ENC->9 (PWM), IN5->26, IN6->27; END->10 (PWM), IN7->28, IN8->29
SD Module:           CS->53, SCK->52, MOSI->51, MISO->50, VCC 5V/3.3V per board, GND
ArduCAM Mini OV2640: CS->7, SCK->52, MOSI->51, MISO->50, VCC 3.3V, GND; (use level‑safe wiring)
NOTE: Power motors from a separate battery through L298N with shared grounds.
========================================================== */

/* ================== CALIBRATION NOTES ==================
1) Moisture calibration: place probe in air -> note analogRead as moistAir; in water/mud -> moistWater.
   Update constants and thresholds. Expect module variation.
2) Servo angles: adjust SERVO_UP/DOWN to align with your arm geometry.
3) Speeds/thresholds: tune BASE_SPEED, OBSTACLE_CM, CLEAR_CM to your field.
4) SD & camera: if not using camera, keep ENABLE_CAMERA undefined; logging still works if SD present.
5) Encoders: for better navigation, add wheel encoders and implement PID + waypoint driving.
========================================================== */

