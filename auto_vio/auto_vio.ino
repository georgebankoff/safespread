/*
 * SafeSpread - Phone-VIO Waypoint Navigator
 *
 * Position/heading come entirely from the SafeSpreadVIO iPhone app's ARKit
 * pose, sent as !P packets over BLE. No internal dead reckoning.
 *
 * Controls:
 *   '1' -> Start Autonomous Mission
 *   '2' -> Emergency Stop / Reset
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include "nav_math.h"

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define PCA9685_ADDR 0x40
const uint8_t STEER_CH = 0;
const uint8_t ESC_CH   = 4;

const int VALVE_PIN = 5;
const int PUMP_PIN  = 6;

const int NEUTRAL_US       = 1500;
const int THROTTLE_FWD_US  = 1620;
const int THROTTLE_TURN_US = 1600;

const int STEER_CENTER_US = 1500;
const int STEER_LEFT_US   = 2390;
const int STEER_RIGHT_US  = 700;

const float BAR_WIDTH_FT          = 17.0f / 12.0f;
const float LANE_OVERLAP_FRACTION = 0.15f;

// Defaults (480 sqft square); overridable at runtime from the app via '!D'.
float fieldWidthFt  = 21.91f;
float fieldLengthFt = 21.91f;
const float WAYPOINT_TOLERANCE_FT = 0.5f;
const unsigned long VIO_TIMEOUT_MS = 1000;

// Sized with headroom above the ~39 waypoints the current field/bar/overlap
// constants produce (see buildWaypoints in nav_math.h).
const int MAX_WAYPOINTS = 48;
Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;
int currentWaypointIndex = 0;

float robotX_ft    = 0.0f;
float robotY_ft    = 0.0f;
float robotHeading = 0.0f;
bool  vioActive    = false;
unsigned long lastVioTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long packetCount = 0;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
BLECharacteristic *txCharacteristic = NULL;
volatile bool bleConnected = false;

enum AutoState { AUTO_IDLE, AUTO_NAVIGATING, AUTO_COMPLETE };
AutoState state = AUTO_IDLE;

void setChannelPulse(uint8_t channel, int microseconds) {
  uint16_t ticks = (uint16_t)(((uint32_t)microseconds * 4096UL) / 20000UL);
  pwm.setPWM(channel, 0, ticks);
}

void setSpray(bool on) {
  digitalWrite(VALVE_PIN, on ? HIGH : LOW);
  digitalWrite(PUMP_PIN, LOW);
}

void stopDrive() {
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, STEER_CENTER_US);
}

void bleLog(String msg) {
  Serial.println(msg);
  if (bleConnected && txCharacteristic != NULL) {
    msg += "\n";
    txCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    txCharacteristic->notify();
  }
}

void regenerateWaypoints() {
  waypointCount = buildWaypoints(fieldWidthFt, fieldLengthFt, BAR_WIDTH_FT,
                                  LANE_OVERLAP_FRACTION, waypoints, MAX_WAYPOINTS);
  currentWaypointIndex = 0;
  bleLog("Area " + String(fieldWidthFt, 1) + " x " + String(fieldLengthFt, 1) +
         " ft (" + String(fieldWidthFt * fieldLengthFt, 0) + " sqft) -> " +
         String(waypointCount) + " waypoints.");
  if (waypointCount >= MAX_WAYPOINTS) {
    bleLog("!! Area too large for waypoint buffer; coverage will be incomplete.");
  }
}

// Port of diagnostics.ino, callable at runtime so wiring can be checked
// without reflashing. Throttle stays at neutral throughout -- this only
// exercises steering, never drives the motor.
void runSelfTest() {
  bleLog("=== SELF TEST ===");

  Wire.beginTransmission(PCA9685_ADDR);
  byte error = Wire.endTransmission();

  if (error != 0) {
    if (error == 2) {
      bleLog("[FAIL] I2C NACK - PCA9685 not found at 0x40.");
      bleLog("       Check VCC/GND on the PCA9685 logic header.");
    } else if (error == 5) {
      bleLog("[FAIL] I2C timeout / bus lockup.");
      bleLog("       Check SDA/SCL wires for breaks or shorts.");
    } else {
      bleLog("[FAIL] I2C error code " + String(error));
    }
    bleLog("=== SELF TEST ABORTED ===");
    return;
  }

  bleLog("[PASS] I2C OK (PCA9685 @ 0x40)");

  setChannelPulse(ESC_CH, NEUTRAL_US);

  bleLog("[INFO] Steering LEFT (" + String(STEER_LEFT_US) + "us)...");
  setChannelPulse(STEER_CH, STEER_LEFT_US);
  delay(1000);

  bleLog("[INFO] Steering RIGHT (" + String(STEER_RIGHT_US) + "us)...");
  setChannelPulse(STEER_CH, STEER_RIGHT_US);
  delay(1000);

  bleLog("[INFO] Steering CENTER (" + String(STEER_CENTER_US) + "us)...");
  setChannelPulse(STEER_CH, STEER_CENTER_US);
  delay(500);

  bleLog("[INFO] Valve ON 1s...");
  setSpray(true);
  delay(1000);
  setSpray(false);
  bleLog("[INFO] Valve OFF.");

  bleLog("If I2C PASSed but the servo never moved:");
  bleLog("  -> 5V/V+ or GND screw terminal loose, or CH0 plug reversed.");
  bleLog("=== SELF TEST COMPLETE ===");
}

void navigateToWaypoint(const Waypoint &wp) {
  float dx = wp.x - robotX_ft;
  float dy = wp.y - robotY_ft;

  float targetHeading = bearingToWaypointDeg(dx, dy);
  float err = angleDiffDeg(targetHeading, robotHeading);

  int steer = STEER_CENTER_US + (int)(err * 25.0f);
  steer = constrain(steer, STEER_RIGHT_US, STEER_LEFT_US);
  setChannelPulse(STEER_CH, steer);

  if (fabsf(err) > 45.0f) {
    setChannelPulse(ESC_CH, THROTTLE_TURN_US);
  } else {
    setChannelPulse(ESC_CH, THROTTLE_FWD_US);
  }

  if (waypointReached(dx, dy, WAYPOINT_TOLERANCE_FT)) {
    currentWaypointIndex++;
    stopDrive();
    setSpray(false);

    if (currentWaypointIndex >= waypointCount) {
      state = AUTO_COMPLETE;
      bleLog("=== MISSION COMPLETE ===");
    } else {
      bleLog(">>> Reached waypoint " + String(currentWaypointIndex) + "/" + String(waypointCount) +
             ". Next: (" + String(waypoints[currentWaypointIndex].x, 1) + ", " +
             String(waypoints[currentWaypointIndex].y, 1) + ")");
      if (currentWaypointIndex % 2 == 0) {
        setSpray(true);
      }
    }
  }
}

#define QSLOTS 16
#define QBYTES 32
static uint8_t qData[QSLOTS][QBYTES];
static uint8_t qLen[QSLOTS];
static volatile uint8_t qHead = 0, qTail = 0;

void queueWrite(const uint8_t *d, size_t n) {
  uint8_t next = (qHead + 1) % QSLOTS;
  if (next == qTail) return;
  if (n > QBYTES) n = QBYTES;
  memcpy(qData[qHead], d, n);
  qLen[qHead] = n;
  qHead = next;
}

static uint8_t acc[64];
static size_t  accLen = 0;

void feed(const uint8_t *d, size_t n) {
  if (n == 1) {
    if (d[0] == '1' && state == AUTO_IDLE) {
      state = AUTO_NAVIGATING;
      currentWaypointIndex = 0;
      setSpray(true);
      bleLog(">>> Mission started. " + String(waypointCount) + " waypoints.");
    } else if (d[0] == '2') {
      state = AUTO_IDLE;
      stopDrive();
      setSpray(false);
      bleLog(">>> Mission stopped.");
    } else if (d[0] == '3') {
      if (state == AUTO_NAVIGATING) {
        bleLog("!! Self test refused: stop the mission first.");
      } else {
        runSelfTest();
        stopDrive();
      }
    }
    return;
  }

  for (size_t i = 0; i < n; i++) {
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }

  size_t i = 0;
  while (accLen - i >= 11) {
    if (acc[i] != '!') { i++; continue; }

    float w, l;
    if (parseAreaPacket(&acc[i], accLen - i, w, l)) {
      if (state == AUTO_NAVIGATING) {
        bleLog("!! Area change refused while navigating.");
      } else if (w > 0.5f && l > 0.5f && w < 500.0f && l < 500.0f) {
        fieldWidthFt = w;
        fieldLengthFt = l;
        regenerateWaypoints();
      } else {
        bleLog("!! Rejected implausible area: " + String(w, 1) + " x " + String(l, 1));
      }
      i += 11;
      continue;
    }

    if (accLen - i < 15) { i++; continue; }
    float x, y, heading;
    if (parsePosePacket(&acc[i], accLen - i, x, y, heading)) {
      robotX_ft = x;
      robotY_ft = y;
      robotHeading = heading;
      vioActive = true;
      lastVioTime = millis();
      packetCount++;
      i += 15;
    } else {
      i++;
    }
  }
  if (i > 0) { memmove(acc, acc + i, accLen - i); accLen -= i; }
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    uint8_t *d = c->getData();
    size_t   n = c->getLength();
    if (d && n) queueWrite(d, n);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    bleConnected = true;
    bleLog("VIO app connected.");
  }
  void onDisconnect(BLEServer *s) {
    bleConnected = false;
    vioActive = false;
    state = AUTO_IDLE;
    stopDrive();
    setSpray(false);
    s->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  setSpray(false);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(50);
  stopDrive();

  regenerateWaypoints();

  BLEDevice::init("SafeSpread");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *svc = server->createService(NUS_SERVICE_UUID);
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  txCharacteristic = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("VIO Waypoint Navigator ready.");
}

void loop() {
  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  // 1Hz telemetry so a silent rover is diagnosable: distinguishes "no pose
  // packets arriving" from "packets fine, navigation misbehaving".
  if (millis() - lastTelemetryTime >= 1000) {
    lastTelemetryTime = millis();
    bleLog("[TLM] " + String(state == AUTO_NAVIGATING ? "RUN" :
                             (state == AUTO_COMPLETE ? "DONE" : "IDLE")) +
           " vio=" + String(vioActive ? "OK" : "NONE") +
           " pkts=" + String(packetCount) +
           " pos=(" + String(robotX_ft, 1) + "," + String(robotY_ft, 1) + ")" +
           " hdg=" + String(robotHeading, 0) +
           " wp=" + String(currentWaypointIndex) + "/" + String(waypointCount));
  }

  if (vioActive && (millis() - lastVioTime > VIO_TIMEOUT_MS)) {
    vioActive = false;
    stopDrive();
    if (state == AUTO_NAVIGATING) {
      bleLog("!!! VIO signal lost. Stopping. !!!");
      state = AUTO_IDLE;
    }
  }

  if (state == AUTO_NAVIGATING) {
    if (vioActive) {
      navigateToWaypoint(waypoints[currentWaypointIndex]);
    } else {
      stopDrive();
    }
  }
}
