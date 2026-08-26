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
const int THROTTLE_REV_US  = 1380;  // reverse leg of the three-point turn

const int STEER_CENTER_US = 1500;
const int STEER_LEFT_US   = 2390;
const int STEER_RIGHT_US  = 700;

const float BAR_WIDTH_FT          = 17.0f / 12.0f;
const float LANE_OVERLAP_FRACTION = 0.15f;

// N x M, matching the app's two inputs and set at runtime via '!D'.
//   N (fieldPassFt)  -- length of the first pass, straight ahead from the
//                       start corner. Lanes run along this axis (+Y).
//   M (fieldWidthFt) -- total width covered by stepping lanes to the right (+X).
float fieldPassFt  = 21.91f;
float fieldWidthFt = 21.91f;
const unsigned long VIO_TIMEOUT_MS = 1000;

int totalLanes  = 0;
int currentLane = 0;

// Steering polarity. +1 means a pulse BELOW centre steers toward increasing
// heading (clockwise / to the rover's right), matching STEER_RIGHT_US < centre.
// If Self Test shows the wheels going opposite to its printed labels, or the
// rover consistently corrects the wrong way, flip this to -1.
const int STEER_SIGN = 1;

const float CROSS_TRACK_GAIN = 90.0f;  // us of steering per ft off the lane
const float HEADING_GAIN     = 14.0f;  // us of steering per degree of error
const float MAX_STEER_OFFSET = 700.0f;

const float PASS_END_TOL_FT   = 0.4f;
const float ALIGN_TOL_FT      = 0.25f;
const float ALIGN_HEADING_TOL = 12.0f;
const unsigned long TURN_PHASE_TIMEOUT_MS = 6000;

float turnStartHeading = 0.0f;
unsigned long phaseStart = 0;

float robotX_ft    = 0.0f;
float robotY_ft    = 0.0f;
float robotHeading = 0.0f;
bool  vioActive    = false;
unsigned long lastVioTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long packetCount = 0;
bool dryRunMode  = false;
bool sprayActive = false;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
BLECharacteristic *txCharacteristic = NULL;
volatile bool bleConnected = false;

// A car cannot translate sideways, so shifting one lane (~1.2 ft) between
// passes needs a three-point turn rather than a waypoint beside the last one.
enum AutoState {
  AUTO_IDLE,
  AUTO_PASS,        // straight run along a lane; the only state that sprays
  AUTO_TURN_FWD,    // forward, full lock: first ~90 deg of the 180
  AUTO_TURN_REV,    // reverse, opposite lock: remaining ~90 deg
  AUTO_TURN_ALIGN,  // creep onto the new lane before spraying again
  AUTO_COMPLETE
};
AutoState state = AUTO_IDLE;

inline bool isNavigating() {
  return state == AUTO_PASS || state == AUTO_TURN_FWD ||
         state == AUTO_TURN_REV || state == AUTO_TURN_ALIGN;
}

// Even lanes run "up" (+Y, heading 0), odd lanes run back "down" (heading 180).
inline bool laneGoesUp(int lane)     { return (lane % 2) == 0; }
inline float laneTargetY(int lane)   { return laneGoesUp(lane) ? fieldPassFt : 0.0f; }
inline float laneDesiredHeading(int lane) { return laneGoesUp(lane) ? 0.0f : 180.0f; }

void setChannelPulse(uint8_t channel, int microseconds) {
  uint16_t ticks = (uint16_t)(((uint32_t)microseconds * 4096UL) / 20000UL);
  pwm.setPWM(channel, 0, ticks);
}

// In dry-run mode the spray state is still tracked and reported so the app
// can show where it *would* have sprayed, but no hardware is energised.
void setSpray(bool on) {
  if (sprayActive != on) {
    sprayActive = on;
    bleLog(on ? "[SPRAY] ON" : "[SPRAY] OFF");
  }

  if (dryRunMode) {
    digitalWrite(VALVE_PIN, LOW);
    digitalWrite(PUMP_PIN, LOW);
    return;
  }

  digitalWrite(VALVE_PIN, on ? HIGH : LOW);
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

void stopDrive() {
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, STEER_CENTER_US);
}

// Log to the app only. Serial is intentionally not used here: in the field
// there is no laptop attached, and everything below is mirrored in the app's
// rover panel. See setup() for the one boot-time exception.
void bleLog(String msg) {
  if (bleConnected && txCharacteristic != NULL) {
    msg += "\n";
    txCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    txCharacteristic->notify();
  }
}

void regenerateLanes() {
  totalLanes = laneCount(fieldWidthFt, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  currentLane = 0;
  float covered = laneCenterX(totalLanes - 1, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION) +
                  BAR_WIDTH_FT * 0.5f;
  bleLog("Area " + String(fieldPassFt, 1) + " x " + String(fieldWidthFt, 1) +
         " ft -> " + String(totalLanes) + " lanes, covering " +
         String(covered, 1) + " ft of width.");
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

// `rightward` > 0 steers toward increasing heading (rover's right).
void steerRightward(float rightward) {
  rightward = constrain(rightward, -MAX_STEER_OFFSET, MAX_STEER_OFFSET);
  int us = STEER_CENTER_US - (int)(STEER_SIGN * rightward);
  setChannelPulse(STEER_CH, constrain(us, STEER_RIGHT_US, STEER_LEFT_US));
}

// Hold the lane line: combine how far off the line we are with how far off
// the lane heading we are. Cross-track sign flips on return passes because
// +X lies to the rover's left when it is heading back down the field.
void holdLane(float laneX, float desiredHeading, bool goingUp) {
  float crossErr = laneX - robotX_ft;
  float crossTerm = (goingUp ? 1.0f : -1.0f) * CROSS_TRACK_GAIN * crossErr;
  float headingTerm = HEADING_GAIN * angleDiffDeg(desiredHeading, robotHeading);
  steerRightward(crossTerm + headingTerm);
}

bool insideField() {
  return robotY_ft >= 0.0f && robotY_ft <= fieldPassFt &&
         robotX_ft >= -0.5f && robotX_ft <= fieldWidthFt + 0.5f;
}

void beginPass(int lane) {
  currentLane = lane;
  phaseStart = millis();
  state = AUTO_PASS;
  bleLog(">>> Pass " + String(lane + 1) + "/" + String(totalLanes) + " at x=" +
         String(laneCenterX(lane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION), 2) + " ft");
}

void beginTurn() {
  setSpray(false);
  turnStartHeading = robotHeading;
  phaseStart = millis();
  state = AUTO_TURN_FWD;
  bleLog(">>> Turning to lane " + String(currentLane + 2) + "/" + String(totalLanes));
}

void runPass() {
  float laneX = laneCenterX(currentLane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  bool goingUp = laneGoesUp(currentLane);
  float targetY = laneTargetY(currentLane);

  holdLane(laneX, laneDesiredHeading(currentLane), goingUp);
  setChannelPulse(ESC_CH, THROTTLE_FWD_US);

  // Spray only on the straight run and only over the rectangle itself, so the
  // path may overrun the ends but the wetted area stays a clean rectangle.
  setSpray(insideField());

  bool reached = goingUp ? (robotY_ft >= targetY - PASS_END_TOL_FT)
                         : (robotY_ft <= targetY + PASS_END_TOL_FT);
  if (reached) {
    setSpray(false);
    if (currentLane + 1 >= totalLanes) {
      stopDrive();
      state = AUTO_COMPLETE;
      bleLog("=== MISSION COMPLETE: " + String(totalLanes) + " lanes ===");
    } else {
      beginTurn();
    }
  }
}

// The 180 is split so a car with a turning circle far wider than one lane can
// still line up on the next lane: swing out forward, then back up around.
void runTurn() {
  // Even lanes head up (+Y) with the next lane to the right; odd lanes head
  // down, putting the next lane to their left.
  bool turnRight = laneGoesUp(currentLane);
  float turned = fabsf(angleDiffDeg(robotHeading, turnStartHeading));
  bool timedOut = (millis() - phaseStart > TURN_PHASE_TIMEOUT_MS);

  if (state == AUTO_TURN_FWD) {
    steerRightward(turnRight ? MAX_STEER_OFFSET : -MAX_STEER_OFFSET);
    setChannelPulse(ESC_CH, THROTTLE_TURN_US);
    if (turned >= 90.0f || timedOut) {
      stopDrive();
      phaseStart = millis();
      state = AUTO_TURN_REV;
    }
    return;
  }

  if (state == AUTO_TURN_REV) {
    // Reversing with opposite lock keeps rotating the same way round.
    steerRightward(turnRight ? -MAX_STEER_OFFSET : MAX_STEER_OFFSET);
    setChannelPulse(ESC_CH, THROTTLE_REV_US);
    if (turned >= 165.0f || timedOut) {
      stopDrive();
      phaseStart = millis();
      state = AUTO_TURN_ALIGN;
    }
    return;
  }

  // AUTO_TURN_ALIGN: creep forward onto the new lane before committing.
  int nextLane = currentLane + 1;
  float laneX = laneCenterX(nextLane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  float desired = laneDesiredHeading(nextLane);
  holdLane(laneX, desired, laneGoesUp(nextLane));
  setChannelPulse(ESC_CH, THROTTLE_TURN_US);

  bool onLine = fabsf(laneX - robotX_ft) < ALIGN_TOL_FT;
  bool onHeading = fabsf(angleDiffDeg(desired, robotHeading)) < ALIGN_HEADING_TOL;
  if ((onLine && onHeading) || timedOut) {
    if (timedOut) bleLog("!! Align timed out; starting pass anyway.");
    beginPass(nextLane);
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
    if (d[0] == '1') {
      // Refused while running: the app re-zeros its VIO origin when Start is
      // pressed, so accepting a restart mid-drive would leave the rover
      // navigating old waypoints in a newly-shifted coordinate frame.
      if (isNavigating()) {
        bleLog("!! Already running. Press Stop first.");
      } else {
        bleLog(">>> Mission started: " + String(totalLanes) + " lanes, " +
               String(fieldPassFt, 1) + " ft per pass.");
        beginPass(0);
      }
    } else if (d[0] == '2') {
      state = AUTO_IDLE;
      stopDrive();
      setSpray(false);
      bleLog(">>> Mission stopped.");
    } else if (d[0] == '3') {
      if (isNavigating()) {
        bleLog("!! Self test refused: stop the mission first.");
      } else {
        runSelfTest();
        stopDrive();
      }
    } else if (d[0] == '4') {
      if (isNavigating()) {
        bleLog("!! Mode change refused: stop the mission first.");
      } else {
        dryRunMode = !dryRunMode;
        setSpray(false);  // never leave hardware energised across a mode change
        bleLog(dryRunMode ? "[MODE] DRY" : "[MODE] WET");
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

    float n, m;  // N = pass length, M = width, in the app's input order
    if (parseAreaPacket(&acc[i], accLen - i, n, m)) {
      if (isNavigating()) {
        bleLog("!! Area change refused while navigating.");
      } else if (n > 0.5f && m > 0.5f && n < 500.0f && m < 500.0f) {
        fieldPassFt = n;
        fieldWidthFt = m;
        regenerateLanes();
      } else {
        bleLog("!! Rejected implausible area: " + String(n, 1) + " x " + String(m, 1));
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
    // Re-announce state so a freshly connected app isn't showing stale UI,
    // and because these were generated at boot with nobody listening.
    bleLog("Area " + String(fieldPassFt, 1) + " x " + String(fieldWidthFt, 1) +
           " ft -> " + String(totalLanes) + " lanes.");
    bleLog(dryRunMode ? "[MODE] DRY" : "[MODE] WET");
    bleLog(sprayActive ? "[SPRAY] ON" : "[SPRAY] OFF");
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

  regenerateLanes();

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

  // The only serial output that remains. Everything else goes to the app,
  // but if the board never reaches this line -- or the app cannot connect --
  // serial is the sole remaining way to tell a hung board from a BLE problem.
  Serial.println("VIO Waypoint Navigator ready. Logging to app from here.");
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
    String phase = "IDLE";
    if (state == AUTO_PASS)            phase = "RUN";
    else if (state == AUTO_TURN_FWD)   phase = "TURN1";
    else if (state == AUTO_TURN_REV)   phase = "TURN2";
    else if (state == AUTO_TURN_ALIGN) phase = "ALIGN";
    else if (state == AUTO_COMPLETE)   phase = "DONE";

    bleLog("[TLM] " + phase +
           " vio=" + String(vioActive ? "OK" : "NONE") +
           " pkts=" + String(packetCount) +
           " pos=(" + String(robotX_ft, 1) + "," + String(robotY_ft, 1) + ")" +
           " hdg=" + String(robotHeading, 0) +
           " lane=" + String(currentLane + 1) + "/" + String(totalLanes) +
           (dryRunMode ? " DRY" : " WET") +
           (sprayActive ? " spray=ON" : " spray=OFF"));
  }

  if (vioActive && (millis() - lastVioTime > VIO_TIMEOUT_MS)) {
    vioActive = false;
    stopDrive();
    if (isNavigating()) {
      bleLog("!!! VIO signal lost. Stopping. !!!");
      state = AUTO_IDLE;
    }
  }

  if (isNavigating()) {
    if (!vioActive) {
      stopDrive();
    } else if (state == AUTO_PASS) {
      runPass();
    } else {
      runTurn();
    }
  }
}
