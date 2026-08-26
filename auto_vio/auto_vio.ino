/*
 * SafeSpread - Phone-VIO Waypoint Navigator
 *
 * Position/heading come entirely from the SafeSpreadVIO iPhone app's ARKit
 * pose, sent as !P packets over BLE. No internal dead reckoning.
 *
 * The route for the whole mission is computed once when Start is pressed and
 * thereafter only followed. Nothing during the run re-decides where to go:
 * tracking error steers the rover back onto the plan instead of producing a
 * different plan, which is what made the earlier reactive version oscillate
 * between maneuvers.
 *
 * Controls (from the app):
 *   '1' -> Start    '2' -> Stop    '3' -> Self test    '4' -> Wet/Dry
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include "nav_math.h"
#include "route.h"
#include "steering.h"

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
const int THROTTLE_REV_US  = 1380;

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

// The rover's two turning circles, measured 2026-08-26 with the sketch in
// turn_radius/. They are not the same size -- the steering trim sits
// off-centre -- and averaging them would ask for left turns tighter than the
// rover can drive and right turns wider than it needs, so both are carried
// separately all the way through the planner.
//
// Re-measure after any steering linkage or trim change: flash
// turn_radius/turn_radius.ino, press Start, and copy the two numbers here.
float turnRadiusLeftFt  = 4.33f;
float turnRadiusRightFt = 2.92f;

// The pulse at which the wheels actually point straight ahead. It is NOT the
// midpoint of the servo's travel, and assuming it was is what made every
// outbound pass sit off to one side and every return pass off to the other --
// which shows up as pairs of overlapping lines with gaps between them, not as
// obvious drift.
//
// The measured circles say where the true centre is. Steering angle grows
// roughly with distance from it, and turning radius goes as the reciprocal of
// the angle, so
//     (LEFT_US - c) / (c - RIGHT_US) = rRight / rLeft
// With 4.33 ft left and 2.92 ft right that puts c near 1709us, not 1500: the
// wheels have been sitting about a fifth of full lock to the right whenever
// the code believed they were straight.
// The arithmetic lives in steering.h so it can be tested on a host against a
// simulated rover whose real centre is known.
float steerTrimUs = 0.0f;               // learned on top of the computed centre
const float STEER_TRIM_LIMIT_US = 250.0f;
const float STEER_TRIM_RATE     = 0.05f;
const float STEER_TRIM_NEAR_FT  = 0.4f; // only learn while actually on the line
int  lastSteerCommandUs = 1500;
unsigned long lastTrimUpdate = 0;

float steerCentreUs() {
  return steeringCentreUs(turnRadiusLeftFt, turnRadiusRightFt,
                          STEER_LEFT_US, STEER_RIGHT_US) + steerTrimUs;
}

// Steering polarity: +1 means a pulse BELOW centre steers toward increasing
// heading (clockwise / to the rover's right), matching STEER_RIGHT_US < centre.
// Measured rather than assumed -- wiring the servo backwards inverts every
// correction, which does not look like a fault, it looks like wandering.
int  steerSign = 1;
bool steerSignChecked = false;
float polarityScore = 0.0f;
float polarityLastHeading = 0.0f;
float polarityLastCommand = 0.0f;
const float POLARITY_DECIDE_AT = 60.0f;

const float MAX_STEER_OFFSET = 700.0f;

// Course over ground: the direction the rover is actually travelling, taken
// from successive positions rather than from which way the phone points. Any
// constant yaw error in how the phone is mounted cancels out of this, which it
// does not for the reported heading -- and a mounting error of a few degrees
// otherwise biases lane tracking in opposite directions on outbound and return
// passes, striping the coverage.
float prevSampleX = 0.0f, prevSampleY = 0.0f;
float courseDeg = 0.0f;
bool  courseValid = false;
const float COURSE_MIN_STEP_FT = 0.20f;
const float COURSE_SMOOTHING   = 0.35f;

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

enum AutoState { AUTO_IDLE, AUTO_FOLLOW, AUTO_COMPLETE };
AutoState state = AUTO_IDLE;

inline bool isNavigating() { return state == AUTO_FOLLOW; }

const int MAX_ROUTE_POINTS = 2600;
RoutePoint route[MAX_ROUTE_POINTS];
int routeCount = 0;
int routeIndex = 0;
int firstPassEnd = 0;      // route index where the all-important first pass ends

// Steer at a point this far ahead on the path. On a straight pass a long
// lookahead tracks smoothly; through a turn it must be shorter than the arc's
// radius or the rover simply cuts the corner and misses the maneuver.
const float LOOKAHEAD_FT      = 2.5f;
const float LOOKAHEAD_TURN_FT = 1.0f;
const float PURSUIT_GAIN      = 16.0f;   // us of steering per degree of error
const int   ROUTE_SEARCH_WINDOW = 80;
const float CUSP_TOL_FT       = 0.5f;

// An RC ESC will not change direction until it has seen neutral, so a
// three-point turn has to pause briefly at each cusp.
const unsigned long DIR_CHANGE_PAUSE_MS = 350;
bool escReverse = false;
unsigned long dirChangeAt = 0;

// If the rover cannot reach the point it is tracking, skip past it rather than
// grinding against it for the rest of the mission.
const unsigned long STALL_TIMEOUT_MS = 8000;
int lastRouteIndex = -1;
unsigned long routeIndexSince = 0;

// Spray while the plan says to, unless the rover is so far off the plan that
// spraying would put brine somewhere it does not belong. The old test -- being
// inside the rectangle -- cut spray off half a foot outside the first lane,
// which sits on x=0, so the most important pass of the mission was the one
// most likely to stop spraying.
const float SPRAY_OFFPLAN_FT       = 1.5f;
const float SPRAY_OFFPLAN_FIRST_FT = 3.0f;   // the first pass gets more rope
const float SPRAY_HYSTERESIS_FT    = 0.5f;
bool sprayInhibited = false;

void bleLog(String msg) {
  if (bleConnected && txCharacteristic != NULL) {
    msg += "\n";
    txCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    txCharacteristic->notify();
  }
}

void resetCourse() {
  courseValid = false;
  prevSampleX = robotX_ft;
  prevSampleY = robotY_ft;
}

void updateCourse() {
  float dx = robotX_ft - prevSampleX;
  float dy = robotY_ft - prevSampleY;
  if (sqrtf(dx * dx + dy * dy) < COURSE_MIN_STEP_FT) return;

  float sample = bearingToWaypointDeg(dx, dy);
  if (!courseValid) {
    courseDeg = sample;
    courseValid = true;
  } else {
    courseDeg = fmodf(courseDeg + COURSE_SMOOTHING * angleDiffDeg(sample, courseDeg) +
                      360.0f, 360.0f);
  }
  prevSampleX = robotX_ft;
  prevSampleY = robotY_ft;
}

// Correlate what we asked the steering to do against what the heading actually
// did. Only meaningful driving forward: in reverse the same lock swings the
// nose the other way, which would cancel the evidence out.
void observeSteeringPolarity() {
  if (steerSignChecked) return;

  float turned = angleDiffDeg(robotHeading, polarityLastHeading);
  polarityLastHeading = robotHeading;

  float cmd = polarityLastCommand;
  if (fabsf(cmd) < 150.0f || fabsf(turned) < 0.5f || fabsf(turned) > 30.0f) return;

  polarityScore += (cmd > 0.0f) == (turned > 0.0f) ? fabsf(turned) : -fabsf(turned);

  if (fabsf(polarityScore) >= POLARITY_DECIDE_AT) {
    steerSignChecked = true;
    if (polarityScore < 0.0f) {
      steerSign = -steerSign;
      bleLog("!! Steering was reversed; polarity corrected automatically.");
    } else {
      bleLog("[OK] Steering polarity confirmed.");
    }
  }
}

// PCA9685 registers. An I2C ACK only proves the chip is powered and
// addressable; it says nothing about whether it is still configured.
#define PCA_MODE1     0x00
#define PCA_PRESCALE  0xFE
#define PCA_SLEEP_BIT 0x10
#define PCA_EXPECTED_PRESCALE 121   // 25MHz / (4096 * 50Hz) - 1

int lastSteerUs = STEER_CENTER_US;
int lastEscUs   = NEUTRAL_US;
unsigned long lastPwmCheck = 0;
unsigned long pwmRecoveries = 0;

void setChannelPulse(uint8_t channel, int microseconds) {
  if (channel == STEER_CH) lastSteerUs = microseconds;
  if (channel == ESC_CH)   lastEscUs = microseconds;
  uint16_t ticks = (uint16_t)(((uint32_t)microseconds * 4096UL) / 20000UL);
  pwm.setPWM(channel, 0, ticks);
}

uint8_t readPcaRegister(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0xFF;
  if (Wire.requestFrom((int)PCA9685_ADDR, 1) != 1) return 0xFF;
  return Wire.read();
}

// A brownout -- typically the steering servo stalling and sagging the rail --
// resets the PCA9685 into its power-on state: asleep, prescaler unset, all
// outputs dead. The ESP32 rides through on its own decoupling and never
// notices, so the sketch keeps sending pulses to a chip that is ignoring them.
bool ensurePwmReady() {
  uint8_t mode1 = readPcaRegister(PCA_MODE1);
  uint8_t prescale = readPcaRegister(PCA_PRESCALE);

  if (mode1 == 0xFF && prescale == 0xFF) return false;

  bool asleep = (mode1 & PCA_SLEEP_BIT) != 0;
  bool wrongRate = abs((int)prescale - PCA_EXPECTED_PRESCALE) > 3;
  if (!asleep && !wrongRate) return true;

  pwmRecoveries++;
  bleLog("!! PWM chip lost its config (mode1=0x" + String(mode1, HEX) +
         " prescale=" + String(prescale) + "). Reinitialising.");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  setChannelPulse(STEER_CH, lastSteerUs);
  setChannelPulse(ESC_CH, lastEscUs);
  return true;
}

// In dry-run mode the spray state is still tracked and reported so the app can
// show where it *would* have sprayed, but no hardware is energised.
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
  setChannelPulse(STEER_CH, (int)steerCentreUs());
}

void announceArea() {
  int lanes = laneCount(fieldWidthFt, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  float covered = laneCenterX(lanes - 1, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION) +
                  BAR_WIDTH_FT * 0.5f;
  bleLog("Area " + String(fieldPassFt, 1) + " x " + String(fieldWidthFt, 1) +
         " ft -> " + String(lanes) + " lanes, covering " +
         String(covered, 1) + " ft of width.");
}

// Port of diagnostics.ino, callable at runtime so wiring can be checked
// without reflashing. Throttle stays at neutral throughout.
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

  uint8_t mode1 = readPcaRegister(PCA_MODE1);
  uint8_t prescale = readPcaRegister(PCA_PRESCALE);
  bool asleep = (mode1 & PCA_SLEEP_BIT) != 0;
  bool wrongRate = abs((int)prescale - PCA_EXPECTED_PRESCALE) > 3;

  if (asleep || wrongRate) {
    bleLog("[FAIL] Chip answers but is not configured:");
    if (asleep)    bleLog("       SLEEP set -- outputs are off (brownout reset?)");
    if (wrongRate) bleLog("       prescale=" + String(prescale) + ", expected ~" +
                          String(PCA_EXPECTED_PRESCALE));
    bleLog("       Reinitialising now.");
    ensurePwmReady();
  } else {
    bleLog("[PASS] Configured: awake, prescale=" + String(prescale) + " (50Hz)");
  }
  if (pwmRecoveries) {
    bleLog("[INFO] Recovered from " + String(pwmRecoveries) +
           " brownout(s) since boot -- check servo power.");
  }

  setChannelPulse(ESC_CH, NEUTRAL_US);

  bleLog("[INFO] Steering LEFT (" + String(STEER_LEFT_US) + "us)...");
  setChannelPulse(STEER_CH, STEER_LEFT_US);
  delay(1000);

  bleLog("[INFO] Steering RIGHT (" + String(STEER_RIGHT_US) + "us)...");
  setChannelPulse(STEER_CH, STEER_RIGHT_US);
  delay(1000);

  // Centre is where the wheels really point straight, computed from the two
  // measured circles -- not the middle of the servo's travel. Look along the
  // rover here: the front wheels should be dead straight. If they are not,
  // re-measure the turning radii, because everything else is built on them.
  bleLog("[INFO] Steering CENTER (" + String((int)steerCentreUs()) +
         "us, servo mid is " + String(STEER_CENTER_US) + ")...");
  setChannelPulse(STEER_CH, (int)steerCentreUs());
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

// `rightward` > 0 steers toward increasing heading (rover's right), with
// MAX_STEER_OFFSET meaning full lock. Each side is scaled by its own travel
// from the true centre, which is not the same on both sides, so an equal
// command gives an equal fraction of the available turn whichever way it goes.
void steerRightward(float rightward) {
  if (steerSign < 0) rightward = -rightward;

  float us = steerPulseUs(rightward, steerCentreUs(),
                          STEER_LEFT_US, STEER_RIGHT_US, MAX_STEER_OFFSET);

  lastSteerCommandUs = (int)us;
  setChannelPulse(STEER_CH, (int)us);
}

// On a straight pass the steering has to average out to whatever pulse really
// does point the wheels straight -- so the average of what we apply IS the
// true centre, and we can just learn it. This catches whatever the measured
// radii did not: a servo that has crept, tyre pull, a cambered lawn. Only
// learn while the rover is already close to its line, or the correction it is
// making to get back on would be mistaken for the bias.
void learnSteeringTrim(float offPlanFt) {
  if (offPlanFt > STEER_TRIM_NEAR_FT) return;
  if (millis() - lastTrimUpdate < 50) return;
  lastTrimUpdate = millis();

  steerTrimUs = updateSteeringTrim(steerTrimUs, (float)lastSteerCommandUs,
                                   steerCentreUs(), STEER_TRIM_RATE,
                                   STEER_TRIM_LIMIT_US);
}

void planRoute() {
  routeCount = buildRoute(fieldPassFt, fieldWidthFt, BAR_WIDTH_FT,
                          LANE_OVERLAP_FRACTION,
                          turnRadiusLeftFt, turnRadiusRightFt,
                          route, MAX_ROUTE_POINTS);
  routeIndex = 0;
  lastRouteIndex = -1;
  sprayInhibited = false;
  escReverse = false;
  dirChangeAt = millis();

  // Where the first pass ends, so it can be given more latitude before spray
  // is cut. It is the pass everything else is lined up against.
  firstPassEnd = routeCount;
  for (int i = 0; i < routeCount; i++) {
    if (!route[i].spray) { firstPassEnd = i; break; }
  }

  int lanes = laneCount(fieldWidthFt, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  int reversals = 0;
  float minY = 0.0f, maxY = 0.0f, minX = 0.0f, maxX = 0.0f;
  for (int i = 0; i < routeCount; i++) {
    if (i > 0 && route[i].reverse != route[i - 1].reverse) reversals++;
    if (route[i].x < minX) minX = route[i].x;
    if (route[i].x > maxX) maxX = route[i].x;
    if (route[i].y < minY) minY = route[i].y;
    if (route[i].y > maxY) maxY = route[i].y;
  }

  bleLog("Planned " + String(routeCount) + " pts, " + String(lanes) +
         " lanes, " + String(reversals) + " direction changes.");
  bleLog("Turn radii L=" + String(turnRadiusLeftFt, 2) + " R=" +
         String(turnRadiusRightFt, 2) + " ft, straight-ahead at " +
         String((int)steerCentreUs()) + "us.");
  bleLog("Needs clear ground " + String(maxY - fieldPassFt, 1) + " ft past the far end, " +
         String(-minY, 1) + " ft behind the start.");

  if (routeCount >= MAX_ROUTE_POINTS) {
    bleLog("!! Route hit the point limit; area is too large to plan fully.");
  }
}

void updateSpray() {
  const RoutePoint &p = route[routeIndex];
  if (!p.spray) {
    sprayInhibited = false;
    setSpray(false);
    return;
  }

  float dx = p.x - robotX_ft, dy = p.y - robotY_ft;
  float off = sqrtf(dx * dx + dy * dy);
  float limit = (routeIndex < firstPassEnd) ? SPRAY_OFFPLAN_FIRST_FT : SPRAY_OFFPLAN_FT;

  if (sprayInhibited) {
    if (off < limit - SPRAY_HYSTERESIS_FT) sprayInhibited = false;
  } else if (off > limit) {
    sprayInhibited = true;
    bleLog("!! " + String(off, 1) + " ft off plan -- spray paused.");
  }

  setSpray(!sprayInhibited);
}

// Pure pursuit: aim at a point a fixed distance ahead on the plan. Drift moves
// that point off to one side, which steers the rover back onto the line.
void runFollow() {
  if (routeCount < 2) {
    stopDrive();
    state = AUTO_COMPLETE;
    bleLog("!! No route planned.");
    return;
  }

  routeIndex = advanceRouteIndex(route, routeCount, routeIndex,
                                 robotX_ft, robotY_ft,
                                 ROUTE_SEARCH_WINDOW, CUSP_TOL_FT);

  // A point it cannot reach must not hold up the rest of the mission.
  if (routeIndex == lastRouteIndex) {
    if (millis() - routeIndexSince > STALL_TIMEOUT_MS && routeIndex + 1 < routeCount) {
      routeIndex++;
      routeIndexSince = millis();
      bleLog("!! Stuck at point " + String(lastRouteIndex) + "; skipping ahead.");
    }
  } else {
    lastRouteIndex = routeIndex;
    routeIndexSince = millis();
  }

  if (routeIndex >= routeCount - 2) {
    setSpray(false);
    stopDrive();
    state = AUTO_COMPLETE;
    bleLog("=== MISSION COMPLETE ===");
    return;
  }

  bool reversing = route[routeIndex].reverse;
  if (reversing != escReverse) {
    escReverse = reversing;
    dirChangeAt = millis();
  }

  int la = lookaheadWithinSegment(route, routeCount, routeIndex,
                                  robotX_ft, robotY_ft,
                                  route[routeIndex].turning ? LOOKAHEAD_TURN_FT
                                                            : LOOKAHEAD_FT);

  float want = bearingToWaypointDeg(route[la].x - robotX_ft,
                                    route[la].y - robotY_ft);

  // Backing up, the rover travels in the direction opposite its nose. Course
  // over ground is preferred going forward, where it cancels out any error in
  // how the phone is mounted; reverse legs are short enough that the reported
  // heading is good enough.
  float reference;
  if (reversing) {
    reference = fmodf(robotHeading + 180.0f, 360.0f);
  } else {
    reference = courseValid ? courseDeg : robotHeading;
  }

  float err = angleDiffDeg(want, reference);
  float command = err * PURSUIT_GAIN;

  // Steering acts on the direction of travel the opposite way in reverse:
  // right lock swings the nose left, so the same command turns the rover's
  // path the other way.
  steerRightward(reversing ? -command : command);

  if (!reversing) {
    observeSteeringPolarity();
    polarityLastCommand = command;

    // Learn the true steering centre on the straight passes, where "straight
    // ahead" is a fact we can check against rather than a guess.
    if (!route[routeIndex].turning) {
      float ox = route[routeIndex].x - robotX_ft;
      float oy = route[routeIndex].y - robotY_ft;
      learnSteeringTrim(sqrtf(ox * ox + oy * oy));
    }
  } else {
    polarityLastHeading = robotHeading;   // don't let a reverse leg pollute it
  }

  if (millis() - dirChangeAt < DIR_CHANGE_PAUSE_MS) {
    setChannelPulse(ESC_CH, NEUTRAL_US);   // let the ESC re-arm
  } else if (reversing) {
    setChannelPulse(ESC_CH, THROTTLE_REV_US);
  } else {
    setChannelPulse(ESC_CH, fabsf(err) > 45.0f ? THROTTLE_TURN_US : THROTTLE_FWD_US);
  }

  updateSpray();
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
      // navigating an old plan in a newly-shifted coordinate frame.
      if (isNavigating()) {
        bleLog("!! Already running. Press Stop first.");
      } else {
        planRoute();
        if (routeCount >= 2) {
          resetCourse();
          state = AUTO_FOLLOW;
          bleLog(">>> Mission started.");
        } else {
          bleLog("!! Could not plan a route; check the area settings.");
        }
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
        setSpray(false);
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

    float np, mp;  // N = pass length, M = width, in the app's input order
    if (parseAreaPacket(&acc[i], accLen - i, np, mp)) {
      if (isNavigating()) {
        bleLog("!! Area change refused while navigating.");
      } else if (np > 0.5f && mp > 0.5f && np < 500.0f && mp < 500.0f) {
        fieldPassFt = np;
        fieldWidthFt = mp;
        announceArea();
      } else {
        bleLog("!! Rejected implausible area: " + String(np, 1) + " x " + String(mp, 1));
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
      // Course over ground is only meaningful driving forward.
      if (state == AUTO_FOLLOW && !escReverse) updateCourse();
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
    announceArea();
    bleLog("Turn radii L=" + String(turnRadiusLeftFt, 2) + " R=" +
           String(turnRadiusRightFt, 2) + " ft.");
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

  announceArea();

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

  // The only serial output that remains. Everything else goes to the app, but
  // if the board never reaches this line -- or the app cannot connect --
  // serial is the sole remaining way to tell a hung board from a BLE problem.
  Serial.println("VIO Waypoint Navigator ready. Logging to app from here.");
}

void loop() {
  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  // Catch a browned-out PWM chip within half a second, whether driving or not.
  if (millis() - lastPwmCheck >= 500) {
    lastPwmCheck = millis();
    ensurePwmReady();
  }

  // 1Hz telemetry so a silent rover is diagnosable: distinguishes "no pose
  // packets arriving" from "packets fine, navigation misbehaving".
  if (millis() - lastTelemetryTime >= 1000) {
    lastTelemetryTime = millis();
    String phase = "IDLE";
    if (state == AUTO_FOLLOW)        phase = escReverse ? "REV" : "RUN";
    else if (state == AUTO_COMPLETE) phase = "DONE";

    bleLog("[TLM] " + phase +
           " vio=" + String(vioActive ? "OK" : "NONE") +
           " pkts=" + String(packetCount) +
           " pos=(" + String(robotX_ft, 1) + "," + String(robotY_ft, 1) + ")" +
           " hdg=" + String(robotHeading, 0) +
           " pt=" + String(routeIndex) + "/" + String(routeCount) +
           " ctr=" + String((int)steerCentreUs()) +
           (dryRunMode ? " DRY" : " WET") +
           (sprayActive ? " spray=ON" : " spray=OFF") +
           (pwmRecoveries ? " pwmfix=" + String(pwmRecoveries) : ""));
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
    } else {
      runFollow();
    }
  }
}
