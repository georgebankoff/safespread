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

const int MAX_LANES = 64;
bool laneCovered[MAX_LANES];
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

// Course over ground: the direction the rover is actually travelling, taken
// from successive positions rather than from which way the phone points.
// Any constant yaw error in how the phone is mounted cancels out of this,
// which it does not for the reported heading -- and with the phone's heading
// a mounting error of a few degrees biases lane tracking in opposite
// directions on outbound and return passes, striping the coverage.
float prevSampleX = 0.0f, prevSampleY = 0.0f;
float courseDeg = 0.0f;
bool  courseValid = false;
const float COURSE_MIN_STEP_FT = 0.20f;  // ignore jitter below real motion
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

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
BLECharacteristic *txCharacteristic = NULL;
volatile bool bleConnected = false;

// A car cannot translate sideways, so shifting one lane (~1.2 ft) between
// passes needs a three-point turn rather than a waypoint beside the last one.
enum AutoState {
  AUTO_IDLE,
  AUTO_PASS,          // straight run along a lane; the only state that sprays
  AUTO_SHUFFLE_FWD,   // short forward leg at full lock
  AUTO_SHUFFLE_REV,   // short reverse leg at opposite lock
  AUTO_BACKOUT,       // reverse clear of the rectangle to make room to line up
  AUTO_TURN_ALIGN,    // settle onto the chosen lane before spraying again
  AUTO_COMPLETE
};
AutoState state = AUTO_IDLE;

inline bool isNavigating() {
  return state == AUTO_PASS || state == AUTO_SHUFFLE_FWD ||
         state == AUTO_SHUFFLE_REV || state == AUTO_BACKOUT ||
         state == AUTO_TURN_ALIGN;
}

// A full-lock arc sweeps sideways by the turning diameter -- several lane
// widths, and wider than a small plot is across. So the 180 is done as a
// sequence of short forward/reverse legs at opposite lock, which rotate the
// rover while leaving it roughly where it started.
const float SHUFFLE_LEG_FT      = 1.2f;
const float ROTATION_DONE_DEG   = 170.0f;
const float HEADLAND_MARGIN_FT  = 3.0f;
const int   MAX_SHUFFLE_LEGS    = 24;

bool turnRightward = true;
int  shuffleLegs = 0;
float legStartX = 0.0f, legStartY = 0.0f;

inline float distanceFromLegStart() {
  float dx = robotX_ft - legStartX;
  float dy = robotY_ft - legStartY;
  return sqrtf(dx * dx + dy * dy);
}

inline float rotationSoFar() {
  return fabsf(angleDiffDeg(robotHeading, turnStartHeading));
}

// Lanes are no longer driven in index order: a full-lock 180 shifts sideways
// by the turning diameter, which is many lane widths, so after each turn we
// take whichever uncovered lane we actually ended up nearest. Direction of
// travel therefore depends on which end of the lane we start from, not on
// whether the lane index is odd or even.
bool passGoesUp = true;
inline float passTargetY()      { return passGoesUp ? fieldPassFt : 0.0f; }
inline float passDesiredHeading(){ return passGoesUp ? 0.0f : 180.0f; }

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
  if (totalLanes > MAX_LANES) totalLanes = MAX_LANES;
  currentLane = 0;
  for (int i = 0; i < MAX_LANES; i++) laneCovered[i] = false;
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

  // Prefer measured course while it is trustworthy, so the phone's mounting
  // angle drops out; fall back to reported heading when barely moving.
  float reference = courseValid ? courseDeg : robotHeading;
  float headingTerm = HEADING_GAIN * angleDiffDeg(desiredHeading, reference);

  steerRightward(crossTerm + headingTerm);
}

bool insideField() {
  return robotY_ft >= 0.0f && robotY_ft <= fieldPassFt &&
         robotX_ft >= -0.5f && robotX_ft <= fieldWidthFt + 0.5f;
}

int nearestUncoveredLane() {
  int best = -1;
  float bestDist = 1e9f;
  for (int i = 0; i < totalLanes; i++) {
    if (laneCovered[i]) continue;
    float d = fabsf(laneCenterX(i, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION) - robotX_ft);
    if (d < bestDist) { bestDist = d; best = i; }
  }
  return best;
}

int lanesRemaining() {
  int n = 0;
  for (int i = 0; i < totalLanes; i++) if (!laneCovered[i]) n++;
  return n;
}

void beginPass(int lane) {
  currentLane = lane;
  // Run whichever way needs less backtracking from where the turn left us.
  passGoesUp = (robotY_ft < fieldPassFt * 0.5f);
  phaseStart = millis();
  resetCourse();
  state = AUTO_PASS;
  bleLog(">>> Lane " + String(lane + 1) + " (x=" +
         String(laneCenterX(lane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION), 2) + " ft, " +
         String(passGoesUp ? "up" : "down") + "), " +
         String(lanesRemaining()) + " left");
}

void beginTurn() {
  setSpray(false);
  turnStartHeading = robotHeading;
  phaseStart = millis();
  shuffleLegs = 0;
  legStartX = robotX_ft;
  legStartY = robotY_ft;

  // Rotate toward whichever side still has uncovered lanes. Heading up, +X is
  // to the rover's right; heading down it is to its left.
  bool workToPositiveX = false;
  for (int i = 0; i < totalLanes; i++) {
    if (!laneCovered[i] &&
        laneCenterX(i, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION) > robotX_ft) {
      workToPositiveX = true;
      break;
    }
  }
  turnRightward = passGoesUp ? workToPositiveX : !workToPositiveX;

  state = AUTO_SHUFFLE_FWD;
  bleLog(">>> Turning (" + String(turnRightward ? "right" : "left") + ")");
}

void runPass() {
  float laneX = laneCenterX(currentLane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  holdLane(laneX, passDesiredHeading(), passGoesUp);
  setChannelPulse(ESC_CH, THROTTLE_FWD_US);

  // Spray only on the straight run and only over the rectangle itself, so the
  // path may overrun the ends but the wetted area stays a clean rectangle.
  setSpray(insideField());

  float targetY = passTargetY();
  bool reached = passGoesUp ? (robotY_ft >= targetY - PASS_END_TOL_FT)
                            : (robotY_ft <= targetY + PASS_END_TOL_FT);
  if (reached) {
    setSpray(false);
    laneCovered[currentLane] = true;

    if (lanesRemaining() == 0) {
      stopDrive();
      state = AUTO_COMPLETE;
      bleLog("=== MISSION COMPLETE: " + String(totalLanes) + " lanes ===");
    } else {
      beginTurn();
    }
  }
}

// Once rotated, choose the lane and the end of the field to work from.
void finishRotation() {
  int lane = nearestUncoveredLane();
  if (lane < 0) {
    stopDrive();
    state = AUTO_COMPLETE;
    bleLog("=== MISSION COMPLETE ===");
    return;
  }
  currentLane = lane;
  // Whichever end we are nearer to is the end we set off from.
  passGoesUp = (robotY_ft < fieldPassFt * 0.5f);
  phaseStart = millis();
  resetCourse();
  state = AUTO_BACKOUT;
  bleLog(">>> Lane " + String(lane + 1) + " next (x=" +
         String(laneCenterX(lane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION), 2) +
         ", off by " +
         String(laneCenterX(lane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION) - robotX_ft, 2) +
         " ft)");
}

void runTurn() {
  bool timedOut = (millis() - phaseStart > TURN_PHASE_TIMEOUT_MS);

  // --- Rotate on the spot -------------------------------------------------
  // Full lock one way going forward, full lock the other way reversing, both
  // of which rotate the same direction. Legs are kept short so the rover
  // pivots rather than driving a wide arc it has no room for.
  if (state == AUTO_SHUFFLE_FWD || state == AUTO_SHUFFLE_REV) {
    bool forward = (state == AUTO_SHUFFLE_FWD);
    steerRightward((turnRightward == forward) ? MAX_STEER_OFFSET : -MAX_STEER_OFFSET);
    setChannelPulse(ESC_CH, forward ? THROTTLE_TURN_US : THROTTLE_REV_US);

    if (rotationSoFar() >= ROTATION_DONE_DEG) {
      stopDrive();
      finishRotation();
      return;
    }

    if (distanceFromLegStart() >= SHUFFLE_LEG_FT || timedOut) {
      stopDrive();
      shuffleLegs++;
      if (shuffleLegs >= MAX_SHUFFLE_LEGS) {
        bleLog("!! Turn did not complete in " + String(MAX_SHUFFLE_LEGS) +
               " legs; continuing anyway.");
        finishRotation();
        return;
      }
      legStartX = robotX_ft;
      legStartY = robotY_ft;
      phaseStart = millis();
      state = forward ? AUTO_SHUFFLE_REV : AUTO_SHUFFLE_FWD;
    }
    return;
  }

  // --- Back clear of the rectangle ---------------------------------------
  // Reversing away from the field buys the run-up needed to settle onto the
  // lane before re-entering, instead of converging halfway down it.
  if (state == AUTO_BACKOUT) {
    float wantY = passGoesUp ? -HEADLAND_MARGIN_FT
                             : fieldPassFt + HEADLAND_MARGIN_FT;
    bool clear = passGoesUp ? (robotY_ft <= wantY) : (robotY_ft >= wantY);

    if (clear || timedOut) {
      stopDrive();
      phaseStart = millis();
      resetCourse();
      state = AUTO_TURN_ALIGN;
      return;
    }

    // Reversing, so counter-steer: the back of the car leads.
    float laneX = laneCenterX(currentLane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
    float crossErr = laneX - robotX_ft;
    steerRightward((passGoesUp ? -1.0f : 1.0f) * CROSS_TRACK_GAIN * crossErr);
    setChannelPulse(ESC_CH, THROTTLE_REV_US);
    return;
  }

  // --- Settle onto the lane ----------------------------------------------
  float laneX = laneCenterX(currentLane, BAR_WIDTH_FT, LANE_OVERLAP_FRACTION);
  float desired = passDesiredHeading();
  holdLane(laneX, desired, passGoesUp);
  setChannelPulse(ESC_CH, THROTTLE_TURN_US);

  float reference = courseValid ? courseDeg : robotHeading;
  bool onLine = fabsf(laneX - robotX_ft) < ALIGN_TOL_FT;
  bool onHeading = fabsf(angleDiffDeg(desired, reference)) < ALIGN_HEADING_TOL;

  // Never keep aligning past the boundary, or the near end of the lane goes
  // unsprayed while the rover is still converging.
  bool enteringField = passGoesUp ? (robotY_ft > 0.0f) : (robotY_ft < fieldPassFt);

  if ((onLine && onHeading) || enteringField || timedOut) {
    if (timedOut) bleLog("!! Align timed out; starting pass anyway.");
    beginPass(currentLane);
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
      // Only meaningful while driving forward, so leave it alone mid-turn.
      if (state == AUTO_PASS || state == AUTO_TURN_ALIGN) updateCourse();
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
    if (state == AUTO_PASS)             phase = "RUN";
    else if (state == AUTO_SHUFFLE_FWD) phase = "ROT-F";
    else if (state == AUTO_SHUFFLE_REV) phase = "ROT-R";
    else if (state == AUTO_BACKOUT)     phase = "BACK";
    else if (state == AUTO_TURN_ALIGN)  phase = "ALIGN";
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
