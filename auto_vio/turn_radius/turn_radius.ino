/*
 * SafeSpread - Turning Radius Measurement
 *
 * Drives one full-lock circle each way and reports the radius it actually
 * traced, measured from the phone's ARKit positions. The route planner in
 * auto_vio.ino builds every turn around `turnRadiusFt`; until that number is
 * measured it is a guess, and a wrong guess produces turns the rover cannot
 * physically drive -- which looks like erratic steering rather than a bad
 * constant.
 *
 * HOW TO RUN
 *   1. Flash this sketch instead of auto_vio.ino.
 *   2. Put the rover in open, flat, level ground with room for a circle of
 *      about 30 x 30 ft. It will drive two circles, one each way.
 *   3. Open the SafeSpreadVIO app, connect, and press Start. (No app changes
 *      are needed: this sketch answers the same '1' / '2' commands.)
 *   4. Read the result off the rover panel. Press Stop at any time to abort.
 *   5. Put the reported number into `turnRadiusFt` in auto_vio.ino and
 *      reflash that sketch.
 *
 * Two independent estimates are reported per direction -- a least-squares
 * circle through the positions, and distance travelled divided by angle swept.
 * They share no arithmetic, so agreement is evidence the measurement is real.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include "circle_fit.h"
#include "measurement.h"
#include "../throttle.h"

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define PCA9685_ADDR 0x40
const uint8_t STEER_CH = 0;
const uint8_t ESC_CH   = 4;

// Matching auto_vio.ino. If they are changed there, change them here too --
// a radius measured at a different steering angle describes a different rover.
const int NEUTRAL_US = 1500;

// Servoed to the same ground speed auto_vio.ino drives at. Turning radius
// depends on speed, so a radius measured at some other speed is a number about
// a different rover -- and a fixed pulse that turns an empty rover will not
// turn a loaded one at all.
const float TARGET_SPEED_FPS      = 1.8f;
const float THROTTLE_MIN_OFFSET   = 70.0f;
const float THROTTLE_MAX_OFFSET   = 400.0f;
const float THROTTLE_START_OFFSET = 150.0f;
const float THROTTLE_GAIN         = 15.0f;
const unsigned long THROTTLE_UPDATE_MS = 200;
const float SPEED_SMOOTHING       = 0.4f;

const int STEER_CENTER_US  = 1500;
const int STEER_LEFT_US    = 2390;
const int STEER_RIGHT_US   = 700;

const unsigned long VIO_TIMEOUT_MS = 1000;

// --- measurement parameters ----------------------------------------------
const float SAMPLE_SPACING_FT   = 0.25f;   // record a position every this far
const int   MAX_SAMPLES         = 400;    // 100 ft of path, well past one circle
const float TARGET_SWEEP_DEG    = 350.0f;  // stop just shy of closing the loop
const float MIN_USABLE_SWEEP    = 90.0f;   // below this the fit is not trustworthy
const float MAX_LEG_DISTANCE_FT = 150.0f;  // runaway guard for a very shallow turn
const unsigned long SETTLE_MS   = 900;     // let the servo reach full lock first
const unsigned long PAUSE_MS    = 2500;    // ESC must pass through neutral
const unsigned long LEG_TIMEOUT_MS = 45000;

// --- pose from the phone --------------------------------------------------
float robotX_ft    = 0.0f;
float robotY_ft    = 0.0f;
float robotHeading = 0.0f;
bool  vioActive    = false;
unsigned long lastVioTime = 0;
unsigned long packetCount = 0;

float speedFps = 0.0f;
float throttleOffsetUs = THROTTLE_START_OFFSET;
float speedPrevX = 0.0f, speedPrevY = 0.0f;
unsigned long speedPrevMs = 0;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
BLECharacteristic *txCharacteristic = NULL;
volatile bool bleConnected = false;

void bleLog(String msg) {
  if (bleConnected && txCharacteristic != NULL) {
    msg += "\n";
    txCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    txCharacteristic->notify();
  }
}

void setChannelPulse(uint8_t channel, int microseconds) {
  uint16_t ticks = (uint16_t)(((uint32_t)microseconds * 4096UL) / 20000UL);
  pwm.setPWM(channel, 0, ticks);
}

void stopDrive() {
  setChannelPulse(ESC_CH, NEUTRAL_US);
  setChannelPulse(STEER_CH, STEER_CENTER_US);
}

// --- packet parsing -------------------------------------------------------
// Deliberately duplicated from nav_math.h: the Arduino build copies only the
// files inside this sketch folder, so a shared header cannot be included from
// the parent. Fifteen bytes of format is a cheaper duplication than a build
// system change, but it must be kept in step with the app's sender.
bool parsePose(const uint8_t *d, size_t n, float &x, float &y, float &heading) {
  if (n < 15 || d[0] != 0x21 || d[1] != 0x50) return false;
  uint8_t sum = 0;
  for (int i = 0; i < 14; i++) sum += d[i];
  if ((uint8_t)(~sum) != d[14]) return false;
  memcpy(&x, d + 2, 4);
  memcpy(&y, d + 6, 4);
  memcpy(&heading, d + 10, 4);
  return true;
}

float angleDiffDeg(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// --- measurement state ----------------------------------------------------
Phase phase = P_IDLE;

int leg = 0;  // 0 = full left lock, 1 = full right lock
const int   LEG_PULSE[2] = { STEER_LEFT_US, STEER_RIGHT_US };
const char *LEG_NAME[2]  = { "LEFT ", "RIGHT" };

float sampleX[MAX_SAMPLES];
float sampleY[MAX_SAMPLES];
int   sampleCount = 0;

float pathLengthFt = 0.0f;
float sweptDeg     = 0.0f;   // signed: + means heading increased (clockwise)
float lastHeading  = 0.0f;
float lastSampleX  = 0.0f, lastSampleY = 0.0f;
unsigned long phaseStart = 0;

LegResult results[2];
bool haveResults = false;

void resetThrottle() {
  throttleOffsetUs = THROTTLE_START_OFFSET;
  speedFps = 0.0f;
  speedPrevX = robotX_ft;
  speedPrevY = robotY_ft;
  speedPrevMs = millis();
}

// Wind the throttle to whatever actually delivers the target speed. Measuring
// a turning circle at a speed the loaded rover cannot reach would describe a
// rover that does not exist.
void driveAtTargetSpeed() {
  unsigned long now = millis();
  if (now - speedPrevMs >= THROTTLE_UPDATE_MS) {
    speedFps = updateSpeedFps(speedFps, robotX_ft - speedPrevX,
                              robotY_ft - speedPrevY, now - speedPrevMs,
                              SPEED_SMOOTHING);
    speedPrevX = robotX_ft;
    speedPrevY = robotY_ft;
    speedPrevMs = now;
    throttleOffsetUs = governThrottle(throttleOffsetUs, speedFps, TARGET_SPEED_FPS,
                                      THROTTLE_GAIN, THROTTLE_MIN_OFFSET,
                                      THROTTLE_MAX_OFFSET);
  }
  setChannelPulse(ESC_CH, NEUTRAL_US + (int)throttleOffsetUs);
}

void resetLegState() {
  sampleCount = 0;
  pathLengthFt = 0.0f;
  sweptDeg = 0.0f;
  lastHeading = robotHeading;
  lastSampleX = robotX_ft;
  lastSampleY = robotY_ft;
}

void beginLeg(int which) {
  leg = which;
  bleLog(">>> Measuring " + String(LEG_NAME[which]) + " turn (steer " +
         String(LEG_PULSE[which]) + "us). Stand clear.");
  setChannelPulse(STEER_CH, LEG_PULSE[which]);
  setChannelPulse(ESC_CH, NEUTRAL_US);
  phase = P_SETTLE;
  phaseStart = millis();
}

// Prefer the circle fit, which uses the whole shape; fall back to the
// arc-length estimate when the fit was refused (a near-straight path).
float bestRadius(const LegResult &r) {
  if (r.fitOk && r.fitR > 0.0f) return r.fitR;
  return r.arcR;
}

void reportResults() {
  bleLog("=== TURN RADIUS RESULT ===");

  for (int i = 0; i < 2; i++) {
    const LegResult &r = results[i];
    if (!r.valid) {
      bleLog(String(LEG_NAME[i]) + ": FAILED - only " + String(fabsf(r.sweptDeg), 0) +
             " deg swept (need " + String((int)MIN_USABLE_SWEEP) + ").");
      continue;
    }

    bleLog(String(LEG_NAME[i]) + ": R = " + String(bestRadius(r), 2) + " ft" +
           "  (circle " + String(2.0f * bestRadius(r), 1) + " ft)");
    bleLog("       fit " + String(r.fitR, 2) + " +/-" + String(r.fitRms, 2) +
           " | arc " + String(r.arcR, 2) +
           " | rotated " + String(r.sweptDeg > 0 ? "CW" : "CCW"));

    // The two estimates are independent, so a disagreement means the samples
    // are not describing a clean circle -- wheel slip, a slope, or VIO drift.
    if (r.fitOk && r.arcR > 0.0f) {
      float diff = fabsf(r.fitR - r.arcR) / bestRadius(r);
      if (diff > 0.25f) {
        bleLog("       !! estimates disagree by " + String(diff * 100.0f, 0) +
               "% -- suspect wheel slip or uneven ground. Re-run.");
      }
    }
    if (r.fitOk && r.fitRms > 0.20f * r.fitR) {
      bleLog("       !! path was not a clean circle (scatter " +
             String(r.fitRms, 2) + " ft).");
    }
  }

  // Steering polarity, settled by observation. Full LEFT lock should rotate
  // the rover counter-clockwise; if both legs turned the same way, or the
  // wrong way, the servo is wired backwards and auto_vio.ino's correction
  // sense is inverted -- which looks like wandering, not like a fault.
  if (results[0].valid && results[1].valid) {
    bool leftCCW = results[0].sweptDeg < 0.0f;
    bool rightCW = results[1].sweptDeg > 0.0f;
    if (leftCCW && rightCW) {
      bleLog("[OK] Steering polarity normal (left lock turns left).");
    } else if (!leftCCW && !rightCW) {
      bleLog("!! STEERING IS REVERSED: left lock turned the rover right.");
      bleLog("   Flip the CH0 servo plug, or set steerSign = -1 in auto_vio.ino.");
    } else {
      bleLog("!! Both legs turned the same way -- steering may not be moving.");
    }

    float lo = bestRadius(results[0]), hi = bestRadius(results[1]);
    if (hi < lo) { float t = lo; lo = hi; hi = t; }
    if (lo > 0.0f && (hi - lo) / lo > 0.20f) {
      bleLog("Note: the two directions differ by " +
             String((hi - lo) / lo * 100.0f, 0) + "%. Steering trim is off-centre.");
    }

    // Plan with the worse of the two: a route built around the tighter circle
    // contains turns the rover cannot make in the other direction.
    bleLog(">>> Set turnRadiusFt = " + String(hi, 1) + " in auto_vio.ino");
    bleLog("    (larger of the two, so every planned turn is drivable both ways)");
  } else {
    bleLog("!! Incomplete measurement -- nothing to recommend. Re-run with more room.");
  }
  bleLog("=== END ===");
}

void finishLeg(const char *why) {
  stopDrive();

  LegResult &r = results[leg];
  CircleFit f = fitCircle(sampleX, sampleY, sampleCount);
  r.fitOk    = f.ok;
  r.fitR     = f.ok ? f.r : 0.0f;
  r.fitRms   = f.ok ? f.rms : 0.0f;
  r.arcR     = arcRadius(pathLengthFt, fabsf(sweptDeg));
  r.sweptDeg = sweptDeg;
  r.pathLen  = pathLengthFt;
  r.samples  = sampleCount;
  r.valid    = fabsf(sweptDeg) >= MIN_USABLE_SWEEP && (f.ok || r.arcR > 0.0f);

  bleLog("    " + String(LEG_NAME[leg]) + " leg ended (" + String(why) + "): " +
         String(fabsf(sweptDeg), 0) + " deg swept, " + String(pathLengthFt, 1) +
         " ft driven, " + String(sampleCount) + " samples.");

  if (leg == 0) {
    phase = P_PAUSE;
    phaseStart = millis();
  } else {
    phase = P_DONE;
    haveResults = true;
    reportResults();
  }
}

void abortRun(const char *why) {
  stopDrive();
  phase = P_IDLE;
  bleLog(">>> Aborted: " + String(why));
}

// --- BLE plumbing ---------------------------------------------------------
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
      if (phase != P_IDLE && phase != P_DONE) {
        bleLog("!! Already measuring. Press Stop first.");
      } else if (!vioActive) {
        bleLog("!! No pose data from the phone yet. Wait for tracking, then Start.");
      } else {
        haveResults = false;
        results[0].valid = results[1].valid = false;
        bleLog("=== TURN RADIUS MEASUREMENT ===");
        bleLog("Two full-lock circles, left then right. Needs ~30x30 ft clear.");
        resetLegState();
        beginLeg(0);
      }
    } else if (d[0] == '2') {
      if (phase == P_IDLE || phase == P_DONE) {
        stopDrive();
      } else {
        abortRun("stopped by user");
      }
    } else if (d[0] == '3') {
      if (haveResults) reportResults();
      else bleLog("No measurement yet. Press Start.");
    } else if (d[0] == '4') {
      bleLog("This sketch only measures. Reflash auto_vio.ino to spray.");
    }
    return;
  }

  for (size_t i = 0; i < n; i++) {
    if (accLen >= sizeof(acc)) { memmove(acc, acc + 1, accLen - 1); accLen--; }
    acc[accLen++] = d[i];
  }

  size_t i = 0;
  while (accLen - i >= 15) {
    if (acc[i] != '!') { i++; continue; }
    float x, y, heading;
    if (parsePose(&acc[i], accLen - i, x, y, heading)) {
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
    bleLog("Turn-radius sketch ready.");
    bleLog("Press Start to drive two full-lock circles and measure the radius.");
  }
  void onDisconnect(BLEServer *s) {
    bleConnected = false;
    vioActive = false;
    phase = P_IDLE;
    stopDrive();
    s->startAdvertising();
  }
};

// --- recording ------------------------------------------------------------
void recordProgress() {
  // Angle swept, accumulated from the phone's heading. Any constant error in
  // how the phone is mounted cancels out of a difference, so mounting angle
  // does not affect this.
  sweptDeg += angleDiffDeg(robotHeading, lastHeading);
  lastHeading = robotHeading;

  float dx = robotX_ft - lastSampleX;
  float dy = robotY_ft - lastSampleY;
  float moved = sqrtf(dx * dx + dy * dy);
  if (moved < SAMPLE_SPACING_FT) return;

  if (sampleCount < MAX_SAMPLES) {
    sampleX[sampleCount] = robotX_ft;
    sampleY[sampleCount] = robotY_ft;
    sampleCount++;
  }
  pathLengthFt += moved;
  lastSampleX = robotX_ft;
  lastSampleY = robotY_ft;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(50);
  stopDrive();

  results[0].valid = results[1].valid = false;

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

  Serial.println("Turn radius measurement ready. Logging to app from here.");
}

unsigned long lastTelemetry = 0;

void loop() {
  while (qTail != qHead) {
    feed(qData[qTail], qLen[qTail]);
    qTail = (qTail + 1) % QSLOTS;
  }

  if (vioActive && (millis() - lastVioTime > VIO_TIMEOUT_MS)) {
    vioActive = false;
    if (phase == P_SETTLE || phase == P_RECORD) abortRun("VIO signal lost");
    else stopDrive();
  }

  switch (phase) {
    case P_SETTLE:
      // Hold full lock with the wheels turning slowly before recording, so the
      // arc that gets fitted is a steady-state circle and not the servo slewing.
      driveAtTargetSpeed();
      if (millis() - phaseStart >= SETTLE_MS) {
        resetLegState();
        resetThrottle();
        phase = P_RECORD;
        phaseStart = millis();
      }
      break;

    case P_RECORD:
      setChannelPulse(STEER_CH, LEG_PULSE[leg]);
      driveAtTargetSpeed();
      recordProgress();

      if (fabsf(sweptDeg) >= TARGET_SWEEP_DEG)        finishLeg("full circle");
      else if (sampleCount >= MAX_SAMPLES)            finishLeg("sample limit");
      else if (pathLengthFt >= MAX_LEG_DISTANCE_FT)   finishLeg("distance limit");
      else if (millis() - phaseStart >= LEG_TIMEOUT_MS) finishLeg("timeout");
      break;

    case P_PAUSE:
      // The ESC has to see neutral before it will arm in a new direction, and
      // the rover needs to actually stop before the next arc begins.
      stopDrive();
      if (millis() - phaseStart >= PAUSE_MS) {
        resetLegState();
        beginLeg(1);
      }
      break;

    default:
      stopDrive();
      break;
  }

  if (millis() - lastTelemetry >= 1000) {
    lastTelemetry = millis();
    const char *names[] = { "IDLE", "SETTLE", "TURNING", "PAUSE", "DONE" };
    bleLog("[TLM] " + String(names[phase]) +
           (phase == P_RECORD || phase == P_SETTLE ? " " + String(LEG_NAME[leg]) : "") +
           " vio=" + String(vioActive ? "OK" : "NONE") +
           " pkts=" + String(packetCount) +
           " pos=(" + String(robotX_ft, 1) + "," + String(robotY_ft, 1) + ")" +
           " swept=" + String(sweptDeg, 0) + "deg" +
           " spd=" + String(speedFps, 1) +
           " thr=" + String((int)throttleOffsetUs) +
           " n=" + String(sampleCount));
  }
}
