#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "steering_map.h"

constexpr int MAX_CALIBRATION_SAMPLES = 32;
constexpr int MAX_CALIBRATION_KNOTS = 9;
constexpr float MIN_CALIBRATION_SWEEP_DEG = 60.0f;

struct SteeringCalibrationSample {
  int pulseUs;
  float curvaturePerFt;
  float sweepDeg;
  int8_t direction;
};

struct SteeringCalibrationFit {
  SteeringKnot knots[MAX_CALIBRATION_KNOTS];
  uint8_t count;
  bool valid;
};

struct SpeedCalibrationSample {
  int pulseUs;
  float speedFps;
  float distanceFt;
  int8_t direction;
};

inline float calibrationMedian(float *values, int count) {
  for (int index = 1; index < count; ++index) {
    const float value = values[index];
    int insert = index;
    while (insert > 0 && values[insert - 1] > value) {
      values[insert] = values[insert - 1];
      --insert;
    }
    values[insert] = value;
  }
  const int middle = count / 2;
  return count % 2 ? values[middle] : (values[middle - 1] + values[middle]) * 0.5f;
}

inline bool fitSteeringCalibration(const SteeringCalibrationSample *samples,
                                   int count, int straightPulseUs,
                                   SteeringCalibrationFit &out) {
  out = {};
  if (samples == nullptr || count < 2 || count > MAX_CALIBRATION_SAMPLES ||
      straightPulseUs < 500 || straightPulseUs > 2500) return false;

  struct PulseGroup {
    int pulseUs;
    float curvatures[MAX_CALIBRATION_SAMPLES];
    int count;
  } groups[MAX_CALIBRATION_KNOTS - 1] = {};
  int groupCount = 0;

  for (int index = 0; index < count; ++index) {
    const SteeringCalibrationSample &sample = samples[index];
    if ((sample.direction != 1 && sample.direction != -1) ||
        sample.pulseUs < 500 || sample.pulseUs > 2500 ||
        sample.pulseUs == straightPulseUs ||
        !std::isfinite(sample.curvaturePerFt) ||
        !std::isfinite(sample.sweepDeg) ||
        std::fabs(sample.sweepDeg) < MIN_CALIBRATION_SWEEP_DEG ||
        std::fabs(sample.curvaturePerFt) < 1e-4f) return false;

    int group = -1;
    for (int candidate = 0; candidate < groupCount; ++candidate) {
      if (groups[candidate].pulseUs == sample.pulseUs) {
        group = candidate;
        break;
      }
    }
    if (group < 0) {
      if (groupCount >= MAX_CALIBRATION_KNOTS - 1) return false;
      group = groupCount++;
      groups[group].pulseUs = sample.pulseUs;
    }
    const float normalized = sample.direction > 0
        ? sample.curvaturePerFt : -sample.curvaturePerFt;
    groups[group].curvatures[groups[group].count++] = normalized;
  }
  if (groupCount < 2) return false;

  out.count = static_cast<uint8_t>(groupCount + 1);
  for (int group = 0; group < groupCount; ++group) {
    out.knots[group] = {
      groups[group].pulseUs,
      calibrationMedian(groups[group].curvatures, groups[group].count),
    };
  }
  out.knots[groupCount] = {straightPulseUs, 0.0f};

  for (int index = 1; index < out.count; ++index) {
    const SteeringKnot knot = out.knots[index];
    int insert = index;
    while (insert > 0 && out.knots[insert - 1].curvaturePerFt > knot.curvaturePerFt) {
      out.knots[insert] = out.knots[insert - 1];
      --insert;
    }
    out.knots[insert] = knot;
  }
  out.valid = validSteeringMap(out.knots, out.count);
  return out.valid;
}

inline bool fitSpeedFeedForward(const SpeedCalibrationSample *samples, int count,
                                int8_t expectedDirection, int neutralPulseUs,
                                float &feedForwardUs) {
  feedForwardUs = 0.0f;
  if (samples == nullptr || count < 3 || count > MAX_CALIBRATION_SAMPLES ||
      (expectedDirection != 1 && expectedDirection != -1)) return false;
  float offsets[MAX_CALIBRATION_SAMPLES];
  for (int index = 0; index < count; ++index) {
    const SpeedCalibrationSample &sample = samples[index];
    const int offset = sample.pulseUs - neutralPulseUs;
    if (sample.direction != expectedDirection ||
        !std::isfinite(sample.speedFps) || !std::isfinite(sample.distanceFt) ||
        sample.distanceFt < 3.0f || sample.speedFps * expectedDirection <= 0.0f ||
        offset * expectedDirection <= 0 || std::abs(offset) > 350) return false;
    offsets[index] = static_cast<float>(std::abs(offset));
  }
  feedForwardUs = calibrationMedian(offsets, count);
  return feedForwardUs > 0.0f && feedForwardUs <= 350.0f;
}

constexpr uint32_t COMPACT_CALIBRATION_MAGIC = 0x5343414c;  // SCAL

struct CompactMotionCalibration {
  uint32_t magic;
  uint16_t schemaVersion;
  uint16_t calibrationId;
  uint32_t hardwareTagHash;
  SteeringKnot knots[MAX_CALIBRATION_KNOTS];
  uint8_t knotCount;
  float forwardFeedForwardUs;
  float reverseFeedForwardUs;
  uint8_t reverseVerified;
  uint16_t checksum;
};

inline uint16_t calibrationChecksum(const CompactMotionCalibration &calibration) {
  uint16_t crc = 0xffff;
  auto addByte = [&](uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  };
  auto addValue = [&](const void *value, size_t size) {
    const uint8_t *bytes = static_cast<const uint8_t *>(value);
    for (size_t index = 0; index < size; ++index) addByte(bytes[index]);
  };
  addValue(&calibration.magic, sizeof(calibration.magic));
  addValue(&calibration.schemaVersion, sizeof(calibration.schemaVersion));
  addValue(&calibration.calibrationId, sizeof(calibration.calibrationId));
  addValue(&calibration.hardwareTagHash, sizeof(calibration.hardwareTagHash));
  addValue(calibration.knots, sizeof(calibration.knots));
  addValue(&calibration.knotCount, sizeof(calibration.knotCount));
  addValue(&calibration.forwardFeedForwardUs, sizeof(calibration.forwardFeedForwardUs));
  addValue(&calibration.reverseFeedForwardUs, sizeof(calibration.reverseFeedForwardUs));
  addValue(&calibration.reverseVerified, sizeof(calibration.reverseVerified));
  return crc;
}

inline CompactMotionCalibration makeCompactCalibration(
    uint16_t schemaVersion, uint16_t calibrationId, uint32_t hardwareTagHash,
    const SteeringCalibrationFit &steering, float forwardFeedForwardUs,
    float reverseFeedForwardUs, bool reverseVerified) {
  CompactMotionCalibration result = {};
  result.magic = COMPACT_CALIBRATION_MAGIC;
  result.schemaVersion = schemaVersion;
  result.calibrationId = calibrationId;
  result.hardwareTagHash = hardwareTagHash;
  result.knotCount = steering.valid ? steering.count : 0;
  for (int index = 0; index < result.knotCount && index < MAX_CALIBRATION_KNOTS; ++index) {
    result.knots[index] = steering.knots[index];
  }
  result.forwardFeedForwardUs = forwardFeedForwardUs;
  result.reverseFeedForwardUs = reverseFeedForwardUs;
  result.reverseVerified = reverseVerified ? 1 : 0;
  result.checksum = calibrationChecksum(result);
  return result;
}

inline bool compactCalibrationValid(const CompactMotionCalibration &calibration) {
  return calibration.magic == COMPACT_CALIBRATION_MAGIC &&
         calibration.schemaVersion == 1 && calibration.hardwareTagHash != 0 &&
         calibration.knotCount >= 3 && calibration.knotCount <= MAX_CALIBRATION_KNOTS &&
         validSteeringMap(calibration.knots, calibration.knotCount) &&
         std::isfinite(calibration.forwardFeedForwardUs) &&
         calibration.forwardFeedForwardUs > 0.0f && calibration.forwardFeedForwardUs <= 350.0f &&
         std::isfinite(calibration.reverseFeedForwardUs) &&
         calibration.reverseFeedForwardUs > 0.0f && calibration.reverseFeedForwardUs <= 350.0f &&
         calibration.reverseVerified == 1 &&
         calibration.checksum == calibrationChecksum(calibration);
}

inline bool calibrationIdentityMatches(const CompactMotionCalibration &calibration,
                                       uint16_t schemaVersion, uint16_t calibrationId,
                                       uint32_t hardwareTagHash) {
  return compactCalibrationValid(calibration) &&
         calibration.schemaVersion == schemaVersion &&
         calibration.calibrationId == calibrationId &&
         calibration.hardwareTagHash == hardwareTagHash;
}

template <typename Store>
class MotionCalibrationPersistence {
 public:
  explicit MotionCalibrationPersistence(Store &store) : store_(store) {}
  bool save(const CompactMotionCalibration &calibration) {
    return compactCalibrationValid(calibration) &&
           store_.write(reinterpret_cast<const uint8_t *>(&calibration), sizeof(calibration));
  }
  bool load(CompactMotionCalibration &calibration) {
    CompactMotionCalibration candidate = {};
    if (!store_.read(reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate)) ||
        !compactCalibrationValid(candidate)) return false;
    calibration = candidate;
    return true;
  }

 private:
  Store &store_;
};
