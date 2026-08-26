#pragma once

// These live in a header rather than in the .ino because the Arduino build
// inserts its generated function prototypes ahead of the first function in the
// sketch -- which is above where a struct declared in the .ino would appear,
// so any function taking one by reference fails to compile.

enum Phase { P_IDLE, P_SETTLE, P_RECORD, P_PAUSE, P_DONE };

struct LegResult {
  bool  valid;
  float fitR;
  float fitRms;
  float arcR;
  float sweptDeg;   // signed: + means the heading increased (clockwise)
  float pathLen;
  int   samples;
  bool  fitOk;
};
