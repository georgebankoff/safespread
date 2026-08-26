#include <cassert>
#include <cstdio>
#include <cstring>
#include "../nav_math.h"

static void buildTestPacket(uint8_t* out, float x, float y, float heading) {
  out[0] = 0x21;
  out[1] = 0x50;
  memcpy(out + 2, &x, 4);
  memcpy(out + 6, &y, 4);
  memcpy(out + 10, &heading, 4);
  uint8_t sum = 0;
  for (int i = 0; i < 14; i++) sum += out[i];
  out[14] = (uint8_t)(~sum);
}

int main() {
  uint8_t packet[15];
  buildTestPacket(packet, 2.5f, -1.25f, 90.0f);

  float x = 0, y = 0, heading = 0;
  bool ok = parsePosePacket(packet, sizeof(packet), x, y, heading);
  assert(ok);
  assert(x == 2.5f);
  assert(y == -1.25f);
  assert(heading == 90.0f);

  uint8_t corrupted[15];
  memcpy(corrupted, packet, 15);
  corrupted[14] ^= 0xFF;
  float cx, cy, ch;
  assert(!parsePosePacket(corrupted, sizeof(corrupted), cx, cy, ch));

  assert(!parsePosePacket(packet, 14, x, y, heading));

  uint8_t badHeader[15];
  memcpy(badHeader, packet, 15);
  badHeader[1] = 'X';
  assert(!parsePosePacket(badHeader, sizeof(badHeader), x, y, heading));

  // --- !D area packet ---
  uint8_t area[11];
  area[0] = 0x21;
  area[1] = 0x44;
  float w = 30.0f, l = 16.5f;
  memcpy(area + 2, &w, 4);
  memcpy(area + 6, &l, 4);
  uint8_t asum = 0;
  for (int i = 0; i < 10; i++) asum += area[i];
  area[10] = (uint8_t)(~asum);

  float gotW = 0, gotL = 0;
  assert(parseAreaPacket(area, sizeof(area), gotW, gotL));
  assert(gotW == 30.0f);
  assert(gotL == 16.5f);

  uint8_t areaBad[11];
  memcpy(areaBad, area, 11);
  areaBad[10] ^= 0xFF;
  assert(!parseAreaPacket(areaBad, sizeof(areaBad), gotW, gotL));

  assert(!parseAreaPacket(area, 10, gotW, gotL));

  // A pose packet must not be mistaken for an area packet.
  assert(!parseAreaPacket(packet, sizeof(packet), gotW, gotL));

  printf("parse_test: all assertions passed\n");
  return 0;
}
