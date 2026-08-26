import { buildPosePacket } from './protocol';

describe('buildPosePacket', () => {
  it('writes header, floats little-endian, and a matching CRC', () => {
    const packet = buildPosePacket(2.5, -1.25, 90);
    expect(packet.length).toBe(15);
    expect(packet[0]).toBe(0x21);
    expect(packet[1]).toBe(0x50);

    const view = new DataView(packet.buffer);
    expect(view.getFloat32(2, true)).toBeCloseTo(2.5);
    expect(view.getFloat32(6, true)).toBeCloseTo(-1.25);
    expect(view.getFloat32(10, true)).toBeCloseTo(90);

    let sum = 0;
    for (let i = 0; i < 14; i++) sum += packet[i];
    expect(packet[14]).toBe((~sum) & 0xFF);
  });

  it('produces a different CRC when a field changes', () => {
    const a = buildPosePacket(0, 0, 0);
    const b = buildPosePacket(1, 0, 0);
    expect(a[14]).not.toBe(b[14]);
  });
});
