import { buildAreaPacket, buildPosePacket } from './protocol';

describe('buildAreaPacket', () => {
  it('writes header, both dimensions little-endian, and a matching CRC', () => {
    const packet = buildAreaPacket(30, 16);
    expect(packet.length).toBe(11);
    expect(packet[0]).toBe(0x21);
    expect(packet[1]).toBe(0x44);

    const view = new DataView(packet.buffer);
    expect(view.getFloat32(2, true)).toBeCloseTo(30);
    expect(view.getFloat32(6, true)).toBeCloseTo(16);

    let sum = 0;
    for (let i = 0; i < 10; i++) sum += packet[i];
    expect(packet[10]).toBe((~sum) & 0xFF);
  });

  it('is distinguishable from a pose packet by its type byte', () => {
    expect(buildAreaPacket(10, 10)[1]).not.toBe(buildPosePacket(0, 0, 0)[1]);
  });
});

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
