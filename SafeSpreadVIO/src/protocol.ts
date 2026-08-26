export function buildPosePacket(x: number, y: number, heading: number): Uint8Array {
  const buf = new ArrayBuffer(15);
  const view = new DataView(buf);
  const bytes = new Uint8Array(buf);

  bytes[0] = 0x21;
  bytes[1] = 0x50;
  view.setFloat32(2, x, true);
  view.setFloat32(6, y, true);
  view.setFloat32(10, heading, true);

  let sum = 0;
  for (let i = 0; i < 14; i++) sum += bytes[i];
  bytes[14] = (~sum) & 0xFF;

  return bytes;
}
