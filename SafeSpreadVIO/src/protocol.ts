/**
 * 11-byte `!D` packet telling the rover the field size to cover:
 * [0]='!' [1]='D' [2..5]=width ft f32le [6..9]=length ft f32le [10]=CRC
 */
export function buildAreaPacket(widthFt: number, lengthFt: number): Uint8Array {
  const buf = new ArrayBuffer(11);
  const view = new DataView(buf);
  const bytes = new Uint8Array(buf);

  bytes[0] = 0x21;
  bytes[1] = 0x44; // 'D'
  view.setFloat32(2, widthFt, true);
  view.setFloat32(6, lengthFt, true);

  let sum = 0;
  for (let i = 0; i < 10; i++) sum += bytes[i];
  bytes[10] = (~sum) & 0xFF;

  return bytes;
}

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
