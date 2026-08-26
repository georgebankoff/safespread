import { crc16Ccitt } from './protocolV2';
import {
  CalibrationWire,
  MissionControl,
  MissionTransport,
} from './missionControl';
import { RectangleDefinition } from './rectangle';

const rectangle: RectangleDefinition = {
  originWorld: { x: 0, y: 0, heading: 0 },
  mAxisHeadingDeg: 0,
  mFt: 20,
  nFt: 8,
  side: 'right',
  startClearFt: 4,
  endClearFt: 6,
  source: 'entered',
};
const calibration: CalibrationWire = {
  id: 3,
  schemaVersion: 1,
  sprayForwardFt: -0.5,
  sprayRightFt: 0,
};

function ack(epoch: number, commandId: number, state: number, calibrationId = 3) {
  const bytes = new Uint8Array(16);
  const view = new DataView(bytes.buffer);
  bytes.set([0x21, 0x41, 2, state]);
  view.setUint16(4, epoch, true);
  view.setUint32(6, commandId, true);
  view.setUint16(10, 0, true);
  view.setUint16(12, calibrationId, true);
  view.setUint16(14, crc16Ccitt(bytes.subarray(0, 14)), true);
  return bytes;
}

class FakeTransport implements MissionTransport {
  writes: Uint8Array[] = [];
  compatibilityStops = 0;
  listener: ((packet: Uint8Array) => void) | null = null;
  onWrite?: (packet: Uint8Array, writeNumber: number) => void;

  async writeWithResponse(packet: Uint8Array) {
    this.writes.push(packet.slice());
    this.onWrite?.(packet, this.writes.length);
  }
  async writeCompatibilityStop() { this.compatibilityStops += 1; }
  subscribeAck(listener: (packet: Uint8Array) => void) {
    this.listener = listener;
    return () => { this.listener = null; };
  }
  emit(packet: Uint8Array) { this.listener?.(packet); }
}

function packetId(packet: Uint8Array) {
  return new DataView(packet.buffer, packet.byteOffset, packet.byteLength).getUint32(6, true);
}

function autoAckConfiguration(transport: FakeTransport, epoch = 7) {
  transport.onWrite = (packet) => {
    const state = packet[1] === 0x44 ? 1 : 0;
    transport.emit(ack(epoch, packetId(packet), state));
  };
}

async function configuredControl(transport: FakeTransport, fresh = () => true) {
  autoAckConfiguration(transport);
  const control = new MissionControl(transport, 7, fresh, { timeoutMs: 5, retries: 2 });
  await control.configure(rectangle, calibration);
  return control;
}

describe('MissionControl ordering and acknowledgements', () => {
  it('requires configure, a fresh pose, arm, then start', async () => {
    const transport = new FakeTransport();
    const notFresh = new MissionControl(transport, 7, () => false, { timeoutMs: 5, retries: 0 });
    await expect(notFresh.arm()).rejects.toThrow('configured');
    autoAckConfiguration(transport);
    await notFresh.configure(rectangle, calibration);
    await expect(notFresh.start()).rejects.toThrow('armed');
    await expect(notFresh.arm()).rejects.toThrow('fresh');

    const freshTransport = new FakeTransport();
    const control = await configuredControl(freshTransport);
    freshTransport.onWrite = (packet) => freshTransport.emit(ack(7, packetId(packet), 2));
    await control.arm();
    freshTransport.onWrite = (packet) => freshTransport.emit(ack(7, packetId(packet), 3));
    await control.start();
    expect(control.state).toBe('running');
  });

  it('retries twice with the same command ID and ignores wrong ACKs', async () => {
    const transport = new FakeTransport();
    const control = await configuredControl(transport);
    const beforeArm = transport.writes.length;
    transport.onWrite = (packet, writeNumber) => {
      const id = packetId(packet);
      transport.emit(ack(8, id, 2));
      transport.emit(ack(7, id + 1, 2));
      if (writeNumber === beforeArm + 2) {
        transport.emit(ack(7, id, 2));
        transport.emit(ack(7, id, 2)); // duplicate replay is idempotent
      }
    };
    await control.arm();
    const attempts = transport.writes.slice(beforeArm);
    expect(attempts).toHaveLength(2);
    expect(attempts.map(packetId)).toEqual([3, 3]);
    expect(control.state).toBe('armed');
  });

  it('times out after the initial attempt plus two retries', async () => {
    const transport = new FakeTransport();
    const control = await configuredControl(transport);
    const beforeArm = transport.writes.length;
    transport.onWrite = undefined;
    await expect(control.arm()).rejects.toThrow('ACK timeout');
    expect(transport.writes.slice(beforeArm)).toHaveLength(3);
    expect(control.state).toBe('fault');
  });

  it('rejects stale calibration acknowledgements', async () => {
    const transport = new FakeTransport();
    transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0, 99));
    const control = new MissionControl(transport, 7, () => true, { timeoutMs: 5, retries: 0 });
    await expect(control.configure(rectangle, calibration)).rejects.toThrow('calibration');
  });
});

describe('MissionControl emergency stop', () => {
  it.each(['idle', 'configured', 'armed', 'running'] as const)(
    'sends acknowledged v2 Stop plus compatibility Stop from %s',
    async (target) => {
      const transport = new FakeTransport();
      const control = new MissionControl(transport, 7, () => true, { timeoutMs: 5, retries: 0 });
      if (target !== 'idle') {
        autoAckConfiguration(transport);
        await control.configure(rectangle, calibration);
      }
      if (target === 'armed' || target === 'running') {
        transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 2));
        await control.arm();
      }
      if (target === 'running') {
        transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 3));
        await control.start();
      }
      transport.onWrite = (packet) => transport.emit(ack(7, packetId(packet), 0));
      await control.stop();
      expect(control.state).toBe('idle');
      expect(transport.compatibilityStops).toBe(1);
      const stopPacket = transport.writes.at(-1)!;
      expect(stopPacket[1]).toBe(0x43);
      expect(stopPacket[3]).toBe(3);
    },
  );
});
