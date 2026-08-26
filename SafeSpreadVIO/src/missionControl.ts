import {
  AckV2,
  buildCalibrationV2,
  buildCommandV2,
  buildRectangleV2,
  parseAckV2,
} from './protocolV2';
import { RectangleDefinition } from './rectangle';

export interface CalibrationWire {
  id: number;
  schemaVersion: number;
  sprayForwardFt: number;
  sprayRightFt: number;
}

export interface MissionTransport {
  writeWithResponse(packet: Uint8Array): Promise<void>;
  writeCompatibilityStop(): Promise<void>;
  subscribeAck(listener: (packet: Uint8Array) => void): () => void;
}

export type MissionClientState = 'idle' | 'configured' | 'armed' | 'running' | 'complete' | 'fault';

interface MissionControlOptions {
  timeoutMs?: number;
  retries?: number;
  dryMode?: boolean;
  preferForwardOnly?: boolean;
}

interface PendingAck {
  commandId: number;
  received: AckV2 | null;
  listeners: Set<(ack: AckV2) => void>;
}

export class MissionControl {
  private currentState: MissionClientState = 'idle';
  private nextCommandId = 1;
  private pending: PendingAck | null = null;
  private busy = false;
  private readonly unsubscribe: () => void;
  private readonly timeoutMs: number;
  private readonly retries: number;
  private readonly dryMode: boolean;
  private readonly preferForwardOnly: boolean;
  private calibrationId = 0;

  constructor(
    private readonly transport: MissionTransport,
    private readonly epoch: number,
    private readonly hasFreshPose: () => boolean,
    options: MissionControlOptions = {},
  ) {
    if (!Number.isInteger(epoch) || epoch < 0 || epoch > 0xffff) {
      throw new RangeError('mission epoch must be uint16');
    }
    this.timeoutMs = options.timeoutMs ?? 750;
    this.retries = options.retries ?? 2;
    this.dryMode = options.dryMode ?? true;
    this.preferForwardOnly = options.preferForwardOnly ?? true;
    this.unsubscribe = transport.subscribeAck((packet) => this.receiveAck(packet));
  }

  get state(): MissionClientState {
    return this.currentState;
  }

  async configure(definition: RectangleDefinition, calibration: CalibrationWire): Promise<AckV2> {
    if (this.currentState !== 'idle') throw new Error('mission must be idle before configure');
    return this.exclusive(async () => {
      this.calibrationId = calibration.id;
      const calibrationCommandId = this.takeCommandId();
      const calibrationAck = await this.sendAndWait(buildCalibrationV2({
        flags: 0,
        epoch: this.epoch,
        commandId: calibrationCommandId,
        calibrationId: calibration.id,
        sprayForwardFt: calibration.sprayForwardFt,
        sprayRightFt: calibration.sprayRightFt,
        schemaVersion: calibration.schemaVersion,
      }), calibrationCommandId);
      this.requireAck(calibrationAck, [0, 1], calibration.id);

      const rectangleCommandId = this.takeCommandId();
      const flags = (definition.side === 'left' ? 1 : 0) |
        (this.dryMode ? 2 : 0) |
        (this.preferForwardOnly ? 4 : 0);
      const rectangleAck = await this.sendAndWait(buildRectangleV2({
        flags,
        epoch: this.epoch,
        commandId: rectangleCommandId,
        mFt: definition.mFt,
        nFt: definition.nFt,
        startClearFt: definition.startClearFt,
        endClearFt: definition.endClearFt,
        calibrationId: calibration.id,
      }), rectangleCommandId);
      this.requireAck(rectangleAck, [1], calibration.id);
      this.currentState = 'configured';
      return rectangleAck;
    });
  }

  async arm(): Promise<AckV2> {
    if (this.currentState !== 'configured') throw new Error('mission must be configured before arm');
    if (!this.hasFreshPose()) throw new Error('a fresh valid pose is required before arm');
    return this.command(1, [2], 'armed');
  }

  async start(): Promise<AckV2> {
    if (this.currentState !== 'armed') throw new Error('mission must be armed before start');
    return this.command(2, [3], 'running');
  }

  async stop(): Promise<AckV2> {
    let acknowledgement: AckV2 | null = null;
    try {
      acknowledgement = await this.command(3, [0], 'idle', false);
      return acknowledgement;
    } finally {
      try {
        await this.transport.writeCompatibilityStop();
      } catch {
        // The v2 Stop result remains authoritative; this is a best-effort belt-and-suspenders Stop.
      }
    }
  }

  notifyDisconnect(): void {
    if (this.currentState !== 'idle') this.currentState = 'fault';
  }

  dispose(): void {
    this.unsubscribe();
  }

  private async command(
    opcode: number,
    expectedStates: number[],
    nextState: MissionClientState,
    validateCalibration = true,
  ): Promise<AckV2> {
    return this.exclusive(async () => {
      const commandId = this.takeCommandId();
      const ack = await this.sendAndWait(buildCommandV2({ opcode, epoch: this.epoch, commandId }), commandId);
      this.requireAck(ack, expectedStates, validateCalibration ? this.calibrationId : null);
      this.currentState = nextState;
      return ack;
    });
  }

  private async exclusive<T>(operation: () => Promise<T>): Promise<T> {
    if (this.busy) throw new Error('another mission command is pending');
    this.busy = true;
    try {
      return await operation();
    } catch (error) {
      this.currentState = 'fault';
      throw error;
    } finally {
      this.busy = false;
    }
  }

  private takeCommandId(): number {
    const id = this.nextCommandId;
    this.nextCommandId = this.nextCommandId === 0xffffffff ? 1 : this.nextCommandId + 1;
    return id;
  }

  private receiveAck(packet: Uint8Array): void {
    const ack = parseAckV2(packet);
    if (!ack || !this.pending || ack.epoch !== this.epoch || ack.commandId !== this.pending.commandId) return;
    if (this.pending.received) return;
    this.pending.received = ack;
    for (const listener of this.pending.listeners) listener(ack);
    this.pending.listeners.clear();
  }

  private async sendAndWait(packet: Uint8Array, commandId: number): Promise<AckV2> {
    this.pending = { commandId, received: null, listeners: new Set() };
    try {
      for (let attempt = 0; attempt <= this.retries; attempt += 1) {
        await this.transport.writeWithResponse(packet);
        const ack = await this.waitForAck();
        if (ack) return ack;
      }
      throw new Error(`ACK timeout for command ${commandId}`);
    } finally {
      this.pending = null;
    }
  }

  private waitForAck(): Promise<AckV2 | null> {
    if (!this.pending) return Promise.resolve(null);
    if (this.pending.received) return Promise.resolve(this.pending.received);
    return new Promise((resolve) => {
      const listener = (ack: AckV2) => {
        clearTimeout(timer);
        resolve(ack);
      };
      const timer = setTimeout(() => {
        this.pending?.listeners.delete(listener);
        resolve(null);
      }, this.timeoutMs);
      this.pending!.listeners.add(listener);
    });
  }

  private requireAck(ack: AckV2, states: number[], calibrationId: number | null): void {
    if (ack.state === 5 || ack.faultCode !== 0) throw new Error(`firmware fault ${ack.faultCode}`);
    if (!states.includes(ack.state)) throw new Error(`unexpected acknowledgement state ${ack.state}`);
    if (calibrationId !== null && ack.calibrationId !== calibrationId) {
      throw new Error('acknowledgement calibration is stale');
    }
  }
}
