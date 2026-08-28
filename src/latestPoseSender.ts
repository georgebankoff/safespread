export interface PoseTransport {
  writePose(packet: Uint8Array): Promise<void>;
}

export class LatestPoseSender {
  private active: Promise<void> | null = null;
  private pending: Uint8Array | null = null;
  private stopping = false;
  private failure: Error | null = null;
  private droppedCount = 0;

  constructor(
    private readonly transport: PoseTransport,
    private readonly onError: (error: Error) => void = () => {},
  ) {}

  get dropped(): number {
    return this.droppedCount;
  }

  offer(packet: Uint8Array): void {
    if (this.stopping) throw new Error('pose sender is stopped');
    if (this.failure) throw this.failure;
    const owned = packet.slice();
    if (this.active) {
      if (this.pending) this.droppedCount += 1;
      this.pending = owned;
      return;
    }
    this.start(owned);
  }

  async stop(): Promise<void> {
    this.stopping = true;
    while (this.active) await this.active;
    if (this.failure) throw this.failure;
  }

  private start(packet: Uint8Array): void {
    const task = this.drain(packet);
    this.active = task;
    void task.then(() => {
      if (this.active === task) this.active = null;
    });
  }

  private async drain(first: Uint8Array): Promise<void> {
    let current: Uint8Array | null = first;
    while (current) {
      try {
        await this.transport.writePose(current);
      } catch (value) {
        this.failure = value instanceof Error ? value : new Error(String(value));
        if (this.pending) {
          this.droppedCount += 1;
          this.pending = null;
        }
        this.onError(this.failure);
        return;
      }
      current = this.pending;
      this.pending = null;
    }
  }
}
