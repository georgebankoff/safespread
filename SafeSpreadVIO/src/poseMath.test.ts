import { applyOrigin, Pose } from './poseMath';

describe('applyOrigin', () => {
  it('returns zero pose when no origin is set', () => {
    expect(applyOrigin({ x: 5, y: 5, heading: 90 }, null)).toEqual({ x: 0, y: 0, heading: 0 });
  });

  it('returns zero pose when raw equals origin', () => {
    const p: Pose = { x: 3, y: 4, heading: 45 };
    const result = applyOrigin(p, p);
    expect(result.x).toBeCloseTo(0);
    expect(result.y).toBeCloseTo(0);
    expect(result.heading).toBeCloseTo(0);
  });

  it('rotates displacement into the origin heading frame', () => {
    // Origin facing 90° (world +X). Raw is 1 unit further along world +X,
    // i.e. straight ahead of where the origin was facing.
    const origin: Pose = { x: 0, y: 0, heading: 90 };
    const raw: Pose = { x: 1, y: 0, heading: 90 };
    const result = applyOrigin(raw, origin);
    expect(result.x).toBeCloseTo(0);
    expect(result.y).toBeCloseTo(1);
    expect(result.heading).toBeCloseTo(0);
  });

  it('wraps heading difference correctly across the 0/360 boundary', () => {
    const origin: Pose = { x: 0, y: 0, heading: 350 };
    const raw: Pose = { x: 0, y: 0, heading: 10 };
    const result = applyOrigin(raw, origin);
    expect(result.heading).toBeCloseTo(20);
  });
});
