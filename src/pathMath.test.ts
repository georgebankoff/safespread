import {
  computeViewBox,
  projector,
  shouldRecord,
  MIN_SAMPLE_STEP_FT,
} from './pathMath';

describe('shouldRecord', () => {
  it('records the first point', () => {
    expect(shouldRecord(undefined, { x: 0, y: 0, spraying: false })).toBe(true);
  });

  it('skips points that have barely moved', () => {
    const last = { x: 0, y: 0, spraying: false };
    expect(shouldRecord(last, { x: 0.01, y: 0, spraying: false })).toBe(false);
  });

  it('records once past the step threshold', () => {
    const last = { x: 0, y: 0, spraying: false };
    const far = { x: MIN_SAMPLE_STEP_FT + 0.01, y: 0, spraying: false };
    expect(shouldRecord(last, far)).toBe(true);
  });

  it('always records a spray transition, however small the move', () => {
    const last = { x: 0, y: 0, spraying: false };
    expect(shouldRecord(last, { x: 0.001, y: 0, spraying: true })).toBe(true);
  });
});

describe('computeViewBox', () => {
  it('covers the rectangle even with no path yet', () => {
    const box = computeViewBox(10, 20, [], 1);
    expect(box.minX).toBe(-1);
    expect(box.minY).toBe(-1);
    expect(box.spanX).toBeCloseTo(12);
    expect(box.spanY).toBeCloseTo(22);
  });

  it('expands to include headland excursions outside the rectangle', () => {
    const box = computeViewBox(10, 20, [{ x: -4, y: 25, spraying: false }], 1);
    expect(box.minX).toBeCloseTo(-5);
    expect(box.minY).toBeCloseTo(-1);
    expect(box.spanX).toBeCloseTo(16); // -5 .. 11
    expect(box.spanY).toBeCloseTo(27); // -1 .. 26
  });
});

describe('projector', () => {
  it('flips Y so the field origin is at the bottom of the canvas', () => {
    const box = computeViewBox(10, 10, [], 0);
    const p = projector(box, 100, 100);
    const origin = p.toPx(0, 0);
    const top = p.toPx(0, 10);
    expect(origin.top).toBeGreaterThan(top.top);
  });

  it('keeps aspect ratio square on a non-square canvas', () => {
    const box = computeViewBox(10, 10, [], 0);
    const p = projector(box, 200, 100);
    const a = p.toPx(0, 0);
    const b = p.toPx(10, 0);
    // 10ft across should map to the smaller dimension, not stretch to 200px
    expect(b.left - a.left).toBeCloseTo(100);
  });
});
