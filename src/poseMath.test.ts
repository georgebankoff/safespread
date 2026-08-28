import { DEFAULT_MOUNT_CALIBRATION } from './hardwareGeometry';
import { applyOrigin, cameraToRover, Pose, roverToSprayBar } from './poseMath';

const baseCalibration = {
  id: 1,
  schemaVersion: 1,
  cameraForwardFt: 0,
  cameraRightFt: 0,
  cameraYawDeg: 0,
  sprayForwardFt: 0,
  sprayRightFt: 0,
};

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

describe('cameraToRover', () => {
  it('leaves pose unchanged when mounting offsets are zero', () => {
    const camera: Pose = { x: 4, y: -2, heading: 35 };
    expect(cameraToRover(camera, baseCalibration)).toEqual(camera);
  });

  it.each([
    [{ x: 0, y: 1, heading: 0 }, { x: 0, y: 0, heading: 0 }],
    [{ x: 1, y: 0, heading: 90 }, { x: 0, y: 0, heading: 90 }],
    [{ x: 0, y: -1, heading: 180 }, { x: 0, y: 0, heading: 180 }],
  ])('removes a one-foot forward camera offset at cardinal headings', (camera, expected) => {
    const rover = cameraToRover(camera, { ...baseCalibration, cameraForwardFt: 1 });
    expect(rover.x).toBeCloseTo(expected.x);
    expect(rover.y).toBeCloseTo(expected.y);
    expect(rover.heading).toBeCloseTo(expected.heading);
  });

  it('subtracts mounting yaw and wraps across zero', () => {
    expect(
      cameraToRover(
        { x: 0, y: 0, heading: 1 },
        { ...baseCalibration, cameraYawDeg: 3 },
      ).heading,
    ).toBeCloseTo(358);
  });
});

describe('roverToSprayBar', () => {
  it('applies forward and right offsets at the chassis heading', () => {
    const spray = roverToSprayBar(
      { x: 10, y: 5, heading: 90 },
      { ...baseCalibration, sprayForwardFt: -1, sprayRightFt: 0.5 },
    );
    expect(spray.x).toBeCloseTo(9);
    expect(spray.y).toBeCloseTo(4.5);
    expect(spray.heading).toBe(90);
  });

  it('uses the measured mount to locate the rear axle and spray bar by default', () => {
    const rover = cameraToRover(
      { x: 0, y: 1.75, heading: 0 },
      DEFAULT_MOUNT_CALIBRATION,
    );
    const spray = roverToSprayBar(rover, DEFAULT_MOUNT_CALIBRATION);

    expect(rover.x).toBeCloseTo(0);
    expect(rover.y).toBeCloseTo(0);
    expect(spray.x).toBeCloseTo(0);
    expect(spray.y).toBeCloseTo(-2.5 / 12);
  });
});
