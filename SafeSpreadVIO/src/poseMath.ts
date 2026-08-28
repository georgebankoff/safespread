export interface Pose {
  x: number;
  y: number;
  heading: number; // degrees, 0-360
}

export interface MountCalibration {
  id: number;
  schemaVersion: number;
  cameraForwardFt: number;
  cameraRightFt: number;
  cameraYawDeg: number;
  sprayForwardFt: number;
  sprayRightFt: number;
}

export function normalizeHeading(heading: number): number {
  return ((heading % 360) + 360) % 360;
}

export function wrappedHeadingDelta(to: number, from: number): number {
  const delta = normalizeHeading(to) - normalizeHeading(from);
  return ((delta + 540) % 360) - 180;
}

function localOffsetToWorld(rightFt: number, forwardFt: number, headingDeg: number) {
  const theta = (headingDeg * Math.PI) / 180;
  return {
    x: rightFt * Math.cos(theta) + forwardFt * Math.sin(theta),
    y: -rightFt * Math.sin(theta) + forwardFt * Math.cos(theta),
  };
}

export function cameraToRover(camera: Pose, calibration: MountCalibration): Pose {
  const chassisHeading = normalizeHeading(camera.heading - calibration.cameraYawDeg);
  const offset = localOffsetToWorld(
    calibration.cameraRightFt,
    calibration.cameraForwardFt,
    chassisHeading,
  );
  return {
    x: camera.x - offset.x,
    y: camera.y - offset.y,
    heading: chassisHeading,
  };
}

export function roverToSprayBar(rover: Pose, calibration: MountCalibration): Pose {
  const offset = localOffsetToWorld(
    calibration.sprayRightFt,
    calibration.sprayForwardFt,
    rover.heading,
  );
  return {
    x: rover.x + offset.x,
    y: rover.y + offset.y,
    heading: normalizeHeading(rover.heading),
  };
}

export function applyOrigin(raw: Pose, origin: Pose | null): Pose {
  if (!origin) return { x: 0, y: 0, heading: 0 };

  const dx = raw.x - origin.x;
  const dy = raw.y - origin.y;
  const theta = (origin.heading * Math.PI) / 180;
  const cos = Math.cos(theta);
  const sin = Math.sin(theta);

  const x = dx * cos - dy * sin;
  const y = dx * sin + dy * cos;
  const heading = normalizeHeading(raw.heading - origin.heading);

  return { x, y, heading };
}
