export interface Pose {
  x: number;
  y: number;
  heading: number; // degrees, 0-360
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
  let heading = raw.heading - origin.heading;
  heading = ((heading % 360) + 360) % 360;

  return { x, y, heading };
}
