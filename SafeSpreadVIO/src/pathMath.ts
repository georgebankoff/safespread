export interface PathPoint {
  x: number;
  y: number;
  spraying: boolean;
}

/** Only keep a sample once the rover has actually moved, so a mission of any
 *  length stays bounded, but always keep one where spray switches state so the
 *  sprayed segments are not smeared across the transition. */
export const MIN_SAMPLE_STEP_FT = 0.15;
export const MAX_PATH_POINTS = 3000;

export function shouldRecord(
  last: PathPoint | undefined,
  next: PathPoint
): boolean {
  if (!last) return true;
  if (last.spraying !== next.spraying) return true;
  const dx = next.x - last.x;
  const dy = next.y - last.y;
  return dx * dx + dy * dy >= MIN_SAMPLE_STEP_FT * MIN_SAMPLE_STEP_FT;
}

export interface ViewBox {
  minX: number;
  minY: number;
  spanX: number;
  spanY: number;
}

/** Fit the rectangle and every point travelled -- including excursions into
 *  the headland -- into one box, so nothing is drawn off-canvas. */
export function computeViewBox(
  widthFt: number,
  lengthFt: number,
  points: PathPoint[],
  marginFt = 1
): ViewBox {
  let minX = 0;
  let maxX = widthFt;
  let minY = 0;
  let maxY = lengthFt;

  for (const p of points) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
  }

  minX -= marginFt;
  maxX += marginFt;
  minY -= marginFt;
  maxY += marginFt;

  return {
    minX,
    minY,
    spanX: Math.max(maxX - minX, 0.001),
    spanY: Math.max(maxY - minY, 0.001),
  };
}

/** Map field feet to canvas pixels, preserving aspect ratio and flipping Y
 *  (field +Y is away from the operator; screen +Y goes down). */
export function projector(box: ViewBox, canvasW: number, canvasH: number) {
  const scale = Math.min(canvasW / box.spanX, canvasH / box.spanY);
  const offsetX = (canvasW - box.spanX * scale) / 2;
  const offsetY = (canvasH - box.spanY * scale) / 2;

  return {
    scale,
    toPx(xFt: number, yFt: number) {
      return {
        left: offsetX + (xFt - box.minX) * scale,
        top: offsetY + (box.spanY - (yFt - box.minY)) * scale,
      };
    },
  };
}
