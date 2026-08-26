export type TrackingState = 'normal' | 'limited' | 'notAvailable';

/**
 * Whether a pose is trustworthy enough to drive the rover from.
 * `limited` still yields usable ARKit poses (it is the normal state while
 * indoors or shortly after start), so only `notAvailable` withholds data.
 */
export function isPoseUsable(state: TrackingState): boolean {
  return state !== 'notAvailable';
}

export type PoseUpdatePayload = {
  x: number;
  y: number;
  heading: number;
  trackingState: TrackingState;
};
