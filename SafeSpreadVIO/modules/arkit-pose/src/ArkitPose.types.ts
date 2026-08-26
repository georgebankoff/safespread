export type TrackingState = 'normal' | 'limited' | 'notAvailable';

export type PoseUpdatePayload = {
  x: number;
  y: number;
  heading: number;
  trackingState: TrackingState;
};
