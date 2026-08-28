export type TrackingState = 'normal' | 'limited' | 'notAvailable';
export type TrackingReason =
  | 'none'
  | 'initializing'
  | 'excessiveMotion'
  | 'insufficientFeatures'
  | 'relocalizing'
  | 'interrupted'
  | 'sessionFailed'
  | 'unknown';
export type MappingStatus = 'notAvailable' | 'limited' | 'extending' | 'mapped' | 'unknown';

/**
 * Whether a pose is trustworthy enough to drive the rover from.
 * Autonomous motion requires ARKit's strongest tracking state. A limited pose
 * remains useful for UI diagnostics but is never eligible to drive the rover.
 */
export function isPoseUsable(state: TrackingState): boolean {
  return state === 'normal';
}

type TrackingMetadata = {
  frameTimestampMs: number;
  emittedTimestampMs: number;
  sequence: number;
  trackingState: TrackingState;
  trackingReason: TrackingReason;
  mappingStatus: MappingStatus;
};

export type PoseUpdatePayload = TrackingMetadata & {
  kind: 'pose';
  x: number;
  y: number;
  heading: number;
} | TrackingMetadata & {
  kind: 'status';
  error?: string;
};
