import { useCallback, useEffect, useState } from 'react';
import ArkitPoseModule from '../modules/arkit-pose/src/ArkitPoseModule';
import { isPoseUsable, TrackingState } from '../modules/arkit-pose/src/ArkitPose.types';
import { applyOrigin, Pose } from './poseMath';

export function useVIOPose() {
  const [raw, setRaw] = useState<Pose>({ x: 0, y: 0, heading: 0 });
  const [trackingState, setTrackingState] = useState<TrackingState>('notAvailable');
  const [origin, setOrigin] = useState<Pose | null>(null);

  useEffect(() => {
    ArkitPoseModule.start();
    const subscription = ArkitPoseModule.addListener('onPoseUpdate', (event) => {
      if (event.kind === 'pose') {
        setRaw({ x: event.x, y: event.y, heading: event.heading });
      }
      setTrackingState(event.trackingState);
    });
    return () => {
      subscription.remove();
      ArkitPoseModule.stop();
    };
  }, []);

  const zero = useCallback(() => {
    setOrigin(raw);
  }, [raw]);

  return {
    pose: applyOrigin(raw, origin),
    trackingState,
    trackingOk: isPoseUsable(trackingState),
    zero,
  };
}
