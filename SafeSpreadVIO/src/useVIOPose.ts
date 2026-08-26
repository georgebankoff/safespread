import { useCallback, useEffect, useState } from 'react';
import ArkitPoseModule from '../modules/arkit-pose/src/ArkitPoseModule';
import { applyOrigin, Pose } from './poseMath';

export function useVIOPose() {
  const [raw, setRaw] = useState<Pose>({ x: 0, y: 0, heading: 0 });
  const [trackingOk, setTrackingOk] = useState(false);
  const [origin, setOrigin] = useState<Pose | null>(null);

  useEffect(() => {
    ArkitPoseModule.start();
    const subscription = ArkitPoseModule.addListener('onPoseUpdate', (event) => {
      setRaw({ x: event.x, y: event.y, heading: event.heading });
      setTrackingOk(event.trackingState === 'normal');
    });
    return () => {
      subscription.remove();
      ArkitPoseModule.stop();
    };
  }, []);

  const zero = useCallback(() => {
    setOrigin(raw);
  }, [raw]);

  return { pose: applyOrigin(raw, origin), trackingOk, zero };
}
