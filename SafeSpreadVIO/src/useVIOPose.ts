import { useEffect, useMemo, useRef, useState } from 'react';
import ArkitPoseModule from '../modules/arkit-pose/src/ArkitPoseModule';
import {
  isPoseUsable,
  MappingStatus,
  PoseUpdatePayload,
  TrackingReason,
  TrackingState,
} from '../modules/arkit-pose/src/ArkitPose.types';
import { MountCalibration } from './poseMath';
import { PoseDecision, PosePipeline, PoseReadiness, ValidatedPose } from './posePipeline';

export const DEFAULT_MOUNT_CALIBRATION: MountCalibration = {
  id: 0,
  schemaVersion: 1,
  cameraForwardFt: 0,
  cameraRightFt: 0,
  cameraYawDeg: 0,
  sprayForwardFt: 0,
  sprayRightFt: 0,
};

function monotonicNow(): number {
  return globalThis.performance?.now() ?? Date.now();
}

export function useVIOPose(calibration: MountCalibration = DEFAULT_MOUNT_CALIBRATION) {
  const pipeline = useMemo(
    () => new PosePipeline(calibration),
    [
      calibration.id,
      calibration.schemaVersion,
      calibration.cameraForwardFt,
      calibration.cameraRightFt,
      calibration.cameraYawDeg,
      calibration.sprayForwardFt,
      calibration.sprayRightFt,
    ],
  );
  const [rawEvent, setRawEvent] = useState<PoseUpdatePayload | null>(null);
  const [validatedPose, setValidatedPose] = useState<ValidatedPose | null>(null);
  const [latestDecision, setLatestDecision] = useState<PoseDecision>({
    ok: false,
    reason: 'tracking',
  });
  const [trackingState, setTrackingState] = useState<TrackingState>('notAvailable');
  const [trackingReason, setTrackingReason] = useState<TrackingReason>('unknown');
  const [mappingStatus, setMappingStatus] = useState<MappingStatus>('notAvailable');
  const [readiness, setReadiness] = useState<PoseReadiness>({ ready: false, reason: 'noSamples' });
  const [fresh, setFresh] = useState(false);
  const latestPoseRef = useRef<ValidatedPose | null>(null);

  useEffect(() => {
    ArkitPoseModule.start();
    const subscription = ArkitPoseModule.addListener('onPoseUpdate', (event) => {
      const receivedAtMs = monotonicNow();
      const decision = pipeline.ingest(event, receivedAtMs);
      setRawEvent(event);
      setTrackingState(event.trackingState);
      setTrackingReason(event.trackingReason);
      setMappingStatus(event.mappingStatus);
      setLatestDecision(decision);
      if (decision.ok) {
        latestPoseRef.current = decision.pose;
        setValidatedPose(decision.pose);
        setFresh(true);
      } else {
        setFresh(false);
      }
      setReadiness(pipeline.readiness(receivedAtMs));
    });
    return () => {
      subscription.remove();
      ArkitPoseModule.stop();
      pipeline.reset();
      latestPoseRef.current = null;
    };
  }, [pipeline]);

  useEffect(() => {
    const timer = setInterval(() => {
      const nowMs = monotonicNow();
      const latest = latestPoseRef.current;
      setFresh(Boolean(latest && latest.captureAgeMs + nowMs - latest.receivedAtMs <= 250));
      setReadiness(pipeline.readiness(nowMs));
    }, 50);
    return () => clearInterval(timer);
  }, [pipeline]);

  return {
    pose: validatedPose?.rover ?? null,
    validatedPose,
    latestDecision,
    rawEvent,
    trackingState,
    trackingReason,
    mappingStatus,
    trackingOk: fresh && isPoseUsable(trackingState) && latestDecision.ok,
    readiness,
  };
}
