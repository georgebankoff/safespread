import { NativeModule, requireNativeModule } from 'expo';

import { PoseUpdatePayload } from './ArkitPose.types';

declare class ArkitPoseModule extends NativeModule<{
  onPoseUpdate: (event: PoseUpdatePayload) => void;
}> {
  start(): void;
  stop(): void;
}

export default requireNativeModule<ArkitPoseModule>('ArkitPose');
