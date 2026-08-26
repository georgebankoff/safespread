import ExpoModulesCore
import ARKit

public class ArkitPoseModule: Module {
  private var session: ARSession?
  private var delegate: ArkitSessionDelegate?

  public func definition() -> ModuleDefinition {
    Name("ArkitPose")

    Events("onPoseUpdate")

    Function("start") {
      let session = ARSession()
      let delegate = ArkitSessionDelegate { [weak self] payload in
        self?.sendEvent("onPoseUpdate", payload)
      }
      session.delegate = delegate

      let config = ARWorldTrackingConfiguration()
      config.worldAlignment = .gravity
      session.run(config)

      self.session = session
      self.delegate = delegate
    }

    Function("stop") {
      self.session?.pause()
      self.session = nil
      self.delegate = nil
    }
  }
}

private class ArkitSessionDelegate: NSObject, ARSessionDelegate {
  private let onUpdate: ([String: Any]) -> Void

  init(onUpdate: @escaping ([String: Any]) -> Void) {
    self.onUpdate = onUpdate
  }

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    let t = frame.camera.transform
    let metersToFeet = 3.28084

    // ARKit: camera looks down its local -Z. Our convention: y = forward
    // (world -Z), x = right (world +X), heading 0=+Y, 90=+X.
    let xFt = Double(t.columns.3.x) * metersToFeet
    let yFt = Double(-t.columns.3.z) * metersToFeet

    let forwardX = Double(-t.columns.2.x)
    let forwardZ = Double(-t.columns.2.z)
    var headingDeg = atan2(forwardX, -forwardZ) * 180.0 / Double.pi
    if headingDeg < 0 { headingDeg += 360 }

    let trackingState: String
    switch frame.camera.trackingState {
    case .normal: trackingState = "normal"
    case .limited: trackingState = "limited"
    case .notAvailable: trackingState = "notAvailable"
    }

    onUpdate([
      "x": xFt,
      "y": yFt,
      "heading": headingDeg,
      "trackingState": trackingState,
    ])
  }
}
