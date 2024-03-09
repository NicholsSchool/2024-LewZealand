package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionReal extends PhotonVisionSubsystemBase implements AprilTagVisionIO {

  public AprilTagVisionReal(String cameraName, Transform3d cameraToRobot) {
    super(cameraName, cameraToRobot);
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    if (this.hasTargets()) {
      PhotonTrackedTarget target = this.getBestTarget();
      inputs.estimatedPose = this.getBestPoseEstimate();
      inputs.bestID = target.getFiducialId();
      inputs.timestamp = super.getTargetTimestamp();
    }
  }

  public void periodic() {
    super.periodic();
  }

  /** Please check hasTargets() before this is called. */
  public PhotonTrackedTarget getBestTarget() {
    return super.getBestTarget();
  }

  @AutoLogOutput
  public Pose3d getBestPoseEstimate() {
    Pose3d robotPose = new Pose3d();
    PhotonTrackedTarget target;
    if (this.hasTargets()) {
      target = this.getBestTarget();
      robotPose =
          PhotonUtils.estimateFieldToRobotAprilTag(
              target.getBestCameraToTarget(),
              FieldConstants.aprilTags.getTagPose(target.getFiducialId()).get(),
              super.getCameraToRobotTransform());
    }
    return robotPose;
  }
}
