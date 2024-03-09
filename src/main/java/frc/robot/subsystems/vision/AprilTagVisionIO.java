package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {

  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d estimatedPose = new Pose3d(new Pose2d(0, 0, new Rotation2d(0.0)));
    public double timestamp = 0.0;
    public int bestID = -1;
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
  ;
}
