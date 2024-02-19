package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class DriveToAmplifier extends DriveToPose {
  static int targetAprilTag =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() != Alliance.Blue
          ? 5
          : 6;

  public static final Pose2d amplifierScoringPose =
      new Pose2d(
          FieldConstants.aprilTags.getTagPose(targetAprilTag).get().getX(),
          FieldConstants.aprilTags.getTagPose(targetAprilTag).get().getY()
              - Units.inchesToMeters(Constants.RobotConstants.robotSideLengthInches / 2.0),
          Rotation2d.fromDegrees(90.0));

  /** Automatically drives to the amplifier. */
  public DriveToAmplifier(Drive drive) {
    super(
        drive,
        () -> {
          return AllianceFlipUtil.apply(amplifierScoringPose);
        });
  }
}
