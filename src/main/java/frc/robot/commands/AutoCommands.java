package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.StagingLocations;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommands {
  // Subsystems
  private final Drive drive;

  public AutoCommands(Drive drive) {
    this.drive = drive;
  }

  public Command driveToPose(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              return AllianceFlipUtil.apply(pose);
            });
    return drvToPose.until(drvToPose::atGoal);
  }

  public Command driveToPoseRelative(Pose2d pose) {
    var drvToPose =
        new DriveToPose(
            this.drive,
            () -> {
              return pose;
            });
    return drvToPose.until(drvToPose::atGoal);
  }

  public Command autoTest() {
    return new AutoFieldTest(drive);
  }

  public Command driveToNote() {
    return new DriveToNote(drive);
  }

  public Command driveNotePickup() {
    return new DriveNotePickup(drive);
  }

  /** EXAMPLE!!!: Scores three game pieces on field-side (high cone, high cube, mid cube). */
  public Command amplifierScoreFour() {
    /*  return driveToPose(
    new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(180.0)));*/

    return sequence(
        // reset(startingPose),
        new DriveToAmplifier(drive),
        driveToPose(
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.spikeTranslations[0], Rotation2d.fromDegrees(0.0)))),
        new DriveToAmplifier(drive),
        driveToPose(
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.spikeTranslations[1], Rotation2d.fromDegrees(0.0)))),
        new DriveToAmplifier(drive),
        driveToPose(
            AllianceFlipUtil.apply(
                new Pose2d(StagingLocations.spikeTranslations[2], Rotation2d.fromDegrees(0.0)))),
        new DriveToAmplifier(drive));
  }
}
