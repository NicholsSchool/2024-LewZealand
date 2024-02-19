package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommands {
  // Subsystems
  private final Drive drive;

  public static final double allianceNoteX = 2.218;
  public static final Translation2d note1Translation = new Translation2d(allianceNoteX, 6.908);
  public static final Translation2d note2Translation = new Translation2d(allianceNoteX, 5.513);
  public static final Translation2d note3Translation = new Translation2d(allianceNoteX, 4.049);

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
        driveToPose(new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(-90.0))),
        driveToPose(new Pose2d(note1Translation, Rotation2d.fromDegrees(0.0))),
        driveToPose(new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(-90.0))),
        driveToPose(new Pose2d(note2Translation, Rotation2d.fromDegrees(0.0))),
        driveToPose(new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(-90.0))),
        driveToPose(new Pose2d(note3Translation, Rotation2d.fromDegrees(0.0))),
        driveToPose(
            new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(-90.0))));
  }
}
