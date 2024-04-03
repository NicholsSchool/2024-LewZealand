package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.StagingLocations;
import frc.robot.commands.arm_commands.ArmGoToPosAuto;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class AutoCommands {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Intake intake;
  private final Outtake outtake;

  private final LoggedDashboardNumber autoDelaySeconds =
      new LoggedDashboardNumber("Autonomous Time Delay", 0.25);

  public AutoCommands(Drive drive, Arm arm, Intake intake, Outtake outtake) {
    this.drive = drive;
    this.arm = arm;
    this.intake = intake;
    this.outtake = outtake;
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

  public Command splineToPose(Pose2d pose) {
    var splToPose =
        new SplineToPose(
            this.drive,
            () -> {
              return pose;
            });
    return splToPose.until(splToPose::atGoal);
  }

  /**
   * Score amp using positions relative to the robot starting position of 0.0.
   *
   * @param isBlue
   * @return
   */
  public Command scoreAmpRelative(boolean isBlue) {
    double scoringAngle = isBlue ? 270.0 : 90.0;
    double nudgeComponent = isBlue ? 0.25 : -0.25;
    double horizontalComponent = isBlue ? -0.25 : 0.25;

    var relativeDriveCommand =
        driveToPoseRelative(
                new Pose2d(
                    Units.inchesToMeters(60.0), 0.0, new Rotation2d(Math.toRadians(scoringAngle))))
            .withTimeout(4.0);

    var raiseArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armAmpPosDeg).withTimeout(3.0);

    var adjustCommand =
        driveToPoseRelative(
                new Pose2d(
                    Units.inchesToMeters(60.0),
                    nudgeComponent,
                    new Rotation2d(Math.toRadians(scoringAngle))))
            .withTimeout(2.0);

    var poopCommand = intake.runPoopCommand().withTimeout(1.0);

    var lowerArmCommand = new ArmGoToPosAuto(arm, ArmConstants.armDrivePosDeg).withTimeout(3.0);

    var crossLineCommand =
        driveToPoseRelative(
                new Pose2d(
                    Units.inchesToMeters(120.0),
                    horizontalComponent,
                    new Rotation2d(Math.toRadians(0.0))))
            .withTimeout(4.0);

    return new SequentialCommandGroup(
        relativeDriveCommand,
        raiseArmCommand,
        adjustCommand,
        poopCommand,
        lowerArmCommand,
        crossLineCommand);
  }

  private Command DriveToAmplifierWithFudge(Drive drive) {
    // add fudge factors (in inches) to the amplifier scoring position to
    // accommodate inaccuracies in field layout.
    return new DriveToAmplifier(drive, 6, 0);
  }

  public Command TenFootTest(Drive drive) {
    return new DriveToPose(drive, new Pose2d(new Translation2d(3.048, 0), new Rotation2d(0)));
  }

  public Command scoreAmpField() {
    // 1) drive to the amp while raising the arm.
    // 2) stuff note into amp
    return new SequentialCommandGroup(
        new ArmGoToPosAuto(arm, ArmConstants.armStartPosDeg)
            .withTimeout(2), // raise arm release brake.
        new WaitCommandTunable(
            () ->
                autoDelaySeconds
                    .get()), // wait for tunable amount of time to allow alliance members to move
        DriveToAmplifierWithFudge(drive).withTimeout(2), // drive to amp scoring position
        waitSeconds(1), // wait?? not sure why.
        // new ArmGoToPosAuto(arm, ArmConstants.armAmpPosDeg).withTimeout(2), // arm at amp score
        // positions
        new ParallelCommandGroup(
            outtake.runAmpCommand(),
            intake.runDigestCommand())); // run intake and outtake at same time to score
  }

  public Command scoreAmpFieldAndCross() {
    var crossLineCommand =
        driveToPose(
                new Pose2d(
                    FieldConstants.StagingLocations.spikeTranslations[2],
                    new Rotation2d(Math.toRadians(135 - 180))))
            .withTimeout(3.0);

    // 1) drive to the amp while raising the arm.
    // 2) stuff note into amp
    // 4) raise arm and cross line
    return new SequentialCommandGroup(
        scoreAmpField(),
        new ParallelCommandGroup(
            new ArmGoToPosAuto(arm, ArmConstants.armDrivePosDeg), crossLineCommand));
  }

  public Command scoreAmpAndNotePickupField() {
    var crossLineCommand =
        driveToPose(
                new Pose2d(
                    FieldConstants.StagingLocations.spikeTranslations[2],
                    new Rotation2d(Math.toRadians(-45.0))))
            .withTimeout(3.0);

    // 1) drive to the amp while raising the arm.
    // 2) stuff note into amp
    // 3) drive to note while lowering arm and intaking.
    // 4) raise arm
    return new SequentialCommandGroup(
        scoreAmpField(),
        new ArmGoToPosAuto(arm, ArmConstants.armIntakePosDeg),
        new ParallelCommandGroup(intake.runEatCommand(), crossLineCommand),
        new ArmGoToPosAuto(arm, ArmConstants.armDrivePosDeg));
  }

  public Command scoreAmpAndNotePickupScoreField() {
    // 1) score amp and pickup next note
    // 2) score note in amp and cross field
    return new SequentialCommandGroup(scoreAmpAndNotePickupField(), scoreAmpFieldAndCross());
  }

  public Command scoreAmpRelativeBlue() {
    return new SequentialCommandGroup(
        new WaitCommandTunable(() -> autoDelaySeconds.get()), scoreAmpRelative(true));
  }

  public Command scoreAmpRelativeRed() {
    return new SequentialCommandGroup(
        new WaitCommandTunable(() -> autoDelaySeconds.get()), scoreAmpRelative(false));
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
        new DriveToAmplifier(drive, 0, 0),
        driveToPose(new Pose2d(StagingLocations.spikeTranslations[0], Rotation2d.fromDegrees(0.0))),
        new DriveToAmplifier(drive, 0, 0),
        driveToPose(new Pose2d(StagingLocations.spikeTranslations[1], Rotation2d.fromDegrees(0.0))),
        new DriveToAmplifier(drive, 0, 0),
        driveToPose(new Pose2d(StagingLocations.spikeTranslations[2], Rotation2d.fromDegrees(0.0))),
        new DriveToAmplifier(drive, 0, 0));
  }
}
