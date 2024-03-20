package frc.robot;

import static frc.robot.Constants.ArmConstants.ARM_LOCK_SOLENOID_CHANNEL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToAmplifier;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.VoltageCommandRamp;
import frc.robot.commands.arm_commands.ArmExtend;
import frc.robot.commands.arm_commands.ArmGoToPosTeleop;
import frc.robot.commands.arm_commands.ArmManuel;
import frc.robot.commands.arm_commands.ArmRetract;
import frc.robot.commands.arm_commands.ArmSetTargetPos;
import frc.robot.commands.climb_commands.ClimbManual;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONAVX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMaxSwerve;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.example_flywheel.ExampleFlywheel;
import frc.robot.subsystems.example_flywheel.ExampleFlywheelIO;
import frc.robot.subsystems.example_flywheel.ExampleFlywheelIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionReal;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Intake intake;
  private final ExampleFlywheel exampleFlywheel;
  private final AprilTagVision vision;
  private final Climb climb;

  public final Solenoid armLock;

  // shuffleboard
  ShuffleboardTab lewZealandTab;
  public static GenericEntry hasNote;
  public static GenericEntry leftClimbHeight;
  public static GenericEntry rightClimbHeight;

  // Controller
  public static CommandXboxController driveController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  // Auto Commands
  private final AutoCommands autoCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        armLock = new Solenoid(PneumaticsModuleType.CTREPCM, ARM_LOCK_SOLENOID_CHANNEL);
        armLock.set(false);
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONAVX(),
                new ModuleIOMaxSwerve(0),
                new ModuleIOMaxSwerve(1),
                new ModuleIOMaxSwerve(2),
                new ModuleIOMaxSwerve(3));
        // We have no flywheel, so create a simulated just for example.
        exampleFlywheel = new ExampleFlywheel(new ExampleFlywheelIOSim());
        arm = new Arm(new ArmIOReal());
        intake = new Intake(new IntakeIOReal());
        vision = new AprilTagVision(new AprilTagVisionIO() {});
        climb = new Climb(new ClimbIOReal());
        break;

      case ROBOT_SIM:
        armLock = null;
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        exampleFlywheel = new ExampleFlywheel(new ExampleFlywheelIOSim());
        arm = new Arm(new ArmIOSim());
        intake = new Intake(new IntakeIOSim());
        vision = new AprilTagVision(new AprilTagVisionIO() {});
        climb = new Climb(new ClimbIOSim());
        break;

      case ROBOT_FOOTBALL:
        armLock = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        exampleFlywheel = new ExampleFlywheel(new ExampleFlywheelIOSim());
        arm = new Arm(new ArmIOSim());
        intake = new Intake(new IntakeIOSim());
        vision =
            new AprilTagVision(
                new AprilTagVisionReal(
                    Constants.VisionConstants.cameraName, Constants.RobotConstants.cameraToRobot));
        climb = new Climb(new ClimbIOSim());

        break;

        //   case ROBOT_REPLAY:
      default:
        armLock = null;
        // Replayed robot, disable IO implementations since the replay
        // will supply the data.
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        exampleFlywheel = new ExampleFlywheel(new ExampleFlywheelIO() {});
        arm = new Arm(new ArmIOSim()); // TODO: make interfaces
        intake = new Intake(new IntakeIOSim());
        vision = new AprilTagVision(new AprilTagVisionIO() {});
        climb = new Climb(new ClimbIOSim());

        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> exampleFlywheel.runVelocity(flywheelSpeedInput.get()),
                exampleFlywheel::stop,
                exampleFlywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Create auto commands
    autoCommands = new AutoCommands(drive);

    autoChooser.addOption(
        "Drive forward 2.5 m",
        autoCommands.driveToPoseRelative(new Pose2d(2.5, 0, new Rotation2d())));
    autoChooser.addOption("Score Four", autoCommands.amplifierScoreFour());
    autoChooser.addOption("auto field test", autoCommands.autoTest());
    autoChooser.addOption("Drive to note", autoCommands.driveToNote());
    autoChooser.addOption("drivenote pickup", autoCommands.driveNotePickup());
    autoChooser.addOption("Drive To Amplifier", new DriveToAmplifier(drive));

    // add testing auto functions
    addTestingAutos();

    // Configure the button bindings
    configureButtonBindings();

    initShuffleboard();
  }

  private void initShuffleboard() {
    // Configure the Shuffleboard
    lewZealandTab = Shuffleboard.getTab("Lew Zealand");
    hasNote = lewZealandTab.add("Has Note", false).getEntry();
    leftClimbHeight = lewZealandTab.add("Left Climb", 0.0).getEntry();
    rightClimbHeight = lewZealandTab.add("Right Climb", 0.0).getEntry();
  }

  public void updateShuffleboard() {
    hasNote.setBoolean(intake.hasNote());
    leftClimbHeight.setDouble(climb.getLeftEncoder());
    rightClimbHeight.setDouble(climb.getRightEncoder());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
            () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
            () -> -driveController.getRightX() * 0.7));
    driveController.start().onTrue(new InstantCommand(() -> drive.resetFieldHeading()));
    driveController
        .leftTrigger(0.9)
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX()));

    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 180,
                () -> drive.getYaw()));
    driveController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 0,
                () -> drive.getYaw()));
    driveController
        .x()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 90,
                () -> drive.getYaw()));
    driveController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> -90,
                () -> drive.getYaw()));

    driveController.povDown().whileTrue(new DriveToAmplifier(drive));

    // intake/outtake
    driveController.rightTrigger().whileTrue(intake.runEatCommand());
    operatorController.leftTrigger().whileTrue(intake.runVomitCommand());
    operatorController.rightTrigger().whileTrue(intake.runPoopCommand());

    // Arm Controls
    arm.setDefaultCommand(new ArmGoToPosTeleop(arm));
    new Trigger(() -> Math.abs(operatorController.getRightY()) >= Constants.JOYSTICK_DEADBAND)
        .whileTrue(new ArmManuel(arm, () -> -operatorController.getRightY()));

    operatorController.a().onTrue(new ArmSetTargetPos(arm, ArmConstants.armIntakePosDeg));
    operatorController.b().onTrue(new ArmSetTargetPos(arm, ArmConstants.armDrivePosDeg));
    operatorController.x().onTrue(new ArmSetTargetPos(arm, ArmConstants.armTrapPosDeg));
    operatorController.y().onTrue(new ArmSetTargetPos(arm, ArmConstants.armAmpPosDeg));

    operatorController.back().onTrue(new ArmExtend(arm));
    operatorController.start().onTrue(new ArmRetract(arm));

    // TEMPORARY!!! FOR TESTING. TODO: REMOVE THIS!!!
    climb.setDefaultCommand(
        new ClimbManual(
            climb,
            () ->
                MathUtil.applyDeadband(
                    -operatorController.getLeftY() * 1.0 // input is voltage so 12 is maximum power
                    ,
                    Constants.JOYSTICK_DEADBAND)));

    operatorController.start().onTrue(new ArmExtend(arm));
    operatorController.back().onTrue(new ArmRetract(arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void addTestingAutos() {
    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            exampleFlywheel,
            exampleFlywheel::runVolts,
            exampleFlywheel::getCharacterizationVelocity));

    autoChooser.addOption(
        "Module Drive Ramp Test",
        new VoltageCommandRamp(drive, drive::runDriveCommandRampVolts, 0.5, 5.0));
    autoChooser.addOption(
        "Module Turn Ramp Test",
        new VoltageCommandRamp(drive, drive::runTurnCommandRampVolts, 0.5, 5.0));
  }
}
