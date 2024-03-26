package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: rework intake and outtake for the coulson wheels

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.1);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.5);

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private final SimpleMotorFeedforward ffModel;

  private static double setpointRPMs = 0.0;
  private static double setpointRadPerSec = 0.0;

  private static enum IntakeMode {
    kStopped,
    kEating,
    kVomiting,
    kPooping
  };

  private IntakeMode mode = IntakeMode.kStopped;

  public Intake(IntakeIO io) {
    System.out.println("[Init] Creating Intake");
    this.io = io;
    io.setBrakeMode(false);

    controller.setPID(kP.get(), kI.get(), 0.0);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.1, 0.12);
        break;
      case ROBOT_SIM:
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.09);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      controller.setPID(kP.get(), kI.get(), 0.0);
      controller.reset();
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      controller.reset();
      mode = IntakeMode.kStopped;
    } else {
      switch (mode) {
        case kEating:
          setpointRPMs = Constants.IntakeConstants.kEatRPM;
          break;
        case kVomiting:
          setpointRPMs = Constants.IntakeConstants.kVomitRPM;
          break;
        case kPooping:
          setpointRPMs = Constants.IntakeConstants.kPoopRPM;
          break;
        case kStopped:
          setpointRPMs = 0.0;
      }

      setpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(setpointRPMs);
      controller.setSetpoint(setpointRadPerSec);
      double voltage = ffModel.calculate(setpointRadPerSec);
      // + controller.calculate(inputs.velocityRadPerSec);
      io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }
  }

  public boolean hasNote() {
    return inputs.hasNote;
  }

  public boolean hasNoNote() {
    return !inputs.hasNote;
  }

  public void eat() {
    mode = IntakeMode.kEating;
  }

  public void vomit() {
    mode = IntakeMode.kVomiting;
  }

  public void poop() {
    mode = IntakeMode.kPooping;
  }

  public void stop() {
    System.out.println("Intake: Stopped");
    mode = IntakeMode.kStopped;
  }

  @AutoLogOutput
  public double getSetpointRadians() {
    return setpointRadPerSec;
  }

  @AutoLogOutput
  public double getFF() {
    return ffModel.calculate(setpointRadPerSec);
  }

  @AutoLogOutput
  public double getPID() {
    return controller.calculate(inputs.velocityRadPerSec);
  }

  @AutoLogOutput
  public IntakeMode getState() {
    return mode;
  }

  @AutoLogOutput
  public double getSetPointRPMs() {
    return setpointRPMs;
  }

  @AutoLogOutput
  public double getVelocityRPMs() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public Command runEatCommand() {
    // run eat mode until a note is obtained
    return new FunctionalCommand(
        () -> System.out.println("Intake: Eat"),
        () -> this.eat(),
        interrupted -> this.stop(),
        this::hasNote,
        this);
  }

  public Command runVomitCommand() {
    // Run vomit until a note is expelled.

    // run a sequence that runs vomit mode until note is moved off sensor,
    // then keep running motors for a second to keep advancing,
    // then stop.
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Intake: Vomit"), this),
            new RunCommand(() -> this.vomit(), this).unless(this::hasNoNote).until(this::hasNoNote),
            new WaitCommand(IntakeConstants.kVomitDelay) // run a bit more to advance the note
            )
        .finallyDo(() -> this.stop());
  }

  public Command runPoopCommand() {
    // Run eat until a note is expelled.

    // run a sequence that runs poop mode until note is moved off sensor,
    // then keep running motors for a second to keep advancing,
    // then stop.
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Intake: Poop"), this),
            new RunCommand(() -> this.poop(), this).unless(this::hasNoNote).until(this::hasNoNote),
            new WaitCommand(IntakeConstants.kVomitDelay) // run a bit more to advance the note
            )
        .finallyDo(() -> this.stop());
  }
}
