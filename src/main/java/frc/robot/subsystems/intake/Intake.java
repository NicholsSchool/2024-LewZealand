package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber eatVelocity =
      new LoggedTunableNumber("Intake/EatVelocityRPMs");
  private static final LoggedTunableNumber vomitVelocity =
      new LoggedTunableNumber("Intake/VomitVelocityRPMs");
  private static final LoggedTunableNumber digestVelocity =
      new LoggedTunableNumber("Intake/DigestVelocityRPMs");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private static enum IntakeMode {
    kStopped,
    kEating,
    kVomiting,
    kDigesting
  };

  private IntakeMode mode = IntakeMode.kStopped;

  // initialize tunable values
  static {
    eatVelocity.initDefault(0.0);
    vomitVelocity.initDefault(0.0);
    digestVelocity.initDefault(0.0);
    kP.initDefault(IntakeConstants.kP);
    kD.initDefault(IntakeConstants.kD);
  }

  public Intake(IntakeIO io) {
    System.out.println("[Init] Creating Intake");
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    double setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      controller.reset();
      mode = IntakeMode.kStopped;
    } else {
      switch (mode) {
        case kEating:
          setpoint = eatVelocity.get();
          break;
        case kVomiting:
          setpoint = vomitVelocity.get();
          break;
        case kDigesting:
          setpoint = digestVelocity.get();
          break;
        case kStopped:
          setpoint = 0.0;
      }

      controller.setSetpoint(setpoint);
      io.setVoltage(controller.calculate(inputs.velocityRPMs));
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

  public void digest() {
    mode = IntakeMode.kDigesting;
  }

  public void stop() {
    mode = IntakeMode.kStopped;
  }

  public Command runEatCommand() {
    // run eat mode until a note is obtained
    return new FunctionalCommand(
        () -> System.out.println("Starting to Eat"),
        () -> this.eat(),
        interrupted -> this.stop(),
        this::hasNote,
        this);
  }

  public Command runVomitCommand() {
    // Run eat until a note is expelled.

    // run a sequence that runs vomit mode until note is moved off sensor,
    // then keep running motors for a second to keep advancing,
    // then stop.
    return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("Starting to Vomit"), this),
            new RunCommand(() -> this.vomit(), this).until(this::hasNoNote),
            new WaitCommand(IntakeConstants.kVomitDelay) // run a bit more to advance the note
            )
        .finallyDo(() -> this.stop());
  }

  public Command runDigestCommand() {
    // TODO: Update this to push note through
    return new StartEndCommand(() -> this.digest(), () -> this.stop(), this);
  }
}
