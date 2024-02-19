package frc.robot.subsystems.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  private static final LoggedTunableNumber ampVelocity =
      new LoggedTunableNumber("Outtake/EatVelocityRPMs");
  private static final LoggedTunableNumber speakerVelocity =
      new LoggedTunableNumber("Outtake/VomitVelocityRPMs");
  private static final LoggedTunableNumber trapVelocity =
      new LoggedTunableNumber("Outtake/DigestVelocityRPMs");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Outtake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Outtake/kD");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private static enum OuttakeMode {
    kStopped,
    kAmp,
    kSpeaker,
    kTrap
  };

  private OuttakeMode mode = OuttakeMode.kStopped;

  // initialize tunable values
  static {
    ampVelocity.initDefault(0.0);
    speakerVelocity.initDefault(0.0);
    trapVelocity.initDefault(0.0);
    kP.initDefault(OuttakeConstants.kP);
    kD.initDefault(OuttakeConstants.kD);
  }

  public Outtake(OuttakeIO io) {
    System.out.println("[Init] Creating Outtake");
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    double setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      controller.reset();
      mode = OuttakeMode.kStopped;
    } else {
      switch (mode) {
        case kAmp:
          setpoint = ampVelocity.get();
          break;
        case kSpeaker:
          setpoint = speakerVelocity.get();
          break;
        case kTrap:
          setpoint = trapVelocity.get();
          break;
        case kStopped:
          setpoint = 0.0;
      }

      controller.setSetpoint(setpoint);
      io.setVoltage(controller.calculate(inputs.velocityRPMs));
    }
  }

  public void setAmp() {
    mode = OuttakeMode.kAmp;
  }

  public void setSpeaker() {
    mode = OuttakeMode.kSpeaker;
  }

  public void setTrap() {
    mode = OuttakeMode.kTrap;
  }

  public void stop() {
    mode = OuttakeMode.kStopped;
  }
}
