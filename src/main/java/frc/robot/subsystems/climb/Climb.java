package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private double voltageCommand = 0.0;

  private static enum ClimbMode {
    kStopped,
    kSetPosition,
    kFullyRetract,
    kFullyExtend
  };

  private ClimbMode mode = ClimbMode.kStopped;

  public Climb(ClimbIO io) {
    System.out.println("[Init] Creating Climb");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.stop();
      mode = ClimbMode.kStopped;
    } else {
      switch (mode) {
        case kSetPosition:
          break;
        case kFullyRetract:
          break;
        case kFullyExtend:
          break;
        case kStopped:
          io.stop();
      }
    }
  }

  public void fullyRetract() {
    mode = ClimbMode.kFullyRetract;
  }

  public void fullyExtend() {
    mode = ClimbMode.kFullyExtend;
  }

  public void setPosition() {
    mode = ClimbMode.kSetPosition;
  }

  public void setVoltage(double voltage) {
    voltageCommand = voltage;
    if ((inputs.leftPositionRads > 33.3 && voltageCommand > 0)
        || (inputs.leftPositionRads < 0.0 && voltageCommand < 0)) {
      io.setVoltageLeft(0.0);
    } else io.setVoltageLeft(voltage);
    if ((inputs.rightPositionRads > 33.3 && voltageCommand > 0)
        || (inputs.rightPositionRads < 0.0 && voltageCommand < 0)) {
      io.setVoltageRight(0.0);
    } else io.setVoltageRight(voltage);
  }

  @AutoLogOutput
  public double getVoltageCommand() {
    return this.voltageCommand;
  }

  public void setPower(double power) {
    mode = ClimbMode.kSetPosition;
  }

  @AutoLogOutput
  public double getLeftEncoder() {
    return inputs.leftPositionRads;
  }

  @AutoLogOutput
  public double getRightEncoder() {
    return inputs.rightPositionRads;
  }

  public void stop() {
    mode = ClimbMode.kStopped;
  }
}
