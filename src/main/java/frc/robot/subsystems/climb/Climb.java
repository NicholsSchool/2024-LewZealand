package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

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

  public void setPower(double power) {
    mode = ClimbMode.kSetPosition;
  }

  public void stop() {
    mode = ClimbMode.kStopped;
  }
}
