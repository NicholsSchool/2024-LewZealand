package frc.robot.subsystems.climb;

public class ClimbIOSim implements ClimbIO {

  public ClimbIOSim() {
    System.out.println("[Init] Creating ClimbIOSim");
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPositionRads = 0.0;
    inputs.rightPositionRads = 0.0;
    inputs.leftVelocityRadsPerSec = 0.0;
    inputs.rightVelocityRadsPerSec = 0.0;
    inputs.leftClimbCurrent = 0.0;
    inputs.rightClimbCurrent = 0.0;
    inputs.isLocked = false;
  }

  @Override
  public void stop() {}
}
