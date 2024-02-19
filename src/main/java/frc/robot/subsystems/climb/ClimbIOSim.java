package frc.robot.subsystems.climb;

public class ClimbIOSim implements ClimbIO {

  public ClimbIOSim() {
    System.out.println("[Init] Creating ClimbIOSim");
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPosition = 0.0;
    inputs.rightPosition = 0.0;
  }

  @Override
  public void stop() {}
}
