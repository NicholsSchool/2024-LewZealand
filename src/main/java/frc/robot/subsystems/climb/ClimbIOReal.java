package frc.robot.subsystems.climb;

public class ClimbIOReal implements ClimbIO {

  public ClimbIOReal() {
    System.out.println("[Init] Creating ClimbIOReal");
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPosition = 0.0;
    inputs.rightPosition = 0.0;
  }

  @Override
  public void stop() {}
}
