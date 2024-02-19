package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {
  public ArmIOSim() {
    System.out.println("[Init] Creating ArmIOSim");
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = 0.0;
    inputs.isExtended = false;
  }

  @Override
  public void stop() {}
}
