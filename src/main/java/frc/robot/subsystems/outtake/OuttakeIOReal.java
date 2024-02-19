package frc.robot.subsystems.outtake;

public class OuttakeIOReal implements OuttakeIO {

  public OuttakeIOReal() {
    System.out.println("[Init] Creating OuttakeIOReal");
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.velocityRPMs = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setBrakeMode(boolean brake) {}

  @Override
  public void setDirection(boolean forward) {}
}
