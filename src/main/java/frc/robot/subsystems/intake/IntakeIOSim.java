package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {

  public IntakeIOSim() {
    System.out.println("[Init] Creating IntakeIOSim");
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: do fancy pretending for hasNote
    inputs.velocityRPMs = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
    inputs.hasNote = false;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setBrakeMode(boolean brake) {}

  @Override
  public void setDirection(boolean forward) {}
}
