package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private DigitalInput breamBreak;

  public IntakeIOReal() {
    breamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);
    System.out.println("[Init] Creating IntakeIOReal");
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRPMs = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
    inputs.hasNote = !breamBreak.get(); // TODO: might be opposite
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setBrakeMode(boolean brake) {}

  @Override
  public void setDirection(boolean forward) {}
}
