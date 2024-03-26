package frc.robot.subsystems.outtake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class OuttakeIOSim implements OuttakeIO {
  private FlywheelSim sim =
      new FlywheelSim(
          DCMotor.getFalcon500(1), Constants.OuttakeConstants.GEAR_RATIO_REDUCTION, 0.004);
  private double appliedVolts = 0.0;

  public OuttakeIOSim() {
    System.out.println("[Init] Creating OuttakeIOSim");
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.velocityRPMs = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setBrakeMode(boolean brake) {}

  @Override
  public void setDirection(boolean forward) {}

  @Override
  public void stop() {
    appliedVolts = 0;
    sim.setInputVoltage(appliedVolts);
  }
}
