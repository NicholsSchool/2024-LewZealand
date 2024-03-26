package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
  double voltage = 0.0;

  private final double startingAngleRads =
      Units.degreesToRadians(75.0); // set this to desired arm starting angle
  private final double momentScaleFactor = 0.525; // scale factor to apply to the moment to
  // result in the arm not moving with 0.0 command
  // (only gravity FF applied).
  // this may be result of physics sim inaccuracies (TJG)
  private double momentOfInertiaKgMetersSquared =
      ((ARM_MASS_LBS * KgPerLb) * Math.pow(ARM_LENGTH_METERS, 2)) * momentScaleFactor;

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          ARM_GEAR_REDUCTION,
          momentOfInertiaKgMetersSquared,
          ARM_LENGTH_METERS,
          MIN_ANGLE_RADS,
          MAX_ANGLE_RADS,
          true, // simulate gravity
          startingAngleRads);

  public ArmIOSim() {
    System.out.println("[Init] Creating ArmIOSim " + momentOfInertiaKgMetersSquared);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.setInputVoltage(voltage);
    sim.update(Constants.loopPeriodSecs);

    inputs.angleRads = sim.getAngleRads();
    inputs.angleDegs = Units.radiansToDegrees(inputs.angleRads);
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = new double[] {voltage};
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps(), sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }
}
