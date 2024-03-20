package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.CAN.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ArmIOReal implements ArmIO {
  private CANSparkMax leader;
  private CANSparkMax follower;
  private AbsoluteEncoder armEncoder;

  private Solenoid piston;

  public ArmIOReal() {
    System.out.println("[Init] Creating ArmIOReal");

    leader = new CANSparkMax(kArmLeaderCanId, MotorType.kBrushless);
    leader.restoreFactoryDefaults();
    armEncoder = leader.getAbsoluteEncoder(Type.kDutyCycle);

    leader.setInverted(false);
    leader.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    leader.setIdleMode(IdleMode.kBrake);
    armEncoder.setPositionConversionFactor(2.0 * Math.PI);
    armEncoder.setVelocityConversionFactor(2.0 * Math.PI);
    leader.burnFlash();

    follower = new CANSparkMax(kArmFollowerCanId, MotorType.kBrushless);
    follower.setInverted(false);
    follower.restoreFactoryDefaults();
    follower.setIdleMode(IdleMode.kBrake);
    follower.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    // follower.follow(leader);
    // follower.setControlFramePeriodMs(20);
    follower.burnFlash();

    piston = new Solenoid(PneumaticsModuleType.CTREPCM, ARM_SOLENOID_CHANNEL);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleRads = checkRangeRadians(armEncoder.getPosition());
    inputs.angleDegs = checkRangeDegrees(Units.radiansToDegrees(inputs.angleRads));
    inputs.velocityRadsPerSec = armEncoder.getVelocity();
    inputs.appliedOutput = new double[] {leader.getAppliedOutput(), follower.getAppliedOutput()};
    inputs.busVoltage = new double[] {leader.getBusVoltage(), follower.getBusVoltage()};
    inputs.appliedVolts =
        new double[] {
          inputs.busVoltage[0] * inputs.appliedOutput[0],
          inputs.busVoltage[1] * inputs.appliedOutput[1]
        };
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.isExtended = piston.get();
  }

  private double checkRangeRadians(double angle) {
    return angle >= Math.toRadians(200.0) ? angle - 2.0 * Math.PI : angle;
  }

  private double checkRangeDegrees(double angle) {
    return angle >= 200.0 ? angle - 360.0 : angle;
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
    follower.setVoltage(voltage);
  }

  /** Retracts Pistons */
  @Override
  public void retract() {
    piston.set(false); // TODO: confirm
  }

  /** Extends Pistons */
  @Override
  public void extend() {
    piston.set(true); // TODO: confirm
  }
}
