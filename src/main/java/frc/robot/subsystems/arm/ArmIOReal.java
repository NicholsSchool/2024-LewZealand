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
    follower.restoreFactoryDefaults();
    follower.setIdleMode(IdleMode.kBrake);
    follower.follow(leader);
    follower.burnFlash();

    piston = new Solenoid(PneumaticsModuleType.CTREPCM, ARM_SOLENOID_CHANNEL);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleRads = armEncoder.getPosition();
    inputs.angleDegs = Units.radiansToDegrees(inputs.angleRads);
    inputs.velocityRadsPerSec = armEncoder.getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.isExtended = piston.get(); // TODO: check that default is what we think
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
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
