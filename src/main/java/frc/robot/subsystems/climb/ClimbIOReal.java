package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class ClimbIOReal implements ClimbIO {

  private CANSparkMax leftClimb;
  private CANSparkMax rightClimb;

  private RelativeEncoder leftClimbEncoder;
  private RelativeEncoder rightClimbEncoder;

  public ClimbIOReal() {
    System.out.println("[Init] Creating ClimbIOReal");

    leftClimb = new CANSparkMax(Constants.CAN.kLeftClimberId, MotorType.kBrushless);
    rightClimb = new CANSparkMax(Constants.CAN.kRightClimberId, MotorType.kBrushless);

    leftClimbEncoder = leftClimb.getEncoder();
    rightClimbEncoder = rightClimb.getEncoder();

    leftClimb.setIdleMode(IdleMode.kBrake);
    leftClimb.setInverted(false);
    leftClimb.setSmartCurrentLimit(35); // amps
    leftClimbEncoder.setPositionConversionFactor(2 * Math.PI / 25.0);
    leftClimbEncoder.setVelocityConversionFactor(2 * Math.PI / 25.0);
    leftClimb.burnFlash();

    rightClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setInverted(true);
    rightClimb.setSmartCurrentLimit(35); // amps
    rightClimbEncoder.setPositionConversionFactor(2 * Math.PI / 25.0);
    rightClimbEncoder.setVelocityConversionFactor(2 * Math.PI / 25.0);
    rightClimb.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPositionRads = leftClimbEncoder.getPosition();
    inputs.rightPositionRads = rightClimbEncoder.getPosition();
    inputs.leftVelocityRadsPerSec = leftClimbEncoder.getVelocity();
    inputs.rightVelocityRadsPerSec = rightClimbEncoder.getVelocity();
    inputs.leftClimbCurrent = leftClimb.getOutputCurrent();
    inputs.rightClimbCurrent = rightClimb.getOutputCurrent();
  }

  public void setVoltageLeft(double voltage) {
    leftClimb.setVoltage(voltage);
  }

  public void setVoltageRight(double voltage) {
    rightClimb.setVoltage(voltage);
  }

  @Override
  public void stop() {}
}
