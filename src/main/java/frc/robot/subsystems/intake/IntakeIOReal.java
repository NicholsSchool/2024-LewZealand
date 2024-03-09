package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private DigitalInput breamBreak;
  private CANSparkMax motor;
  private RelativeEncoder encoder;

  public IntakeIOReal() {
    System.out.println("[Init] Creating IntakeIOReal");

    breamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);
    motor = new CANSparkMax(CAN.kIntakeCanId, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    encoder = motor.getEncoder();
    motor.setIdleMode(IdleMode.kCoast);
    encoder.setPositionConversionFactor(2.0 * Math.PI);
    encoder.setVelocityConversionFactor(2.0 * Math.PI / 60.0 / 5.0);
    motor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadPerSec = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.hasNote = !breamBreak.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
