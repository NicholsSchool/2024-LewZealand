package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeIOReal implements OuttakeIO {
  private final TalonFX motor;

  public OuttakeIOReal() {
    System.out.println("[Init] Creating OuttakeIOReal");

    motor = new TalonFX(Constants.CAN.kOuttakeCanId);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = OuttakeConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.velocityRPMs =
        (motor.getVelocity().getValueAsDouble() / Constants.OuttakeConstants.GEAR_RATIO_REDUCTION)
            * 60.0;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setDirection(boolean forward) {
    motor.setInverted(!forward);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
