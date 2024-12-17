package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EleavtorIOReal implements ElevatorIO {

  private TalonFX leftElevator;
  private TalonFX rightElevator;

  public EleavtorIOReal() {
    leftElevator = new TalonFX(37);
    rightElevator = new TalonFX(36);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 35;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftElevator.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightElevator.getConfigurator().apply(config);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftPositionRads = leftElevator.getPosition().getValueAsDouble();
    inputs.rightPositionRads = rightElevator.getPosition().getValueAsDouble();
    inputs.leftVelocityRadsPerSec = leftElevator.getVelocity().getValueAsDouble();
    inputs.rightVelocityRadsPerSec = rightElevator.getVelocity().getValueAsDouble();
    inputs.leftClimbCurrent = leftElevator.getStatorCurrent().getValueAsDouble();
    inputs.rightClimbCurrent = rightElevator.getStatorCurrent().getValueAsDouble();
  }

  public void setVoltageLeft(double voltage) {
    leftElevator.setVoltage(voltage);
  }

  public void setVoltageRight(double voltage) {
    rightElevator.setVoltage(voltage);
  }

  public void stop() {
    leftElevator.stopMotor();
    rightElevator.stopMotor();
  }
}
