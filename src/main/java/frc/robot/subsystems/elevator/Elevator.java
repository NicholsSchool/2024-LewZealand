package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltageLeft(voltage * 0.1);
    io.setVoltageRight(voltage * 0.1);
  }
}
