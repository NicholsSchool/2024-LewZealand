package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  public ElevatorIOSim() {
    System.out.println("Creating Eleavtor Sim");
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftPositionRads = 0.0;
    inputs.rightPositionRads = 0.0;
    inputs.leftVelocityRadsPerSec = 0.0;
    inputs.rightVelocityRadsPerSec = 0.0;
    inputs.leftClimbCurrent = 0.0;
    inputs.rightClimbCurrent = 0.0;
  }
}
