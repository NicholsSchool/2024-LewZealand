package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

public class AprilTagVision {

  private AprilTagVisionIO io;
  private AprilTagVisionIOInputsAutoLogged inputs;

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
  }
}
