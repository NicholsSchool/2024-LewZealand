package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import java.util.Objects;

/** Hardware interface for the NAVX 3 axis gyro */
public class GyroIONAVX implements GyroIO {
  private final AHRS navx;

  /** Constructor to initialize the NAVX */
  public GyroIONAVX() {
    if (Objects.requireNonNull(Constants.getRobot()) == Constants.RobotType.ROBOT_2024C) {
      navx = new AHRS(SPI.Port.kMXP);
    } else {
      throw new RuntimeException("Invalid robot for NAVX");
    }
  }

  /**
   * Update the AK hardware inputs
   *
   * @param inputs the inputs to update
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    inputs.pitchRad = navx.getRawGyroX();
    inputs.velocityPitchRadPerSec = navx.getVelocityX();
    inputs.yawRad = navx.getRawGyroY();
    inputs.velocityYawRadPerSec = navx.getVelocityY();
    inputs.rollRad = navx.getRawGyroZ();
    inputs.velocityRollRadPerSec = navx.getVelocityZ();
  }
}
