package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs
    implements LoggableInputs, Cloneable {
  public static Rotation2d yawPos;
  public static double yawVelocityRadPerSec;

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("PitchRad", pitchRad);
    table.put("VelocityPitchRadPerSec", velocityPitchRadPerSec);
    table.put("YawRad", yawRad);
    table.put("VelocityYawRadPerSec", velocityYawRadPerSec);
    table.put("RollRad", rollRad);
    table.put("VelocityRollRadPerSec", velocityRollRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    pitchRad = table.get("PitchRad", pitchRad);
    velocityPitchRadPerSec = table.get("VelocityPitchRadPerSec", velocityPitchRadPerSec);
    yawRad = table.get("YawRad", yawRad);
    velocityYawRadPerSec = table.get("VelocityYawRadPerSec", velocityYawRadPerSec);
    rollRad = table.get("RollRad", rollRad);
    velocityRollRadPerSec = table.get("VelocityRollRadPerSec", velocityRollRadPerSec);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.pitchRad = this.pitchRad;
    copy.velocityPitchRadPerSec = this.velocityPitchRadPerSec;
    copy.yawRad = this.yawRad;
    copy.velocityYawRadPerSec = this.velocityYawRadPerSec;
    copy.rollRad = this.rollRad;
    copy.velocityRollRadPerSec = this.velocityRollRadPerSec;
    return copy;
  }
}
