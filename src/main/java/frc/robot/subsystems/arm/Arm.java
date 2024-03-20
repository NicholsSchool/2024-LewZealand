package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double manuelInput = 0.0;

  private ArmFeedforward ARM_FF = new ArmFeedforward(ARM_FF_KS, ARM_FF_KG, ARM_FF_KV, ARM_FF_KA);

  private final ProfiledPIDController armPidController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private Timer timerMoveToPose; // timer to check for timeout of move to position
  private double voltageCommand = 0.0;

  private double previousVelocity = 0.0;
  private double acclerationRad = 0.0;

  private double targetAngleDeg = 0.0;
  private double voltageCmdPid = 0.0;
  private double voltageCmdFF = 0.0;
  private boolean reachedTargetPos = false;
  private boolean targetPosSet = false;

  // tunable parameters
  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/kV");
  private static final LoggedTunableNumber armKa = new LoggedTunableNumber("Arm/kA");

  private static final LoggedTunableNumber positionToleranceDeg =
      new LoggedTunableNumber("Arm/PositionToleranceDeg");
  private static final LoggedTunableNumber armMaxVelocityRad =
      new LoggedTunableNumber("Arm/MaxVelocityRad");
  private static final LoggedTunableNumber armMaxAccelerationRad =
      new LoggedTunableNumber("Arm/MaxAccelerationRad");
  private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/Kp");
  private static final LoggedTunableNumber armKi = new LoggedTunableNumber("Arm/Ki");
  private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/Kd");
  private static final LoggedTunableNumber moveToPosTimeoutSec =
      new LoggedTunableNumber("Arm/TimeoutSec");

  private static enum ArmState {
    kManuel,
    kGoToPos,
  };

  private static enum PistonState {
    kExtended,
    kRetracted
  };

  private ArmState armState;
  private PistonState pistonState;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
    armState = ArmState.kGoToPos;
    pistonState = PistonState.kRetracted;

    reachedTargetPos = false;
    timerMoveToPose = new Timer();
    timerMoveToPose.start();
    timerMoveToPose.reset();

    armKg.initDefault(ARM_FF_KG);
    armKv.initDefault(ARM_FF_KV);
    armKa.initDefault(ARM_FF_KA);
    positionToleranceDeg.initDefault(1.0);
    armMaxVelocityRad.initDefault(Constants.ArmConstants.ARM_VEL_LIMIT);
    armMaxAccelerationRad.initDefault(Constants.ArmConstants.ARM_ACCEL_LIMIT);
    armKp.initDefault(Constants.ArmConstants.ARM_P);
    armKi.initDefault(Constants.ArmConstants.ARM_I);
    armKd.initDefault(Constants.ArmConstants.ARM_D);
    moveToPosTimeoutSec.initDefault(5.0);

    armPidController.setP(armKp.get());
    armPidController.setI(armKi.get());
    armPidController.setD(armKd.get());
    armPidController.setConstraints(
        new TrapezoidProfile.Constraints(armMaxVelocityRad.get(), armMaxAccelerationRad.get()));
    armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    updateTunables();

    acclerationRad = (inputs.velocityRadsPerSec - previousVelocity) / 0.02;
    previousVelocity = inputs.velocityRadsPerSec;

    // if no target pose has been set yet,
    // then set as current position. This
    // is to have arm hold its starting position on enable.
    if (!targetPosSet) {
      targetPosSet = true;
      setTargetPos(90.0);
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      // manuelInput = 0.0;
      // armState = ArmState.kManuel;
    }

    switch (armState) {
      case kManuel:
        voltageCmdPid = 0.0;
        voltageCmdFF =
            ARM_FF.calculate(
                inputs.angleRads, Constants.ArmConstants.ARM_VEL_LIMIT * softLimit(manuelInput));
        break;
      case kGoToPos:
        voltageCmdPid = armPidController.calculate(inputs.angleRads);
        voltageCmdFF = ARM_FF.calculate(inputs.angleRads, armPidController.getSetpoint().velocity);

        if (!reachedTargetPos) {
          reachedTargetPos = armPidController.atGoal();
          if (reachedTargetPos) {
            System.out.println("Arm Move To Pos Reached Goal!");
            timerMoveToPose.stop();
          }
        }
        break;
    }
    voltageCommand = voltageCmdPid + voltageCmdFF;
    io.setVoltage(voltageCommand);

    switch (pistonState) {
      case kExtended:
        io.extend();
        break;
      case kRetracted:
        io.retract();
        break;
    }
  }

  // set soft limits on the input velocity of the arm to make
  // sure arm does not extend passed danger position.
  public double softLimit(double inputVel) {
    // weird ranges due to [0, 360] angle range of the arm angle input
    if ((inputs.angleDegs >= 100.0 && inputs.angleDegs <= 200.0) && inputVel > 0
        || (inputs.angleDegs <= 2.0 || inputs.angleDegs >= 200.0) && inputVel < 0) {
      return 0.0;
    }
    return inputVel;
  }

  // this method can be used to determine if GoToPos
  // has completed either with timeout or has reached goal.
  public boolean hasReachedTarget() {
    if (reachedTargetPos) {
      return true;
    } else {
      if (timerMoveToPose.get() > moveToPosTimeoutSec.get()) {
        System.out.println("Arm Move To Pos Timeout!");
        return true;
      } else {
        return false;
      }
    }
  }

  // called from run command on every cycle when manual is running.
  public void setManuel(double manuelInput) {
    armState = ArmState.kManuel;
    this.manuelInput = manuelInput;
  }

  // assumption is that this is called once to set the target position, not continuously.
  public void setTargetPos(double targetAngleDeg) {
    this.targetAngleDeg = targetAngleDeg;
    System.out.println("Arm Go To Pos(deg): " + targetAngleDeg);
    timerMoveToPose.restart();
    // set new goal of the pid controller.
    armPidController.setGoal(Units.degreesToRadians(targetAngleDeg));
    armPidController.reset(inputs.angleRads);
    reachedTargetPos = false;
    targetPosSet = true;
  }

  // called from run command on every cycle while motion profile is running.
  public void setGoToPos() {
    this.manuelInput = 0.0;
    armState = ArmState.kGoToPos;
  }

  // called from instant command
  public void setExtended() {
    pistonState = PistonState.kExtended;
  }

  // called from instant command
  public void setRetracted() {
    pistonState = PistonState.kRetracted;
  }

  private void updateTunables() {
    // Update from tunable numbers
    if (armKg.hasChanged(hashCode())
        || armKv.hasChanged(hashCode())
        || armKa.hasChanged(hashCode())) {
      ARM_FF = new ArmFeedforward(0.0, armKg.get(), armKv.get(), armKa.get());
    }

    if (armMaxVelocityRad.hasChanged(hashCode())
        || armMaxAccelerationRad.hasChanged(hashCode())
        || positionToleranceDeg.hasChanged(hashCode())
        || armKp.hasChanged(hashCode())
        || armKi.hasChanged(hashCode())
        || armKd.hasChanged(hashCode())) {
      armPidController.setP(armKp.get());
      armPidController.setI(armKi.get());
      armPidController.setD(armKd.get());
      armPidController.setConstraints(
          new TrapezoidProfile.Constraints(armMaxVelocityRad.get(), armMaxAccelerationRad.get()));
      armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
    }
  }

  // COMMANDS

  // Create command that will move to target angle until motion completes.
  public Command runGoToPosCommand(double targetAngleDeg) {
    // run eat mode until a note is obtained
    return new FunctionalCommand(
        () -> this.setTargetPos(targetAngleDeg),
        this::setGoToPos,
        interrupted -> this.armState = ArmState.kManuel, // back to manual when done
        this::hasReachedTarget,
        this);
  }

  public double getAngleDeg() {
    return inputs.angleDegs;
  }

  // THINGS TO LOG IN ADV SCOPE

  @AutoLogOutput
  public double getTargetAngleDeg() {
    return targetAngleDeg;
  }

  @AutoLogOutput
  public double getAcceleration() {
    return acclerationRad;
  }

  @AutoLogOutput
  public double getVoltageCommand() {
    return voltageCommand;
  }

  @AutoLogOutput
  public double getVoltageCommandPid() {
    return voltageCmdPid;
  }

  @AutoLogOutput
  public double getVoltageCommandFF() {
    return voltageCmdFF;
  }

  @AutoLogOutput
  public double[] getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public ArmState getMode() {
    return armState;
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return armPidController.atGoal();
  }

  @AutoLogOutput
  public double moveDurationTimeSeconds() {
    return timerMoveToPose.get();
  }
}
