package frc.robot.subsystems.intake.intakedeploy;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("removal")
public class IntakeDeploySubsystem extends SubsystemBase {

  public enum IntakeState {
    STOWED,
    DEPLOYED,
    MOVING_TO_STOW,
    MOVING_TO_DEPLOY,
    HOMING
  }

  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final DigitalInput hallSensor;
  private final PIDController pid;

  private IntakeState state = IntakeState.HOMING;

  // ---------- Tunable NT entries ----------
  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;
  private final DoubleEntry deployAngleEntry;
  private final DoubleEntry stowAngleEntry;
  private final DoubleEntry deployHoldEntry;
  private final DoubleEntry stowHoldEntry;
  private final DoubleEntry toleranceEntry;
  private final DoubleEntry maxOutputVoltsEntry;
  private final DoubleEntry homingVoltsEntry;
  private final DoubleEntry deployFFEntry;
  private final DoubleEntry stowFFEntry;

  // ---------- Logging publishers ----------
  private final BooleanPublisher hallTriggeredPub;
  private final BooleanPublisher atSetpointPub;

  public IntakeDeploySubsystem() {

    motor = new SparkFlex(Constants.IntakeDeploy.MOTOR_ID, MotorType.kBrushless);
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    // This soft limit will prevent the motor controller from attempting to drive
    // mechanism to deploy angle past hard stop
    // Tune direction and value. Starting at 33.75
    config.softLimit.forwardSoftLimit(90).forwardSoftLimitEnabled(true);

    // Apply configuration to motor.
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize hall effect sensor and encoder
    hallSensor = new DigitalInput(Constants.IntakeDeploy.HALL_SENSOR_PORT);
    encoder = motor.getEncoder();

    // motor = new CANSparkMax(Constants.IntakeDeploy.MOTOR_ID,
    // MotorType.kBrushless);
    // hallSensor = new DigitalInput(Constants.IntakeDeploy.HALL_SENSOR_PORT);

    pid =
        new PIDController(
            Constants.IntakeDeploy.kP, Constants.IntakeDeploy.kI, Constants.IntakeDeploy.kD);

    pid.setTolerance(Constants.IntakeDeploy.POSITION_TOLERANCE);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Tuning/IntakeDeploy");

    kPEntry = table.getDoubleTopic("kP").getEntry(Constants.IntakeDeploy.kP);
    kIEntry = table.getDoubleTopic("kI").getEntry(Constants.IntakeDeploy.kI);
    kDEntry = table.getDoubleTopic("kD").getEntry(Constants.IntakeDeploy.kD);

    deployAngleEntry =
        table.getDoubleTopic("DeployAngle").getEntry(Constants.IntakeDeploy.DEPLOY_ANGLE);
    stowAngleEntry = table.getDoubleTopic("StowAngle").getEntry(Constants.IntakeDeploy.STOW_ANGLE);

    deployHoldEntry =
        table.getDoubleTopic("DeployHoldVolts").getEntry(Constants.IntakeDeploy.DEPLOY_HOLD_VOLTS);
    stowHoldEntry =
        table.getDoubleTopic("StowHoldVolts").getEntry(Constants.IntakeDeploy.STOW_HOLD_VOLTS);

    toleranceEntry =
        table.getDoubleTopic("Tolerance").getEntry(Constants.IntakeDeploy.POSITION_TOLERANCE);
    maxOutputVoltsEntry =
        table.getDoubleTopic("MaxOutputVolts").getEntry(Constants.IntakeDeploy.MAX_OUTPUT_VOLTS);
    homingVoltsEntry =
        table.getDoubleTopic("HomingOutputVolts").getEntry(Constants.IntakeDeploy.HOMING_VOLTS);
    deployFFEntry =
        table.getDoubleTopic("DeployFFVolts").getEntry(Constants.IntakeDeploy.DEPLOY_FF_VOLTS);
    stowFFEntry =
        table.getDoubleTopic("StowFFVolts").getEntry(Constants.IntakeDeploy.STOW_FF_VOLTS);

    hallTriggeredPub = table.getBooleanTopic("HallTriggered").publish();
    atSetpointPub = table.getBooleanTopic("AtSetpoint").publish();

    // Push defaults so they appear immediately
    kPEntry.set(Constants.IntakeDeploy.kP);
    kIEntry.set(Constants.IntakeDeploy.kI);
    kDEntry.set(Constants.IntakeDeploy.kD);
    deployAngleEntry.set(Constants.IntakeDeploy.DEPLOY_ANGLE);
    stowAngleEntry.set(Constants.IntakeDeploy.STOW_ANGLE);
    deployHoldEntry.set(Constants.IntakeDeploy.DEPLOY_HOLD_VOLTS);
    stowHoldEntry.set(Constants.IntakeDeploy.STOW_HOLD_VOLTS);
    toleranceEntry.set(Constants.IntakeDeploy.POSITION_TOLERANCE);
    maxOutputVoltsEntry.set(Constants.IntakeDeploy.MAX_OUTPUT_VOLTS);
    homingVoltsEntry.set(Constants.IntakeDeploy.HOMING_VOLTS);
    deployFFEntry.set(Constants.IntakeDeploy.DEPLOY_FF_VOLTS);
    stowFFEntry.set(Constants.IntakeDeploy.STOW_FF_VOLTS);

    SmartDashboard.putString("IntakeDeploy/StartupState", state.name());
  }

  // Helper function to clamp voltage to safe range (or to slow for testing)
  private double clampVoltage(double volts) {
    return Math.max(-maxOutputVoltsEntry.get(), Math.min(maxOutputVoltsEntry.get(), volts));
  }

  // Helper function to set voltage with clamping (replace all motor.setVoltage
  // calls with this)
  private void setClampedVoltage(double volts) {
    double clamped = clampVoltage(volts);
    SmartDashboard.putNumber("IntakeDeploy/ClampedVoltage", clamped);
    Logger.recordOutput("IntakeDeploy/ClampedVoltage", clamped);
    motor.setVoltage(clamped);
  }

  public void deploy() {
    if (state == IntakeState.HOMING) return; // ignore until homed
    state = IntakeState.MOVING_TO_DEPLOY;
  }

  public void stow() {
    state = IntakeState.MOVING_TO_STOW;
  }

  // Run this function during robot initialization to home intake
  public void startHoming() {
    if (isStowedSensorTriggered()) {
      encoder.setPosition(0.0);
      state = IntakeState.STOWED;
    } else {
      state = IntakeState.HOMING;
    }
  }

  public boolean isStowedSensorTriggered() {
    return !hallSensor.get();
  }

  public double getAngleDegrees() {
    return encoder.getPosition() * (360.0 / Constants.IntakeDeploy.GEAR_RATIO);
  }

  public IntakeState getState() {
    return state;
  }

  private void updatePIDFromDashboard() {
    pid.setPID(kPEntry.get(), kIEntry.get(), kDEntry.get());

    pid.setTolerance(toleranceEntry.get());
  }

  @Override
  public void periodic() {
    updatePIDFromDashboard();

    double angle = getAngleDegrees();
    double output = 0.0;

    double deployAngle = deployAngleEntry.get();
    double stowAngle = stowAngleEntry.get();
    double deployHold = deployHoldEntry.get();
    double stowHold = stowHoldEntry.get();
    double homingVolts = homingVoltsEntry.get();

    switch (state) {
      case MOVING_TO_DEPLOY:
        double pidOutput = pid.calculate(angle, deployAngle);
        // Feedforward helps slow descent, reduce bounce
        double deployFF =
            Math.abs(deployFFEntry.get()); // Positive is downwards (homing is negative)
        setClampedVoltage(pidOutput + deployFF);

        if (pid.atSetpoint()) {
          state = IntakeState.DEPLOYED;
        }
        break;

      case MOVING_TO_STOW:
        if (isStowedSensorTriggered() || Math.abs(angle - stowAngle) < 2) {
          setClampedVoltage(stowHold); // hold at top
          state = IntakeState.STOWED;
        } else {
          double pidOutputStow = pid.calculate(angle, stowAngle);
          double stowFF = -Math.abs(stowFFEntry.get()); // negative to help lift
          setClampedVoltage(pidOutputStow + stowFF);
        }
        break;

      case DEPLOYED:
        output = deployHold;
        setClampedVoltage(output);
        break;

      case STOWED:
        output = -stowHold;
        setClampedVoltage(output);
        break;

      case HOMING:
        if (isStowedSensorTriggered()) {
          output = -stowHold;
          setClampedVoltage(output);
          encoder.setPosition(0.0);
          pid.reset();
          state = IntakeState.STOWED;
        } else {
          setClampedVoltage(-homingVolts); // gentle upward voltage
        }
        break;
    }

    // SmartDashboard logging
    SmartDashboard.putNumber("IntakeDeploy/AngleDeg", angle);
    SmartDashboard.putNumber("IntakeDeploy/RawRotations", encoder.getPosition());
    SmartDashboard.putString("IntakeDeploy/State", state.name());
    SmartDashboard.putBoolean("IntakeDeploy/HallTriggered", isStowedSensorTriggered());
    SmartDashboard.putBoolean("IntakeDeploy/AtSetpoint", pid.atSetpoint());
    SmartDashboard.putNumber("IntakeDeploy/PIDOutput", output);
    SmartDashboard.putNumber("IntakeDeploy/Error", pid.getPositionError());

    Logger.recordOutput("IntakeDeploy/AngleDeg", angle);
    Logger.recordOutput("IntakeDeploy/RawRotations", encoder.getPosition());
    Logger.recordOutput("IntakeDeploy/State", state.name());
    Logger.recordOutput("IntakeDeploy/HallTriggered", isStowedSensorTriggered());
    Logger.recordOutput("IntakeDeploy/AtSetpoint", pid.atSetpoint());
    Logger.recordOutput("IntakeDeploy/PIDOutput", output);
    Logger.recordOutput("IntakeDeploy/Error", pid.getPositionError());

    // NetworkTables live logging
    hallTriggeredPub.set(isStowedSensorTriggered());
    atSetpointPub.set(pid.atSetpoint());
  }

  @Override
  public void simulationPeriodic() {}
}

// @SuppressWarnings("removal")

// Logging
// Logger.recordOutput("IntakeDeploy/Position",position);Logger.recordOutput("IntakeDeploy/PIDError",pid.getPositionError());Logger.recordOutput("IntakeDeploy/PIDOutput",output);
// Logger.recordOutput("IntakeDeploy/Goal", goalPosition);
