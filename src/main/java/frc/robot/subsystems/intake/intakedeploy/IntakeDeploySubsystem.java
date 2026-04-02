package frc.robot.subsystems.intake.intakedeploy;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("removal")
public class IntakeDeploySubsystem extends SubsystemBase {

  private final SparkFlex intakeDeploy = new SparkFlex(59, MotorType.kBrushless);
  private final DigitalInput hallSensor = new DigitalInput(9);

  // Default positions
  public static final double STOW_POSITION = 0.0;

  // NetworkTables tunables (start with values from Constants)
  private final DoubleEntry deployPositionEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("deployPosition")
      .getEntry(Constants.IntakeDeploy.deployPosition);

  private final DoubleEntry holdVoltageEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("holdVoltage")
      .getEntry(Constants.IntakeDeploy.holdVoltage);

  private final DoubleEntry rampThresholdEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("rampThreshold")
      .getEntry(Constants.IntakeDeploy.rampThreshold);

  private final DoubleEntry positionThresholdEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("positionThreshold")
      .getEntry(Constants.IntakeDeploy.positionThreshold);

  private final DoubleEntry kPEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("kP")
      .getEntry(Constants.IntakeDeploy.kP);

  private final DoubleEntry kIEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("kI")
      .getEntry(Constants.IntakeDeploy.kI);

  private final DoubleEntry kDEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("kD")
      .getEntry(Constants.IntakeDeploy.kD);

  private final PIDController pid = new PIDController(kPEntry.get(), kIEntry.get(), kDEntry.get());

  private boolean isDeploying = false;
  private boolean isStowing = false;
  private boolean isHolding = false;

  private double goalPosition = STOW_POSITION;
  private double stowTimer = 0.0;
  private static final double STOW_DEBOUNCE = 0.2; // seconds

  public IntakeDeploySubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    intakeDeploy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(positionThresholdEntry.get());

    // Force publish initial values to NetworkTables
    deployPositionEntry.setDefault(Constants.IntakeDeploy.deployPosition);
    holdVoltageEntry.setDefault(Constants.IntakeDeploy.holdVoltage);
    rampThresholdEntry.setDefault(Constants.IntakeDeploy.rampThreshold);
    positionThresholdEntry.setDefault(Constants.IntakeDeploy.positionThreshold);
    kPEntry.setDefault(Constants.IntakeDeploy.kP);
    kIEntry.setDefault(Constants.IntakeDeploy.kI);
    kDEntry.setDefault(Constants.IntakeDeploy.kD);
  }

  /** Commands just set the goal and flags. PID handles movement asynchronously */
  public void deploy() {
    goalPosition = deployPositionEntry.get();
    isDeploying = true;
    isStowing = false;
    isHolding = false;
  }

  public void stow() {
    goalPosition = STOW_POSITION;
    isStowing = true;
    isDeploying = false;
    isHolding = false;
  }

  /** Hall sensor: true when magnet is detected (intake stowed) */
  public boolean isStowed() {
    return hallSensor.get();
  }

  /** Command objects */
  public Command deployCommand() {
    return Commands.runOnce(this::deploy, this);
  }

  public Command stowCommand() {
    return Commands.runOnce(this::stow, this);
  }

  public Command deployAgitatorCommand() {
    return Commands.sequence(
        Commands.run(() -> intakeDeploy.setVoltage(-0.5), this).withTimeout(0.8),
        Commands.run(() -> intakeDeploy.setVoltage(0.3), this).withTimeout(0.4),
        stowCommand());
  }

  @Override
  public void periodic() {
    double position = intakeDeploy.getEncoder().getPosition() * 360.0;
    pid.setSetpoint(goalPosition);

    pid.setP(kPEntry.get());
    pid.setI(kIEntry.get());
    pid.setD(kDEntry.get());

    double distanceToGoal = goalPosition - position;
    double output = pid.calculate(position);

    // Ramp down near stow
    if (isStowing && Math.abs(distanceToGoal) < rampThresholdEntry.get()) {
      output *= 0.3;
    }

    // Hold voltage when very close and stowed
    if (isStowing && Math.abs(distanceToGoal) < positionThresholdEntry.get() && isStowed()) {
      intakeDeploy.setVoltage(-holdVoltageEntry.get());
      isHolding = true;
    } else {
      intakeDeploy.setVoltage(output);
      isHolding = false;
    }

    // Debounce stow completion and reset encoder
    if (isStowing && Math.abs(position - STOW_POSITION) < positionThresholdEntry.get()) {
      stowTimer += 0.02;
      if (stowTimer >= STOW_DEBOUNCE) {
        isStowing = false;
        goalPosition = STOW_POSITION;
        intakeDeploy.getEncoder().setPosition(0.0);
      }
    } else {
      stowTimer = 0.0;
    }

    // Logging and SmartDashboard
    Logger.recordOutput("IntakeDeploy/Position", position);
    Logger.recordOutput("IntakeDeploy/PIDError", pid.getPositionError());
    Logger.recordOutput("IntakeDeploy/PIDOutput", output);
    Logger.recordOutput("IntakeDeploy/Goal", goalPosition);

    SmartDashboard.putNumber("IntakeDeploy/Position", position);
    SmartDashboard.putNumber("IntakeDeploy/PIDError", pid.getPositionError());
    SmartDashboard.putNumber("IntakeDeploy/PIDOutput", output);
    SmartDashboard.putNumber("IntakeDeploy/Goal", goalPosition);
    SmartDashboard.putBoolean("IntakeDeploy/isStowing", isStowing);
    SmartDashboard.putBoolean("IntakeDeploy/isDeploying", isDeploying);
    SmartDashboard.putBoolean("IntakeDeploy/isHolding", isHolding);
    SmartDashboard.putBoolean("IntakeDeploy/isStowed", isStowed());
  }

  @Override
  public void simulationPeriodic() {
    // WPILib sim
  }

  /** Homing helper: run on robot init to stow and reset encoder if needed */
  public void homeIntake() {
    if (!isStowed()) {
      stow(); // sets goalPosition and flags
    } else {
      intakeDeploy.getEncoder().setPosition(0.0);
    }
  }
}