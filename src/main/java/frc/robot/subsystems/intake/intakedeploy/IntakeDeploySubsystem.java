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

  // Motor and hall sensor
  private final SparkFlex intakeDeploy = new SparkFlex(59, MotorType.kBrushless);
  private final DigitalInput hallSensor = new DigitalInput(9);

  // Positions
  public static final double STOW_POSITION = 0.0;
  public static final double DEFAULT_DEPLOY_POSITION = 18000.0;

  // PID
  private final PIDController pid = new PIDController(0.005, 0.0, 0.0);

  // Tunables (NetworkTables / Elastic)
  private final DoubleEntry deployPositionEntry;
  private final DoubleEntry holdVoltageEntry;
  private final DoubleEntry rampThresholdEntry;
  private final DoubleEntry positionThresholdEntry;
  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;

  // State
  private boolean isDeploying = false;
  private boolean isStowing = false;
  private boolean isHolding = false;
  private double goalPosition = STOW_POSITION;
  private double stowTimer = 0.0;
  private static final double STOW_DEBOUNCE = 0.2; // seconds

  public IntakeDeploySubsystem() {

    // Configure motor
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    intakeDeploy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // NetworkTables setup
    var table = NetworkTableInstance.getDefault().getTable("Tuning/Deploy");

    deployPositionEntry = table.getDoubleTopic("deployPosition").getEntry(DEFAULT_DEPLOY_POSITION);
    holdVoltageEntry = table.getDoubleTopic("holdVoltage").getEntry(Constants.IntakeDeploy.holdVoltage);
    rampThresholdEntry = table.getDoubleTopic("rampThreshold").getEntry(Constants.IntakeDeploy.rampThreshold);
    positionThresholdEntry = table.getDoubleTopic("positionThreshold")
        .getEntry(Constants.IntakeDeploy.positionThreshold);

    kPEntry = table.getDoubleTopic("kP").getEntry(Constants.IntakeDeploy.kP);
    kIEntry = table.getDoubleTopic("kI").getEntry(Constants.IntakeDeploy.kI);
    kDEntry = table.getDoubleTopic("kD").getEntry(Constants.IntakeDeploy.kD);

    // Force initial publish so Elastic sees them
    deployPositionEntry.set(DEFAULT_DEPLOY_POSITION);
    holdVoltageEntry.set(Constants.IntakeDeploy.holdVoltage);
    rampThresholdEntry.set(Constants.IntakeDeploy.rampThreshold);
    positionThresholdEntry.set(Constants.IntakeDeploy.positionThreshold);
    kPEntry.set(Constants.IntakeDeploy.kP);
    kIEntry.set(Constants.IntakeDeploy.kI);
    kDEntry.set(Constants.IntakeDeploy.kD);

    pid.setTolerance(positionThresholdEntry.get());
  }

  // ------------------ Commands ------------------

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

  public boolean isDeployed() {
    return !hallSensor.get(); // hall effect reads false when magnet engaged
  }

  public boolean isStowed() {
    return !hallSensor.get();
  }

  public boolean atDeployPosition() {
    return Math.abs(pid.getPositionError()) < positionThresholdEntry.get();
  }

  public boolean atStowPosition() {
    return Math.abs(pid.getPositionError()) < positionThresholdEntry.get();
  }

  public Command deployCommand() {
    return Commands.either(
        Commands.runOnce(this::deploy).andThen(Commands.waitUntil(this::atDeployPosition)),
        Commands.none(),
        () -> !isDeployed());
  }

  public Command stowCommand() {
    return Commands.runOnce(this::stow)
        .andThen(
            Commands.waitUntil(
                () -> Math.abs(intakeDeploy.getEncoder().getPosition() * 360.0 - STOW_POSITION) < positionThresholdEntry
                    .get()));
  }

  public Command deployAgitatorCommand() {
    return Commands.sequence(
        Commands.run(() -> intakeDeploy.setVoltage(-0.5), this).withTimeout(0.8),
        Commands.run(() -> intakeDeploy.setVoltage(0.3), this).withTimeout(0.4),
        stowCommand());
  }

  public Command homeCommand() {
    return Commands.runOnce(() -> {
      if (!isStowed()) {
        stow();
      }
    }).andThen(Commands.waitUntil(this::isStowed))
        .andThen(Commands.runOnce(() -> intakeDeploy.getEncoder().setPosition(STOW_POSITION)));
  }

  // ------------------ Periodic ------------------

  @Override
  public void periodic() {

    double position = intakeDeploy.getEncoder().getPosition() * 360.0;

    // Update PID gains
    pid.setP(kPEntry.get());
    pid.setI(kIEntry.get());
    pid.setD(kDEntry.get());

    pid.setSetpoint(goalPosition);
    double distanceToGoal = goalPosition - position;
    double output = pid.calculate(position);

    // Ramp down near stow
    if (isStowing && Math.abs(distanceToGoal) < rampThresholdEntry.get()) {
      output *= 0.3;
    }

    // Apply hold voltage if very close and hall sensor engaged
    if (isStowing && Math.abs(distanceToGoal) < positionThresholdEntry.get() && isStowed()) {
      intakeDeploy.setVoltage(-holdVoltageEntry.get());
      isHolding = true;
      isStowing = false;
    } else {
      intakeDeploy.setVoltage(output);
      isHolding = false;
    }

    // Soft debounce for stow completion
    if (isStowing && Math.abs(position - STOW_POSITION) < positionThresholdEntry.get()) {
      stowTimer += 0.02;
      if (stowTimer >= STOW_DEBOUNCE) {
        isStowing = false;
        isDeploying = false;
        goalPosition = STOW_POSITION;
        intakeDeploy.getEncoder().setPosition(STOW_POSITION);
      }
    } else {
      stowTimer = 0.0;
    }

    // Logging
    Logger.recordOutput("IntakeDeploy/Position", position);
    Logger.recordOutput("IntakeDeploy/PIDError", pid.getPositionError());
    Logger.recordOutput("IntakeDeploy/PIDOutput", output);
    Logger.recordOutput("IntakeDeploy/Goal", goalPosition);

    SmartDashboard.putNumber("IntakeDeploy/Position", position);
    SmartDashboard.putNumber("IntakeDeploy/PIDError", pid.getPositionError());
    SmartDashboard.putNumber("IntakeDeploy/PIDOutput", output);
    SmartDashboard.putNumber("IntakeDeploy/Goal", goalPosition);
    SmartDashboard.putBoolean("IntakeDeploy/isDeployed", isDeployed());
    SmartDashboard.putBoolean("IntakeDeploy/isStowing", isStowing);
    SmartDashboard.putBoolean("IntakeDeploy/isDeploying", isDeploying);
    SmartDashboard.putBoolean("IntakeDeploy/isHolding", isHolding);
  }

  @Override
  public void simulationPeriodic() {
    // WPILib sim
  }
}