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

  // Positions
  public static final double STOW_POSITION = 0.0;
  public static final double DEFAULT_DEPLOY_POSITION = 18000.0;

  // PID
  private final PIDController pid = new PIDController(0.005, 0.0, 0.0);

  // NetworkTable tunables
  private final DoubleEntry deployPositionEntry = NetworkTableInstance.getDefault().getTable("Tuning/Deploy")
      .getDoubleTopic("deployPosition").getEntry(DEFAULT_DEPLOY_POSITION);

  private final DoubleEntry holdVoltageEntry = NetworkTableInstance.getDefault().getTable("Tuning/Deploy")
      .getDoubleTopic("holdVoltage").getEntry(0.2);

  private final DoubleEntry rampThresholdEntry = NetworkTableInstance.getDefault().getTable("Tuning/Deploy")
      .getDoubleTopic("rampThreshold").getEntry(1500.0);

  private final DoubleEntry positionThresholdEntry = NetworkTableInstance.getDefault().getTable("Tuning/Deploy")
      .getDoubleTopic("positionThreshold").getEntry(75.0);

  // State flags
  private boolean isDeploying = false;
  private boolean isStowing = false;
  private boolean isHolding = false;

  private double stowTimer = 0.0;
  private static final double STOW_DEBOUNCE = 0.2; // seconds

  private double goalPosition = STOW_POSITION;

  public IntakeDeploySubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    intakeDeploy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(positionThresholdEntry.get());
  }

  /** Deploy intake */
  public void deploy() {
    goalPosition = deployPositionEntry.get(DEFAULT_DEPLOY_POSITION);
    isDeploying = true;
    isStowing = false;
    isHolding = false;
  }

  /** Stow intake */
  public void stow() {
    goalPosition = STOW_POSITION;
    isStowing = true;
    isDeploying = false;
    isHolding = false;
  }

  /** Returns true if intake is physically deployed (magnet NOT engaged) */
  public boolean isDeployed() {
    return !hallSensor.get(); // false when magnet engaged
  }

  /** Returns true if intake is physically stowed (magnet engaged) */
  public boolean isStowed() {
    return hallSensor.get(); // true when magnet engaged
  }

  /** Checks if PID is within tolerance of deploy position */
  public boolean atDeployPosition() {
    return Math.abs(pid.getPositionError()) < positionThresholdEntry.get();
  }

  /** Checks if PID is within tolerance of stow position */
  public boolean atStowPosition() {
    return Math.abs(pid.getPositionError()) < positionThresholdEntry.get();
  }

  /** Command to deploy if not already deployed */
  public Command deployCommand() {
    return Commands.either(
        Commands.runOnce(this::deploy).andThen(Commands.waitUntil(this::atDeployPosition)),
        Commands.none(),
        () -> !isDeployed());
  }

  /** Command to stow if not already stowed */
  public Command stowCommand() {
    return Commands.either(
        Commands.runOnce(this::stow).andThen(Commands.waitUntil(this::atStowPosition)),
        Commands.none(),
        () -> !isStowed());
  }

  /** Periodic PID control and hold logic */
  @Override
  public void periodic() {
    double position = intakeDeploy.getEncoder().getPosition() * 360.0;
    pid.setSetpoint(goalPosition);

    double output = pid.calculate(position);

    double distanceToGoal = goalPosition - position;

    // Ramp down PID near stow
    if (isStowing && Math.abs(distanceToGoal) < rampThresholdEntry.get()) {
      output *= 0.3;
    }

    // Apply hold voltage only when very close to stow
    if (isStowing && isStowed()) {
      intakeDeploy.setVoltage(-holdVoltageEntry.get());
      isHolding = true;
      output = 0.0;
    } else {
      isHolding = false;
    }

    // Debounce stow completion
    if (isStowing && Math.abs(position - STOW_POSITION) < positionThresholdEntry.get()) {
      stowTimer += 0.02; // assuming 20ms periodic
      if (stowTimer >= STOW_DEBOUNCE) {
        isStowing = false;
        isDeploying = false;
        goalPosition = STOW_POSITION;
        intakeDeploy.getEncoder().setPosition(0.0); // reset encoder
      }
    } else {
      stowTimer = 0.0;
    }

    // Apply PID output if not holding
    if (!isHolding) {
      intakeDeploy.setVoltage(output);
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
    SmartDashboard.putBoolean("IntakeDeploy/isStowed", isStowed());
    SmartDashboard.putBoolean("IntakeDeploy/isDeploying", isDeploying);
    SmartDashboard.putBoolean("IntakeDeploy/isStowing", isStowing);
    SmartDashboard.putBoolean("IntakeDeploy/isHolding", isHolding);
  }

  @Override
  public void simulationPeriodic() {
    // WPILib sim
  }

  /** Optional agitator sequence */
  public Command deployAgitatorCommand() {
    return Commands.sequence(
        Commands.run(() -> intakeDeploy.setVoltage(-0.5), this).withTimeout(0.8),
        Commands.run(() -> intakeDeploy.setVoltage(0.3), this).withTimeout(0.4),
        stowCommand());
  }

  /** Homing command for robot init */
  public Command homeCommand() {
    return Commands.runOnce(() -> {
      if (!isStowed()) {
        stow(); // start PID to stow
      }
      intakeDeploy.getEncoder().setPosition(0.0); // reset encoder once stowed
    });
  }
}