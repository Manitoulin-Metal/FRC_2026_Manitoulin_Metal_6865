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

  public static final double STOW_POSITION = 0.0;
  public static final double DEPLOY_POSITION = 18000.0;

  // Tunables
  private final DoubleEntry deployPositionEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("deployPosition")
      .getEntry(DEPLOY_POSITION);

  private final DoubleEntry holdVoltageEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("holdVoltage")
      .getEntry(0.2);

  private final DoubleEntry rampThresholdEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("rampThreshold")
      .getEntry(1500);

  private final DoubleEntry positionThresholdEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("positionThreshold")
      .getEntry(75);

  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;

  private final PIDController pid = new PIDController(0.005, 0, 0);

  private boolean isStowing = false;
  private boolean isDeploying = false;
  private boolean isHolding = false;

  private double goalPosition = 0.0;

  public IntakeDeploySubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);

    intakeDeploy.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    var table = NetworkTableInstance.getDefault().getTable("Tuning/Deploy");

    // PID entries
    kPEntry = table.getDoubleTopic("kP").getEntry(Constants.IntakeDeploy.kP);
    kIEntry = table.getDoubleTopic("kI").getEntry(Constants.IntakeDeploy.kI);
    kDEntry = table.getDoubleTopic("kD").getEntry(Constants.IntakeDeploy.kD);

    // Other tunables
    holdVoltageEntry = table.getDoubleTopic("holdVoltage").getEntry(Constants.IntakeDeploy.holdVoltage);
    rampThresholdEntry = table.getDoubleTopic("rampThreshold").getEntry(Constants.IntakeDeploy.rampThreshold);
    positionThresholdEntry = table.getDoubleTopic("positionThreshold")
        .getEntry(Constants.IntakeDeploy.positionThreshold);

    // Force publish initial values
    kPEntry.set(Constants.IntakeDeploy.kP);
    kIEntry.set(Constants.IntakeDeploy.kI);
    kDEntry.set(Constants.IntakeDeploy.kD);
    holdVoltageEntry.set(Constants.IntakeDeploy.holdVoltage);
    rampThresholdEntry.set(Constants.IntakeDeploy.rampThreshold);
    positionThresholdEntry.set(Constants.IntakeDeploy.positionThreshold);

    pid.setTolerance(positionThresholdEntry.get());
  }

  private double getDeployPosition() {
    return deployPositionEntry.get();
  }

  public void deploy() {
    goalPosition = getDeployPosition();
    isHolding = false;
    isDeploying = true;
    isStowing = false;
  }

  public void stow() {
    goalPosition = STOW_POSITION;
    isHolding = false;
    isStowing = true;
    isDeploying = false;
  }

  public boolean isStowed() {
    return hallSensor.get();
  }

  public boolean atDeployPosition() {
    return Math.abs(pid.getPositionError()) < positionThresholdEntry.get();
  }

  public Command deployCommand() {
    return Commands.runOnce(this::deploy);
  }

  public Command stowCommand() {
    return Commands.runOnce(this::stow);
  }

  public Command homeCommand() {
    return Commands.either(
        Commands.runOnce(() -> {
          intakeDeploy.getEncoder().setPosition(0.0);
          goalPosition = STOW_POSITION;
          isHolding = true;
        }),
        Commands.sequence(
            stowCommand(),
            Commands.runOnce(() -> {
              intakeDeploy.getEncoder().setPosition(0.0);
              goalPosition = STOW_POSITION;
              isHolding = true;
            })),
        this::isStowed);
  }

  @Override
  public void periodic() {
    double position = intakeDeploy.getEncoder().getPosition() * 360.0;

    // ---------- HOLD MODE ----------
    // automatically release hold if moved away from stow
    if (isHolding && Math.abs(position - STOW_POSITION) > positionThresholdEntry.get()) {
      isHolding = false;
    }
    // If holding, just apply a constant voltage to hold position and skip PID
    if (isHolding) {
      intakeDeploy.setVoltage(-holdVoltageEntry.get());

      SmartDashboard.putBoolean("IntakeDeploy/isHolding", true);
      SmartDashboard.putNumber("IntakeDeploy/Position", position);
      return;
    }

    // ---------- PID ----------
    pid.setSetpoint(goalPosition);

    pid.setP(kPEntry.get());
    pid.setI(kIEntry.get());
    pid.setD(kDEntry.get());

    double distanceToGoal = goalPosition - position;

    double output = pid.calculate(position);

    // Slow down near stow
    if (isStowing && Math.abs(distanceToGoal) < rampThresholdEntry.get()) {
      output *= 0.3;
    }

    // ---------- SWITCH TO HOLD ----------
    if (isStowing
        && Math.abs(distanceToGoal) < positionThresholdEntry.get()
        && isStowed()) {

      isHolding = true;
      isStowing = false;
      isDeploying = false;

      goalPosition = STOW_POSITION;
      intakeDeploy.getEncoder().setPosition(0.0);

      intakeDeploy.setVoltage(-holdVoltageEntry.get());

      SmartDashboard.putBoolean("IntakeDeploy/isHolding", true);
      return;
    }

    intakeDeploy.setVoltage(output);

    // ---------- LOGGING ----------
    Logger.recordOutput("IntakeDeploy/Position", position);
    Logger.recordOutput("IntakeDeploy/PIDError", pid.getPositionError());
    Logger.recordOutput("IntakeDeploy/PIDOutput", output);
    Logger.recordOutput("IntakeDeploy/Goal", goalPosition);

    SmartDashboard.putNumber("IntakeDeploy/Position", position);
    SmartDashboard.putNumber("IntakeDeploy/PIDError", pid.getPositionError());
    SmartDashboard.putNumber("IntakeDeploy/PIDOutput", output);
    SmartDashboard.putNumber("IntakeDeploy/Goal", goalPosition);

    SmartDashboard.putBoolean("IntakeDeploy/isStowed", isStowed());
    SmartDashboard.putBoolean("IntakeDeploy/isStowing", isStowing);
    SmartDashboard.putBoolean("IntakeDeploy/isDeploying", isDeploying);
    SmartDashboard.putBoolean("IntakeDeploy/isHolding", isHolding);
  }

  @Override
  public void simulationPeriodic() {
  }
}