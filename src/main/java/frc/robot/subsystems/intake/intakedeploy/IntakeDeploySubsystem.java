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

  // NetworkTables tunables
  private final DoubleEntry deployPositionEntry =
      NetworkTableInstance.getDefault()
          .getTable("Tuning/Deploy")
          .getDoubleTopic("deployPosition")
          .getEntry(DEPLOY_POSITION);

  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;
  private final DoubleEntry holdVoltageEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("holdVoltage")
      .getEntry(0.2);

  private final DoubleEntry rampThresholdEntry =
      NetworkTableInstance.getDefault()
          .getTable("Tuning/Deploy")
          .getDoubleTopic("rampThreshold")
          .getEntry(1500);

  private final DoubleEntry positionThresholdEntry = NetworkTableInstance.getDefault()
      .getTable("Tuning/Deploy")
      .getDoubleTopic("positionThreshold")
      .getEntry(75);

  private double getDeployPosition() {
    return deployPositionEntry.get();
  }

  private final PIDController pid = new PIDController(0.005, 0, 0);

  private boolean isStowing = false;
  private boolean isDeploying = false;
  private boolean isHolding = false;

  private double stowTimer = 0.0;
  private static final double STOW_DEBOUNCE = 0.2; // seconds

  private double goalPosition = 0.0;

  public IntakeDeploySubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    intakeDeploy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(positionThresholdEntry.get());

    var table = NetworkTableInstance.getDefault().getTable("Tuning/Deploy");
    kPEntry = table.getDoubleTopic("kP").getEntry(0.005);
    kIEntry = table.getDoubleTopic("kI").getEntry(0.0);
    kDEntry = table.getDoubleTopic("kD").getEntry(0.0);

    kPEntry.set(Constants.IntakeDeploy.kP);
    kIEntry.set(Constants.IntakeDeploy.kI);
    kDEntry.set(Constants.IntakeDeploy.kD);
  }

  public void deploy() {
    goalPosition = getDeployPosition();
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
    return hallSensor.get();
  }

  public boolean isStowed() {
    return hallSensor.get();
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
                () ->
                    Math.abs(intakeDeploy.getEncoder().getPosition() * 360.0 - STOW_POSITION)
                        < positionThresholdEntry.get()));
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

    // // // Ramp down near stow
    // if (isStowing && Math.abs(distanceToGoal) < rampThresholdEntry.get()) {
    //   output *= 0.3;
    // }

    // Apply hold voltage only if very close
    if (isStowing && Math.abs(distanceToGoal) < positionThresholdEntry.get() && isDeployed()) {
      intakeDeploy.setVoltage(-holdVoltageEntry.get());
      isHolding = true;
      // isStowing = false;
    }

    //

    // // Soft debounce for stow completion
    // if (isStowing && Math.abs(position - STOW_POSITION) < positionThresholdEntry.get()) {
    //   stowTimer += 0.02; // periodic ~20ms
    //   if (stowTimer >= STOW_DEBOUNCE) {
    //     isStowing = false;
    //     isDeploying = false;
    //     goalPosition = STOW_POSITION;
    //     intakeDeploy.getEncoder().setPosition(0.0);
    //   }
    // } else {
    //   stowTimer = 0.0;
    // }

    intakeDeploy.setVoltage(output);

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
