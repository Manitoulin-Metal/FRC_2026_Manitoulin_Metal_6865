// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems.intake.intakedeploy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("removal")
public class IntakeDeploySubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
  SparkMax intakeDeploy = new SparkMax(59, MotorType.kBrushless);

  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pidController;

  /** Creates a new Subsystem. */
  public IntakeDeploySubsystem() {
    intakeDeploy.restoreFactoryDefaults();
    intakeDeploy.setInverted(true);
    intakeDeploy.setIdleMode(IdleMode.kBrake);

    // PID setup
    encoder = intakeDeploy.getEncoder();

    encoder.setPositionConversionFactor(1.0); // Assume 1:1, tune gearing

    encoder.setVelocityConversionFactor(1.0 / 60.0); // RPS

    pidController = intakeDeploy.getPIDController();
    pidController.setP(Constants.IntakeDeploy.kP);
    pidController.setI(Constants.IntakeDeploy.kI);
    pidController.setD(Constants.IntakeDeploy.kD);
    pidController.setFF(Constants.IntakeDeploy.kFF);
    pidController.setFeedbackDevice(encoder);
  }

  public void deploy() {
    pidController.setReference(Constants.IntakeDeploy.DEPLOY_SETPOINT_ROT, ControlType.kPosition);
  }

  public void stow() {
    pidController.setReference(Constants.IntakeDeploy.STOW_SETPOINT_ROT, ControlType.kPosition);
  }

  public boolean atDeployPosition() {
    return Math.abs(encoder.getPosition() - Constants.IntakeDeploy.DEPLOY_SETPOINT_ROT)
        <= Constants.IntakeDeploy.POSITION_TOLERANCE_ROT;
  }

  public boolean atStowPosition() {
    return Math.abs(encoder.getPosition() - Constants.IntakeDeploy.STOW_SETPOINT_ROT)
        <= Constants.IntakeDeploy.POSITION_TOLERANCE_ROT;
  }

  public Command deployCommand() {
    return Commands.runOnce(this::deploy).andThen(Commands.waitUntil(this::atDeployPosition));
  }

  public Command stowCommand() {
    return Commands.runOnce(this::stow).andThen(Commands.waitUntil(this::atStowPosition));
  }

  // Legacy speed control
  @Deprecated
  public void runIntakeDeploy(double speed) {
    intakeDeploy.set(speed);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("IntakeDeploy/PositionRot", encoder.getPosition());
    Logger.recordOutput("IntakeDeploy/VelocityRPS", encoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // Simulation
  }
}
