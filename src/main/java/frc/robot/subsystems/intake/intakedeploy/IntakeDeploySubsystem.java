// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("removal")
public class IntakeDeploySubsystem extends SubsystemBase {
  // Initialize the motor (Flex API - matches IntakeRollerSubsystem)
  private final SparkFlex intakeDeploy = new SparkFlex(59, MotorType.kBrushless);

  public static final double STOW_POSITION = 0.0;
  public static final double DEPLOY_POSITION = 10000.0; // degrees, tune
  private final DoubleEntry kP;
  private final DoubleEntry kI;
  private final DoubleEntry kD;
  private final PIDController pid = new PIDController(0.005, 0, 0); // P, I, D gains - tune

  /** Creates a new Subsystem. */
  public IntakeDeploySubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);

    intakeDeploy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(3.0); // position tolerance (degrees)
    var table = NetworkTableInstance.getDefault().getTable("Tuning/Deploy");

    kP = table.getDoubleTopic("kP").getEntry(0.005);
    kI = table.getDoubleTopic("kI").getEntry(0.0);
    kD = table.getDoubleTopic("kD").getEntry(0.0);

    kP.set(0.005);
    kI.set(0.0);
    kD.set(0.0);
  }

  private double goalPosition = 0.0;

  public void deploy() {
    goalPosition = DEPLOY_POSITION;
  }

  public void stow() {
    goalPosition = STOW_POSITION;
  }

  // private void runPID() {
  //   double position = intakeDeploy.getEncoder().getPosition() * 360.0; // rotations to degrees
  //   pid.setSetpoint(goalPosition);
  //   double output = pid.calculate(position);
  //   intakeDeploy.setVoltage(output);
  // }

  public boolean atDeployPosition() {
    return Math.abs(pid.getPositionError()) < 3.0;
  }

  public boolean atStowPosition() {
    return Math.abs(pid.getPositionError()) < 3.0;
  }

  public Command deployCommand() {
    return Commands.runOnce(this::deploy).andThen(Commands.waitUntil(this::atDeployPosition));
  }

  public Command stowCommand() {
    return Commands.runOnce(this::stow).andThen(Commands.waitUntil(this::atStowPosition));
  }

  @Override
  public void periodic() {
    // double position = intakeDeploy.getEncoder().getPosition() * 360.0;
    // runPID();
    double position = intakeDeploy.getEncoder().getPosition() * 360.0; // rotations to degrees
    pid.setSetpoint(goalPosition);

    pid.setP(kP.get());
    pid.setI(kI.get());
    pid.setD(kD.get());

    double output = pid.calculate(position);
    intakeDeploy.setVoltage(output);

    Logger.recordOutput("IntakeDeploy/Position", position);
    SmartDashboard.putNumber("IntakeDeploy/Position", position);
    Logger.recordOutput("IntakeDeploy/PIDError", pid.getPositionError());
    SmartDashboard.putNumber("IntakeDeploy/PIDError", pid.getPositionError());
    Logger.recordOutput(
        "IntakeDeploy/PIDOutput", 0.0); // Capture from runPID if needed: store output var
    SmartDashboard.putNumber(
        "IntakeDeploy/PIDOutput", 0.0); // Capture from runPID if needed: store output var
    Logger.recordOutput("IntakeDeploy/Goal", goalPosition);
    SmartDashboard.putNumber("IntakeDeploy/Goal", goalPosition);
  }

  @Override
  public void simulationPeriodic() {
    // Simulation handled by WPILib sim
  }
}
