// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.Logger;

/** Creates a new Subsystem. */
public class ShooterSubsystem extends SubsystemBase {

  // Initialize the motor (Kraken direct drive CAN ID 61)

  private static final CANBus kCANBus = new CANBus("DriveCanivore");
  private final TalonFX shooter = new TalonFX(61, kCANBus);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage stopRequest = new VelocityVoltage(0);

  private final LoggedNetworkNumber kPEntry = Constants.Shooter.kPEntry;
  private final LoggedNetworkNumber kIEntry = Constants.Shooter.kIEntry;
  private final LoggedNetworkNumber kDEntry = Constants.Shooter.kDEntry;
  private final LoggedNetworkNumber kVEntry = Constants.Shooter.kVEntry;
  private final LoggedNetworkNumber kSEntry = Constants.Shooter.kSEntry;
  private double targetRps = 0.0;

  public ShooterSubsystem() {
    Slot0Configs speedConfig = new Slot0Configs();
    speedConfig.kS = Constants.Shooter.kS;
    speedConfig.kV = Constants.Shooter.kV;
    speedConfig.kP = Constants.Shooter.kP;
    speedConfig.kI = Constants.Shooter.kI;
    speedConfig.kD = Constants.Shooter.kD;

    shooter.getConfigurator().apply(speedConfig);
  }

  /** Run shooter at velocity RPS */
  public void runShooter(double speedRps) {
    targetRps = speedRps;
    shooter.setControl(velocityRequest.withVelocity(speedRps));
  }

  /** Stop shooter */
  public void stopShooter() {
    shooter.setControl(stopRequest);
  }

  /** One-shot command: immediately spins shooter at given speed */
  public Command shootCommand(double speed) {
    return Commands.runOnce(() -> runShooter(speed), this);
  }

  /** Timed shoot: spins shooter at given speed for 15 seconds */
  public Command timedShootCommand(double speed) {
    return Commands.runEnd(
            () -> runShooter(speed), // start shooting
            this::stopShooter, // stop shooting after finished
            this // requires this subsystem
            )
        .withTimeout(15.0);
  }

  /** Stop command: stops shooter instantly */
  public Command stopCommand() {
    return Commands.runOnce(this::stopShooter, this);
  }

@Override
  public void periodic() {
    // Live PID/FF tuning updates (like IntakeRoller)
    updatePIDIfChanged();

    double velocityRps = getVelocityRps();
    double error = Math.abs(targetRps - velocityRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/VelocityRPS", velocityRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/TargetRPS", targetRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/PIDError", error);
    Logger.recordOutput("Shooter/VelocityRPS", velocityRps);
    Logger.recordOutput("Shooter/TargetRPS", targetRps);
    Logger.recordOutput("Shooter/PIDError", error);
  }

  private void updatePIDIfChanged() {
    double newKP = kPEntry.get();
    double newKI = kIEntry.get();
    double newKD = kDEntry.get();
    double newKV = kVEntry.get();
    double newKS = kSEntry.get();

    Slot0Configs config = new Slot0Configs();
    config.kP = newKP;
    config.kI = newKI;
    config.kD = newKD;
    config.kV = newKV;
    config.kS = newKS;

    shooter.getConfigurator().apply(config);
  }

  /** Get current shooter velocity in rotations per second (RPS) */
  public double getVelocityRps() {
    return shooter.getVelocity().getValueAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
