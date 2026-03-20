// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal
package frc.robot.subsystems.kicker;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

@SuppressWarnings({"unused"})
public class KickerSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
  SparkMax kicker = new SparkMax(62, MotorType.kBrushless);
  private final ShooterSubsystem m_shooter;

  /** Creates a new Subsystem. */
  @SuppressWarnings("deprecation")
  public KickerSubsystem(ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;

    kicker.setInverted(true);
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public final Command kickerCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          kicker(speed);
        });
  }

  public void kicker(double speed) {
    kicker.set(speed);
  }

  /** Stop command to set kicker speed to 0. */
  public Command stopCommand() {
    return Commands.runOnce(() -> kicker(0.0), this);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean kickerCondition() {
    return m_shooter.getVelocityRps() * 60 > Constants.SHOOTER_KICKER_RPM_THRESHOLD;
  }

  @Override
  public void periodic() {
    double shooterRps = m_shooter.getVelocityRps();
    boolean condition = shooterRps * 60 > Constants.SHOOTER_KICKER_RPM_THRESHOLD;
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Kicker/ShooterRPS", shooterRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Kicker/Condition", condition);
    double speed = condition ? -0.5 : 0.0;
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Kicker/SetSpeed", speed);
    kicker.set(speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
