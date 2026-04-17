// Team 6865, Manitoulin Metal
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {

  // Motor
  private final SparkMax kicker = new SparkMax(62, MotorType.kBrushless);
  private final ShooterSubsystem m_shooter;
  private final Timer readyTimer = new Timer();
  private boolean wasReady = false;

  /** Creates a new Kicker subsystem. */
  @SuppressWarnings("deprecation")
  public KickerSubsystem(ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;

    // Invert motor if needed
    kicker.setInverted(true);
  }

  /** Returns true if the shooter is fast enough to allow the kicker to run. */
  public boolean kickerCondition() {
    return m_shooter.atTarget();
  }

  /** Runs the kicker at constant speed if the shooter is ready. */
  public void kicker() {
    boolean isReady = kickerCondition();

    if (isReady) {
      // Shooter just became ready → start timer
      if (!wasReady) {
        readyTimer.reset();
        readyTimer.start();
      }

      // Only run after 1 second delay
      if (readyTimer.hasElapsed(1.0)) {
        kicker.set(0.5);
      } else {
        kicker.set(0.0);
      }

    } else {
      // Reset everything if shooter falls out of range
      readyTimer.stop();
      readyTimer.reset();
      kicker.set(0.0);
    }

    wasReady = isReady;
    if (kickerCondition()) {
      kicker.set(0.5);
    } else {
      kicker.set(0.0);
    }
  }

  /** Command to continuously run the kicker based on shooter velocity. */
  public final Command kickerCommand() {
    return run(this::kicker);
  }

  /** Command to immediately stop the kicker. */
  public Command stopCommand() {
    return Commands.runOnce(() -> kicker.set(0.0), this);
  }

  @Override
  public void periodic() {
    // Logging only — do not control motor here
    double shooterRps = m_shooter.getVelocityRps();
    boolean condition = kickerCondition();

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Kicker/ShooterRPS", shooterRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Kicker/Condition", condition);
    double speed = condition ? 0.5 : 0.0;
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Kicker/SetSpeed", speed);
    kicker.set(speed);
  }

  @Override
  public void simulationPeriodic() {
    // No simulation behavior needed
  }
}
