package frc.robot.subsystems.Whip;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WhipSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
  SparkMax whip = new SparkMax(57, MotorType.kBrushless);
  private final ShooterSubsystem m_shooter;

  /** Creates a new Subsystem. */
  public WhipSubsystem(ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;

    whip.setInverted(true);
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public final Command whipSlowCommand() {
    System.out.println("Whip is Running");
    setManualMode(true);
    return Commands.run(() -> whip(Constants.WHIP_SLOW_SPEED), this).finallyDo(this::stopWhip);
  }

  private void stopWhip() {
    setManualMode(false);
    whip(0.0);
  }

  public Command whipStopCommand() {
    System.out.println("Whip Has Stopped");
    setManualMode(false);
    return stopCommand();
  }

  public void whip(double speed) {
    whip.set(speed);
  }

  /** Stop command to set kicker speed to 0. */
  public Command stopCommand() {
    return Commands.runOnce(() -> whip(0.0), this);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean whipCondition() {
    return m_shooter.getVelocityRps() * 60 > Constants.SHOOTER_WHIP_RPM_THRESHOLD;
  }

  private boolean manualMode = false;
  private final LoggedNetworkNumber whipAutoSpeedEntry =
      new LoggedNetworkNumber("Tuning/Whip/AutoSpeed", -0.5);

  public void setManualMode(boolean mode) {
    this.manualMode = mode;
  }

  public boolean isManualMode() {
    return manualMode;
  }

  @Override
  public void periodic() {
    if (manualMode) {
      return;
    }
    double shooterRps = m_shooter.getVelocityRps();
    boolean condition = shooterRps * 60 > Constants.SHOOTER_WHIP_RPM_THRESHOLD;
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Whip/ShooterRPS", shooterRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Whip/Condition", condition);
    double speed = condition ? whipAutoSpeedEntry.get() : 0.0;
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Whip/SetSpeed", speed);
    whip.set(speed);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Whip/ManualMode", manualMode);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
