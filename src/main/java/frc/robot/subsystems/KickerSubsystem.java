// Team 6865, Manitoulin Metal
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {

  private final SparkMax kicker = new SparkMax(62, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public KickerSubsystem(ShooterSubsystem shooterSubsystem) {
    kicker.setInverted(true);
  }

  /** Direct motor control */
  public void setKicker(double speed) {
    kicker.set(speed);
  }

  /** Stop */
  public void stop() {
    kicker.set(0.0);
  }

  /** Command-based stop */
  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Kicker/Output", kicker.get());
  }
}
