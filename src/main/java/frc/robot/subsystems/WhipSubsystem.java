package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WhipSubsystem extends SubsystemBase {

  // Motor
  private final SparkMax whip = new SparkMax(57, MotorType.kBrushless);

  public WhipSubsystem(ShooterSubsystem shooterSubsystem) {
    whip.setInverted(true);
  }

  /** Direct control (used by shooter command) */
  public void set(double speed) {
    whip.set(speed);
  }

  /** Stop whip */
  public void stop() {
    whip.set(0.0);
  }

  /**
   * Command: run whip continuously at a fixed speed while scheduled (used inside shooter parallel
   * command)
   */
  public Command runWhipCommand() {
    return Commands.run(() -> whip.set(Constants.WHIP_SLOW_SPEED), this);
  }

  /** Command: stop whip */
  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Whip/Output", whip.get());
  }
}
