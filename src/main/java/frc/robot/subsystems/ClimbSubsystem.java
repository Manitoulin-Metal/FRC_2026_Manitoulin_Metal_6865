package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkFlex climbMotor = new SparkFlex(60, MotorType.kBrushless);
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Climb.LIMIT_SWITCH_CHANNEL);

  public ClimbSubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);

    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ========================= MOTOR CONTROL =========================

  public void runClimber(double speed) {

    boolean pressed = isLimitSwitchPressed();

    // DEBUG (watch this live!)
    SmartDashboard.putBoolean("Climb/LimitSwitchPressed", pressed);
    SmartDashboard.putNumber("Climb/SpeedCommand", speed);

    if (pressed && speed > 0) {
      climbMotor.stopMotor(); // stronger than set(0)
      return;
    }

    climbMotor.set(speed);
  }

  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get(); // active low
  }

  // ========================= COMMANDS =========================

  public Command climbCommand(double speed) {
    return run(() -> {
          boolean pressed = isLimitSwitchPressed();

          // Stop downward motion if limit switch pressed
          if (pressed && speed < 0) {
            climbMotor.stopMotor();
          } else {
            climbMotor.set(speed);
          }
        })
        .finallyDo(interrupted -> climbMotor.stopMotor());
  }
}
