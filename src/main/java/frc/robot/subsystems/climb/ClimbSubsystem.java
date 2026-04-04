package frc.robot.subsystems.climb;

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

    // Prevent upward movement if limit switch is pressed
    if (speed > 0 && isLimitSwitchPressed()) {
      climbMotor.set(0);
    } else {
      climbMotor.set(speed);
    }
  }

  public void stopClimber() {
    climbMotor.set(0);
  }

  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get(); // active low
  }

  // ========================= COMMANDS =========================

  public Command climbCommand(double speed) {
    return run(() -> runClimber(speed)).finallyDo(() -> stopClimber());
  }

  public Command stopCommand() {
    return runOnce(this::stopClimber);
  }

  // ========================= DEBUG =========================

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb/LimitSwitchPressed", isLimitSwitchPressed());
  }
}
