// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

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
  // Initialize the motor (Flex/MAX are setup the same way)

  SparkFlex climb1 = new SparkFlex(60, MotorType.kBrushless);
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Climb.LIMIT_SWITCH_CHANNEL);

  /** Creates a new Subsystem. */
  @SuppressWarnings("removal")
  public ClimbSubsystem() {
    SparkFlexConfig config4 = new SparkFlexConfig();

    config4.idleMode(IdleMode.kBrake);

    climb1.configure(config4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public Command StopClimbCommand(double speed) {
    return run(
        () -> {
          stopClimber(0);
        });
  }

  public Command ClimbCommand(double speed) {
    System.out.println("Climb Is Running");
    if (speed > 0 && !limitSwitch.get()) { // !get() = pressed (active low)
      climb1.set(0);
    } else {
      climb1.set(speed);
    }
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          runClimber(speed);
        });
  }

  public void runClimber(double speed) {
    if (speed > 0 && !limitSwitch.get()) { // !get() = pressed (active low)
      climb1.set(0);
    } else {
      climb1.set(speed);
    }
  }

  public void stopClimber(double speed) {
    // Stop the climber motor immediately
    climb1.set(0);
  }

  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  // public boolean IntakeDeployCondition() {
  // Query some boolean state, such as a digital sensor.
  // If needed add IntakeDeploy command.
  // return false;
  // }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb/LimitSwitchPressed", isLimitSwitchPressed());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
