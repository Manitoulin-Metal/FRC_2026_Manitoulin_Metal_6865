// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems.intake.intakeroller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
  SparkFlex IntakeRoller = new SparkFlex(58, MotorType.kBrushless);

  /** Creates a new Subsystem. */
  public IntakeRollerSubsystem() {
    SparkMaxConfig config4 = new SparkMaxConfig();

    config4.idleMode(IdleMode.kBrake);

    IntakeRoller.configure(config4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public Command IntakeRollerCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          runIntakeRoller(speed);
        });
  }

  private void runIntakeRoller(double speed) {
    throw new UnsupportedOperationException("Unimplemented method 'runIntakeRoller'");
  }

  {
  }

  // May need to add set speed code here

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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
