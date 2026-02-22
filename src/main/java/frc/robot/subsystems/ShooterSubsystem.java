// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Creates a new Subsystem. */
public class ShooterSubsystem extends SubsystemBase {

  // Initialize the motor (Kraken direct drive CAN ID 61)
  private static final CANBus kCANBus = new CANBus("canivore");
  private final TalonFX shooter1 = new TalonFX(61, kCANBus);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // run shooter at percentage speed
  public void runShooter(double speed) {
    shooter1.setControl(dutyCycleRequest.withOutput(speed));
  }

  // run shooter at percentage speed
  public void stopShooter(double speed) {
    shooter1.setControl(dutyCycleRequest.withOutput(0));
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   *     <p>/** An example method querying a boolean state of the subsystem (for example, a digital
   *     sensor).
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
