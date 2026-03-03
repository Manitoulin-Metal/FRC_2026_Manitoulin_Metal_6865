// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("removal")
public class IntakeDeploySubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)

  SparkMax intakeDeploy = new SparkMax(59, MotorType.kBrushless);

  /** Creates a new Subsystem. */
  public IntakeDeploySubsystem() {

    // lc.setRangingMode.LaserCan.RangingMode.SHORT lc; setRegionOfInterest(new
    // LaserCan.RegionOfInterest(8, 8, 16, 16));
    // lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS ); lc.setRegionOfInterest(new
    // LaserCan.RegionOfInterest(8, 8, 16, 16));
    // lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS getLaserMeasurement() {
    // LaserCan.Measurement measurement = lc.getMeasurement();
    // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
    // {
    // return measurement.distance_mm;);

  }

  {
    SparkMaxConfig config3 = new SparkMaxConfig();
    config3.inverted(true).idleMode(IdleMode.kBrake);

    // Apply configs - reset old parameters, and persist through power-cycles.
    intakeDeploy.configure(
        config3, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    {
    }
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   * @return a command
   */

  public final Command IntakeDeployCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          runIntakeDeploy(speed);
        });
  }

  public void runIntakeDeploy(double speed) {
    intakeDeploy.set(speed);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  
  public boolean IntakeDeployCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
