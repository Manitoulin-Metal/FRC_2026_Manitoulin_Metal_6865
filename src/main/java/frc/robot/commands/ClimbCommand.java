package frc.robot.commands;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbCommand extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)

  SparkFlex climb1 = new SparkFlex(60, MotorType.kBrushless);

  /** Creates a new Subsystem. */
  @SuppressWarnings("removal")

  public ClimbCommand() {
    SparkMaxConfig config4 = new SparkMaxConfig();

    config4.idleMode(IdleMode.kBrake);

    climb1.configure(config4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   /**
    * Sets motor controllers to run-to-pos based off distance
    * @return a command
    */

   public Command createClimbCommand(double speed) {
     // Inline construction of command goes here.
     // Subsystem::RunOnce implicitly requires `this` subsystem.
     return run(
         () -> {
           runClimber(speed);
         });
   }

   public void runClimber(double speed) {
     climb1.set(speed);
   }

   /**
    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
    * @return value of some boolean subsystem state, such as a digital sensor.
    */
   
   // public boolean IntakeDeployCondition() {
   // Query some boolean state, such as a digital sensor.
   // If needed add IntakeDeploy command.
   // return false;
   // }
    
}
