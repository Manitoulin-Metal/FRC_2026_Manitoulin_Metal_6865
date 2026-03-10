// This is being used by Team 6865, Manitoulin Metal

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Command helpers for the climb subsystem. This class follows the same pattern as DriveCommands -
 * static methods that create commands using the subsystem.
 */

public class ClimbCommand {

  private ClimbCommand() {
    // Utility class - no instantiation
  }

  /**
   * Creates a command that runs the climber at a given speed.
   *
   * @param climbSubsystem The climb subsystem to use
   * @param speed The speed (-1.0 to 1.0) to run the climber
   * @return A command that runs the climber
   */
  
  public static Command runClimber(ClimbSubsystem climbSubsystem, double speed) {
    return run(
        () -> {
          climbSubsystem.runClimber(speed);
        },
        climbSubsystem);
  }

  /**
   * Creates a command that stops the climber.
   *
   * @param climbSubsystem The climb subsystem to use
   * @return A command that stops the climber
   */

  public static Command stopClimber(ClimbSubsystem climbSubsystem) {
    return run(
        () -> {
          climbSubsystem.runClimber(0.0);
        },
        climbSubsystem);
  }
}
