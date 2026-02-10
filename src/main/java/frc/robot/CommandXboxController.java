package frc.robot;

/** Custom wrapper for WPILib CommandXboxController to add any team-specific functionality. */
public class CommandXboxController
    extends edu.wpi.first.wpilibj2.command.button.CommandXboxController {

  public CommandXboxController(int port) {
    super(port);
  }

  // Add any custom methods or overrides here if needed
}
