package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class SimpleDriveAndSpinAuto extends SequentialCommandGroup {

  public SimpleDriveAndSpinAuto(Drive drive) {

    addCommands(
        // Drive forward for 1 second
        Commands.run(
                () ->
                    drive.runVelocity(
                        new ChassisSpeeds(
                            1.0, // vx meters/sec (forward)
                            0.0, // vy
                            0.0 // omega rad/sec
                            )),
                drive)
            .withTimeout(1.0),

        // Spin in place for 2 seconds
        Commands.run(
                () ->
                    drive.runVelocity(
                        new ChassisSpeeds(
                            0.0, 0.0, Math.toRadians(45.0) // slow rotation
                            )),
                drive)
            .withTimeout(2.0),

        // Stop
        Commands.runOnce(drive::stop, drive));
  }
}
