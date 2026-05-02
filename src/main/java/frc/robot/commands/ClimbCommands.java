package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/**
 * Clean ClimbCommands
 *
 * <p>Responsibility: - Only REQUEST actions - No state logic - No override systems - Subsystem owns
 * all safety + limits
 */
public final class ClimbCommands {

  private ClimbCommands() {}

  // ============================================================
  // ALLIANCE TAG SELECTION
  // ============================================================

  private static int getClimbTagId() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    return (alliance == Alliance.Red) ? 16 : 32;
  }

  // ============================================================
  // DRIVE ALIGNMENT (VISION ONLY)
  // ============================================================

  public static Command autoClimbDrive(Drive drive, Vision vision) {

    return Commands.run(
            () -> {
              int tagId = getClimbTagId();

              if (!vision.hasTag(tagId)) {
                drive.runVelocity(new ChassisSpeeds(0, 0, 0));
                return;
              }

              double tx = vision.getTX();
              double ty = vision.getTY();

              final double kP_X = 0.05;
              final double kP_Y = 0.05;
              final double kP_ROT = 0.03;

              double strafe = tx * kP_X;
              double forward = ty * kP_Y;
              double omega = tx * kP_ROT;

              drive.runVelocity(new ChassisSpeeds(forward, strafe, omega));
            },
            drive)
        .finallyDo(drive::stop)
        .withName("ClimbAutoAlignDrive");
  }

  // ============================================================
  // CLIMB ACTIONS (DIRECT STATE REQUESTS ONLY)
  // ============================================================

  public static Command autoClimberUp(ClimbSubsystem climb) {
    return climb.upCommand().withName("ClimbAutoUp");
    // return Commands.runOnce(climb::moveUp, climb).withName("ClimbAutoUp");
  }

  public static Command climbUp(ClimbSubsystem climb) {
    System.out.println("Climb Up Command Created");
    return Commands.startEnd(climb::moveUp, climb::stop, climb).withName("ClimbUp");
  }

  public static Command climbDown(ClimbSubsystem climb) {
    return Commands.startEnd(climb::moveDown, climb::stop, climb).withName("ClimbDown");
  }

  public static Command stop(ClimbSubsystem climb) {
    return Commands.runOnce(climb::stop, climb).withName("ClimbStop");
  }

  public static Command home(ClimbSubsystem climb) {
    return climb.homeCommand().withName("ClimbHome");
  }

  // ============================================================
  // FULL AUTO CLIMB SEQUENCE
  // ============================================================

  public static Command climbSequence(Drive drive, Vision vision, ClimbSubsystem climb) {

    return Commands.sequence(

            // Align under bar
            autoClimbDrive(drive, vision).withTimeout(2.5),

            // Raise hook
            climbUp(climb).withTimeout(1.8),

            // settle time
            Commands.waitSeconds(0.4),

            // adjust / descend
            climbDown(climb).withTimeout(1.0),

            // ensure stop
            stop(climb))
        .withName("FullClimbSequence");
  }
}
