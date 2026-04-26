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
 * ClimbCommands (Alliance-aware + Phase 2 clean architecture)
 *
 * <p>Responsibility: - NO direct motor control - Climb logic stays in subsystem - Drive alignment
 * uses vision - Alliance-aware target selection
 */
public final class ClimbCommands {

  private ClimbCommands() {}

  // ============================================================
  // ALLIANCE-AWARE CLIMB TAG RESOLUTION
  // ============================================================

  private static int getClimbTagId() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Red vs Blue climb targets (adjust if field changes)
    return (alliance == Alliance.Red) ? 16 : 32;
  }

  // ============================================================
  // AUTO ALIGN TO CLIMB TAG (DRIVE ONLY)
  // ============================================================

  public static Command autoClimbDrive(Drive drive, Vision vision) {

    return Commands.run(
            () -> {
              int tagId = getClimbTagId();

              // If robot cannot see climb tag → do not move blindly
              if (!vision.hasTag(tagId)) {
                drive.runVelocity(new ChassisSpeeds(0, 0, 0));
                return;
              }

              // ================================
              // VISION ERROR INPUT (REAR CAMERA)
              // ================================
              double tx = vision.getTX(); // left/right error (deg)
              double ty = vision.getTY(); // forward/back proxy

              // ================================
              // TUNING GAINS (START HERE)
              // ================================
              final double kP_X = 0.05;
              final double kP_Y = 0.05;
              final double kP_ROT = 0.03;

              // ================================
              // CONTROL OUTPUT
              // ================================
              double strafe = tx * kP_X;
              double forward = ty * kP_Y;
              double omega = tx * kP_ROT;

              // ================================
              // DRIVE OUTPUT
              // ================================
              drive.runVelocity(new ChassisSpeeds(forward, strafe, omega));
            },
            drive)
        .withName("ClimbAutoAlignDrive");
  }

  // ============================================================
  // MANUAL CLIMB COMMANDS (STATE MACHINE WRAPPERS)
  // ============================================================

  public static Command climbUp(ClimbSubsystem climb) {
    return Commands.run(climb::moveUp, climb).withName("ClimbUp");
  }

  public static Command climbDown(ClimbSubsystem climb) {
    return Commands.run(climb::moveDown, climb).withName("ClimbDown");
  }

  public static Command stop(ClimbSubsystem climb) {
    return Commands.runOnce(climb::stop, climb).withName("ClimbStop");
  }

  // ============================================================
  // OVERRIDE CONTROL (SAFETY LAYER)
  // ============================================================

  public static Command enableManualOverride(ClimbSubsystem climb) {
    return Commands.runOnce(() -> climb.setManualOverride(true), climb)
        .withName("ClimbManualOverrideON");
  }

  public static Command disableManualOverride(ClimbSubsystem climb) {
    return Commands.runOnce(() -> climb.setManualOverride(false), climb)
        .withName("ClimbManualOverrideOFF");
  }

  // ============================================================
  // FULL CLIMB SEQUENCE (READY)
  // ============================================================

  public static Command climbSequence(Drive drive, Vision vision, ClimbSubsystem climb) {

    return Commands.sequence(

        // Lock climb subsystem first (prevents interference)
        enableManualOverride(climb),

        // Align under bar using rear camera
        autoClimbDrive(drive, vision).withTimeout(2.5),

        // Engage climb motion
        climbUp(climb).withTimeout(1.8),
        Commands.waitSeconds(0.5),

        // Controlled descent / hook adjustment
        climbDown(climb).withTimeout(1.0),

        // Stop everything
        stop(climb),

        // Restore normal robot behavior
        disableManualOverride(climb));
  }
}
