package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public final class ClimbCommands {

  private ClimbCommands() {}

  // ============================================================
  // PID CONTROLLERS
  // ============================================================

  private static final PIDController forwardController =
      new PIDController(Constants.Climb.PID.kP_FORWARD, 0.0, 0.0);

  private static final PIDController strafeController =
      new PIDController(Constants.Climb.PID.kP_STRAFE, 0.0, 0.0);

  private static final PIDController turnController =
      new PIDController(Constants.Climb.PID.kP_TURN, 0.0, 0.0);

  static {
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ============================================================
  // CLIMBER COMMANDS
  // ============================================================

  public static Command waitForHome(ClimbSubsystem climb) {
    return Commands.waitUntil(climb::isHomed).withTimeout(2.0).withName("WaitForClimbHome");
  }

  public static Command hookUp(ClimbSubsystem climb) {
    return Commands.startEnd(climb::moveUp, climb::stop, climb).withName("HookUp");
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

  public static Command autoClimberUp(ClimbSubsystem climb) {
    return climb.upCommand().withName("ClimbAutoUp");
  }

  // ============================================================
  // FULL SEQUENCE
  // ============================================================

  public static Command climbSequence(Drive drive, Vision vision, ClimbSubsystem climb) {

    return Commands.sequence(
            DriveCommands.dockToClimb(drive, vision).withTimeout(4.0),
            hookUp(climb).withTimeout(1.8),
            Commands.waitSeconds(0.4),
            climbDown(climb).withTimeout(1.0),
            stop(climb))
        .withName("FullClimbSequence");
  }
}
