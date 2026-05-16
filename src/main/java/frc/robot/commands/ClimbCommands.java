package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

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
  // ALLIANCE HELPERS
  // ============================================================

  private static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  private static boolean isRed() {
    return alliance() == Alliance.Red;
  }

  // ONE unified yaw target (no contradictions)
  private static Rotation2d targetYaw() {
    // Flip if your physical mounting requires it
    return isRed() ? Rotation2d.kZero : Rotation2d.fromDegrees(180);
  }

  // ============================================================
  // FILTER STATE
  // ============================================================

  private static double filteredTX = 0.0;
  private static double filteredTY = 0.0;

  private static void updateFilter(double tx, double ty) {
    filteredTX = 0.8 * filteredTX + 0.2 * tx;
    filteredTY = 0.8 * filteredTY + 0.2 * ty;
  }

  // ============================================================
  // SIMPLE ALIGN (FIXED)
  // ============================================================

  public static Command autoClimbDrive(Drive drive, Vision vision) {

    return Commands.run(
            () -> {
              int targetTag = isRed() ? 16 : 32;

              if (!vision.hasTag(targetTag)) {
                drive.stop();
                return;
              }

              double tx = vision.getTX();
              double ty = vision.getTY();

              updateFilter(tx, ty);

              double kP_X = 0.05;
              double kP_Y = 0.05;
              double kP_ROT = 0.03;

              double vx = filteredTY * kP_Y;
              double vy = filteredTX * kP_X;

              double yawError = drive.getRotation().minus(targetYaw()).getRadians();

              double omega = yawError * kP_ROT;

              drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
            },
            drive)
        .finallyDo(drive::stop)
        .withName("ClimbAutoAlignDrive");
  }

  // ============================================================
  // MAIN DOCKING COMMAND (CLEAN + CONSISTENT)
  // ============================================================

  public static Command dockToClimb(Drive drive, Vision vision) {

    return Commands.run(
            () -> {
              if (!LimelightHelpers.getTV(Constants.Climb.Vision.REAR_LIMELIGHT)) {
                drive.stop();
                return;
              }

              double tx = LimelightHelpers.getTX(Constants.Climb.Vision.REAR_LIMELIGHT);
              double ty = LimelightHelpers.getTY(Constants.Climb.Vision.REAR_LIMELIGHT);

              updateFilter(tx, ty);

              double targetTX = Constants.Climb.Vision.targetTX();
              double targetTY = Constants.Climb.Vision.targetTY();

              double txError = filteredTX - targetTX;
              double tyError = filteredTY - targetTY;

              double yawError = drive.getRotation().minus(targetYaw()).getRadians();

              Logger.recordOutput("ClimbDock/TX", filteredTX);
              Logger.recordOutput("ClimbDock/TY", filteredTY);
              Logger.recordOutput("ClimbDock/TXError", txError);
              Logger.recordOutput("ClimbDock/TYError", tyError);
              Logger.recordOutput("ClimbDock/YawError", yawError);

              double vx = forwardController.calculate(tyError, 0.0);
              double vy = strafeController.calculate(txError, 0.0);
              double omega = turnController.calculate(yawError, 0.0);

              double distance = Math.hypot(txError, tyError);

              if (distance > 3.0) {
                omega = 0.0;
              }

              vx = MathUtil.clamp(vx, -0.8, 0.8);
              vy = MathUtil.clamp(vy, -0.8, 0.8);
              omega = MathUtil.clamp(omega, -1.0, 1.0);

              if (Math.abs(vx) < 0.03) vx = 0.0;
              if (Math.abs(vy) < 0.03) vy = 0.0;
              if (Math.abs(omega) < 0.03) omega = 0.0;

              drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
            },
            drive)
        .until(
            () -> {
              if (!LimelightHelpers.getTV(Constants.Climb.Vision.REAR_LIMELIGHT)) {
                return false;
              }

              double tx = filteredTX;
              double ty = filteredTY;

              double txError = Math.abs(tx - Constants.Climb.Vision.targetTX());
              double tyError = Math.abs(ty - Constants.Climb.Vision.targetTY());

              double yawError = Math.abs(drive.getRotation().minus(targetYaw()).getRadians());

              return txError < 1.0 && tyError < 1.0 && yawError < 0.08;
            })
        .finallyDo(drive::stop)
        .withName("DockToClimb");
  }

  // ============================================================
  // CLIMBER COMMANDS (UNCHANGED LOGICALLY)
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
            dockToClimb(drive, vision).withTimeout(4.0),
            hookUp(climb).withTimeout(1.8),
            Commands.waitSeconds(0.4),
            climbDown(climb).withTimeout(1.0),
            stop(climb))
        .withName("FullClimbSequence");
  }
}
