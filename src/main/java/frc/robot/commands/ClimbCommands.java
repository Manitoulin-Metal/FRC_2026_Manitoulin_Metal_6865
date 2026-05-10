package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
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

  public static Command waitForHome(ClimbSubsystem climb) {
    return Commands.waitUntil(climb::isHomed).withTimeout(2.0);
  }

  public static Command autoClimberUp(ClimbSubsystem climb) {
    return climb.upCommand().withName("ClimbAutoUp");
    // return Commands.runOnce(climb::moveUp, climb).withName("ClimbAutoUp");
  }

  public static Command dockToClimb(Drive drive, Vision vision) {

    turnController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              var poseOpt = vision.getRearTargetSpacePose();

              if (poseOpt.isEmpty()) {
                drive.stop();
                return;
              }

              Pose3d pose = poseOpt.get();

              double forwardError = pose.getZ() - Constants.Climb.Vision.TARGET_FORWARD_METERS;

              double lateralError = pose.getX() - Constants.Climb.Vision.TARGET_LATERAL_METERS;

              double yawError =
                  pose.getRotation()
                      .toRotation2d()
                      .minus(Constants.Climb.Vision.TARGET_YAW)
                      .getRadians();

              double vx = -forwardController.calculate(forwardError, 0.0);

              double vy = -strafeController.calculate(lateralError, 0.0);

              double omega = -turnController.calculate(yawError, 0.0);

              // Precision mode
              double distance = Math.hypot(forwardError, lateralError);

              double maxLinear =
                  distance < Constants.Climb.Vision.PRECISION_MODE_DISTANCE
                      ? Constants.Climb.Vision.PRECISION_LINEAR_SPEED
                      : Constants.Climb.Vision.MAX_LINEAR_SPEED;

              vx = MathUtil.clamp(vx, -maxLinear, maxLinear);

              vy = MathUtil.clamp(vy, -maxLinear, maxLinear);

              omega =
                  MathUtil.clamp(
                      omega,
                      -Constants.Climb.Vision.MAX_ANGULAR_SPEED,
                      Constants.Climb.Vision.MAX_ANGULAR_SPEED);

              // Deadbands
              if (Math.abs(vx) < 0.03) vx = 0.0;
              if (Math.abs(vy) < 0.03) vy = 0.0;
              if (Math.abs(omega) < 0.03) omega = 0.0;

              drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
            },
            drive)
        .until(
            () -> {
              var poseOpt = vision.getRearTargetSpacePose();

              if (poseOpt.isEmpty()) {
                return false;
              }

              Pose3d pose = poseOpt.get();

              double forwardError =
                  Math.abs(pose.getZ() - Constants.Climb.Vision.TARGET_FORWARD_METERS);

              double lateralError =
                  Math.abs(pose.getX() - Constants.Climb.Vision.TARGET_LATERAL_METERS);

              double yawError =
                  Math.abs(
                      pose.getRotation()
                          .toRotation2d()
                          .minus(Constants.Climb.Vision.TARGET_YAW)
                          .getRadians());

              return forwardError < Constants.Climb.Vision.FORWARD_TOLERANCE
                  && lateralError < Constants.Climb.Vision.LATERAL_TOLERANCE
                  && yawError < Constants.Climb.Vision.YAW_TOLERANCE_RAD;
            })
        .finallyDo(interrupted -> drive.stop());
  }
  // public static Command climbUp(ClimbSubsystem climb) {
  //   System.out.println("Climb Up Command Created");
  //   return Commands.startEnd(climb::moveUp, climb::stop, climb).withName("ClimbUp");
  // }
  public static Command hookUp(ClimbSubsystem climb) {
    return Commands.startEnd(() -> climb.moveUp(), climb::stop, climb).withName("HookUp");
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
            hookUp(climb).withTimeout(1.8),

            // settle time
            Commands.waitSeconds(0.4),

            // adjust / descend
            climbDown(climb).withTimeout(1.0),

            // ensure stop
            stop(climb))
        .withName("FullClimbSequence");
  }
}
