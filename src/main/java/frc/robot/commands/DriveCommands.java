package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {

  private DriveCommands() {}

  // ============================================================
  // TAG SELECTION
  // ============================================================

  static int getClimbTagId() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 16 : 32;
  }

  // ============================================================
  // TELEOP DRIVE (UNCHANGED LOGIC, CLEANED)
  // ============================================================

  public static Command joystickDrive(
      Drive drive,
      java.util.function.DoubleSupplier xSupplier,
      java.util.function.DoubleSupplier ySupplier,
      java.util.function.DoubleSupplier omegaSupplier,
      java.util.function.Supplier<Boolean> robotCentricSupplier) {

    return Commands.run(
        () -> {
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.05);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.05);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), 0.05);
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  x * drive.getMaxLinearSpeedMetersPerSec(),
                  y * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isRed ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
        },
        drive);
  }

  // ============================================================
  // CLIMB DOCKING (BACK-IN, CLEAN SINGLE-SOURCE VERSION)
  // ============================================================
  @SuppressWarnings("resource")
  public static Command dockToClimb(Drive drive, Vision vision) {

    PIDController xPID = new PIDController(Constants.Climb.PID.kP_FORWARD, 0.0, 0.0);
    PIDController yPID = new PIDController(Constants.Climb.PID.kP_STRAFE, 0.0, 0.0);
    PIDController rotPID = new PIDController(Constants.Climb.PID.kP_TURN, 0.0, 0.0);

    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.runEnd(
        () -> {

          // =========================================================
          // 1. ROBOT POSE
          // =========================================================
          Pose2d robotPose = drive.getPose();

          // =========================================================
          // 2. TAG POSE
          // =========================================================
          Optional<Pose2d> tagOpt = vision.getClimbTagPose();

          if (tagOpt.isEmpty()) {
            drive.stop();
            Logger.recordOutput("Dock/HasTarget", false);
            return;
          }

          Pose2d tagPose = tagOpt.get();

          // =========================================================
          // 3. TUNABLE OFFSET (ONLY SOURCE OF GOAL POSITION)
          // =========================================================
          double forward = Constants.Climb.Vision.targetForward.get();
          double strafe = Constants.Climb.Vision.targetStrafe.get();
          double yawDeg = Constants.Climb.Vision.targetYawDeg.get();

          Logger.recordOutput("Dock/TargetForward", forward);
          Logger.recordOutput("Dock/TargetStrafe", strafe);
          Logger.recordOutput("Dock/TargetYawDeg", yawDeg);

          // =========================================================
          // 4. BUILD GOAL POSE (FIELD FRAME)
          // =========================================================
          Transform2d tagToGoal =
              new Transform2d(new Translation2d(forward, strafe), Rotation2d.fromDegrees(yawDeg));

          Pose2d goalPose = tagPose.transformBy(tagToGoal);

          // =========================================================
          // 5. BACK-IN HEADING (STABLE, NO SELF-FIGHTING)
          //
          // Robot should face AWAY from tag while docking
          // =========================================================
          Translation2d goalToTag = tagPose.getTranslation().minus(goalPose.getTranslation());

          Rotation2d faceTag = new Rotation2d(Math.atan2(goalToTag.getY(), goalToTag.getX()));

          Rotation2d backInHeading = faceTag.rotateBy(Rotation2d.kPi);

          Pose2d finalGoal = new Pose2d(goalPose.getTranslation(), backInHeading);

          // =========================================================
          // 6. ERROR (FIELD SPACE)
          // =========================================================
          double errorX = finalGoal.getX() - robotPose.getX();
          double errorY = finalGoal.getY() - robotPose.getY();

          double errorTheta =
              MathUtil.angleModulus(
                  finalGoal.getRotation().getRadians() - robotPose.getRotation().getRadians());

          double distance = Math.hypot(errorX, errorY);

          Logger.recordOutput("Dock/GoalPose", goalPose);
          Logger.recordOutput("Dock/FinalGoalPose", finalGoal);
          Logger.recordOutput("Dock/ErrorX", errorX);
          Logger.recordOutput("Dock/ErrorY", errorY);
          Logger.recordOutput("Dock/Distance", distance);

          // =========================================================
          // 7. SPEED LIMITS (PREVENT OVERSHOOT / OSCILLATION)
          // =========================================================
          boolean slow = distance < Constants.Climb.Vision.SLOW_MODE_DISTANCE;

          double maxLinear =
              slow
                  ? Constants.Climb.Vision.SLOW_MAX_LINEAR
                  : Constants.Climb.Vision.FAST_MAX_LINEAR;

          double maxOmega =
              slow ? Constants.Climb.Vision.SLOW_MAX_OMEGA : Constants.Climb.Vision.FAST_MAX_OMEGA;

          // =========================================================
          // 8. PID CONTROL
          // =========================================================
          double vx =
              MathUtil.clamp(
                  xPID.calculate(robotPose.getX(), finalGoal.getX()), -maxLinear, maxLinear);

          double vy =
              MathUtil.clamp(
                  yPID.calculate(robotPose.getY(), finalGoal.getY()), -maxLinear, maxLinear);

          double omega =
              MathUtil.clamp(
                  rotPID.calculate(
                      robotPose.getRotation().getRadians(), finalGoal.getRotation().getRadians()),
                  -maxOmega,
                  maxOmega);

          // =========================================================
          // 9. DRIVE
          // =========================================================
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation()));
        },
        drive::stop,
        drive);
  }

  public static Command logDockingPoseOnce(Vision vision) {
    return Commands.runOnce(
        () ->
            vision
                .getDockingTarget()
                .ifPresent(
                    transform -> {
                      Logger.recordOutput("Dock/RawForward", transform.getX());
                      Logger.recordOutput("Dock/RawStrafe", transform.getY());
                      Logger.recordOutput("Dock/RawYawDeg", transform.getRotation().getDegrees());
                    }));
  }

  // ============================================================
  // SIMPLE DRIVE TO POSE COMMAND (FOR TESTING / TUNING ONLY)
  // ============================================================

  public static Command driveToPose(
      Drive drive, Pose2d targetPose, double kPLinear, double kPRotation) {

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();

              double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
              double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

              double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

              double rotSpeed = rotError * kPRotation;

              xSpeed = MathUtil.clamp(xSpeed, -3, 3);
              ySpeed = MathUtil.clamp(ySpeed, -3, 3);
              rotSpeed = MathUtil.clamp(rotSpeed, -3, 3);

              drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            },
            drive)
        .andThen(drive::stop);
  }
}
