package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
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
  // CLIMB DOCKING (SIM SAFE + REAL TUNABLE VERSION)
  // ============================================================

  @SuppressWarnings("resource")
  public static Command dockToClimb(Drive drive, Vision vision) {

    PIDController forward = new PIDController(Constants.Climb.PID.kP_FORWARD, 0, 0);
    PIDController strafe = new PIDController(Constants.Climb.PID.kP_STRAFE, 0, 0);
    PIDController turn = new PIDController(Constants.Climb.PID.kP_TURN, 0, 0);
    turn.enableContinuousInput(-Math.PI, Math.PI);

    // =========================================================
    // 🧠 MEMORY OF TARGET (FIELD-RELATIVE)
    // =========================================================
    final Pose2d[] cachedTarget = {null};
    final double[] smoothX = {0}, smoothY = {0}, smoothRot = {0};

    return Commands.runEnd(
        () -> {
          Pose2d robotPose = drive.getPose(); // odometry anchor
          Optional<Transform2d> visionOpt = vision.getDockingTarget();

          boolean hasVision = visionOpt.isPresent();

          // =========================================================
          // 🧭 BUILD / UPDATE TARGET MEMORY
          // =========================================================
          if (hasVision) {
            Transform2d robotToTag = visionOpt.get();

            Pose2d measuredFieldTarget =
                robotPose.transformBy(robotToTag); // convert to field frame

            if (cachedTarget[0] == null) {
              cachedTarget[0] = measuredFieldTarget;
            } else {
              // slow correction of memory (prevents jitter injection)
              double blend = 0.15;

              cachedTarget[0] =
                  new Pose2d(
                      cachedTarget[0].getX() * (1 - blend) + measuredFieldTarget.getX() * blend,
                      cachedTarget[0].getY() * (1 - blend) + measuredFieldTarget.getY() * blend,
                      new Rotation2d(
                          cachedTarget[0].getRotation().getRadians() * (1 - blend)
                              + measuredFieldTarget.getRotation().getRadians() * blend));
            }
          }

          if (cachedTarget[0] == null) {
            drive.runVelocity(new ChassisSpeeds(0, 0, 0));
            Logger.recordOutput("Dock/HasTarget", false);
            return;
          }

          Pose2d target = cachedTarget[0];

          // =========================================================
          // 🧭 ERROR IN FIELD SPACE (stable)
          // =========================================================
          double dx = target.getX() - robotPose.getX();
          double dy = target.getY() - robotPose.getY();

          double thetaErr =
              MathUtil.angleModulus(
                  target.getRotation().getRadians() - robotPose.getRotation().getRadians());

          double dist = Math.hypot(dx, dy);

          // =========================================================
          // 🧠 PHASE LOGIC
          // =========================================================
          boolean lockPhase = dist < 0.10;
          boolean creepPhase = dist < 0.45;

          double scale;
          double maxXY;
          double maxOmega;

          if (lockPhase) {
            scale = 0.12;
            maxXY = 0.06;
            maxOmega = 0.10;
          } else if (creepPhase) {
            scale = 0.35;
            maxXY = 0.22;
            maxOmega = 0.40;
          } else {
            scale = 1.0;
            maxXY = 0.8;
            maxOmega = 1.2;
          }

          // =========================================================
          // 🧽 SOFT FILTERING (extra stability in creep/lock)
          // =========================================================
          double alpha = lockPhase ? 0.20 : 0.35;

          smoothX[0] = alpha * dx + (1 - alpha) * smoothX[0];
          smoothY[0] = alpha * dy + (1 - alpha) * smoothY[0];
          smoothRot[0] = alpha * thetaErr + (1 - alpha) * smoothRot[0];

          // =========================================================
          // CONTROL
          // =========================================================
          double vx = forward.calculate(0, smoothX[0]) * scale;
          double vy = strafe.calculate(0, smoothY[0]) * scale;
          double omega = turn.calculate(0, smoothRot[0]);

          vx = MathUtil.clamp(vx, -maxXY, maxXY) * drive.getMaxLinearSpeedMetersPerSec();

          vy = MathUtil.clamp(vy, -maxXY, maxXY) * drive.getMaxLinearSpeedMetersPerSec();

          omega = MathUtil.clamp(omega, -maxOmega, maxOmega) * drive.getMaxAngularSpeedRadPerSec();

          // =========================================================
          // 🪶 FINAL HOLD STABILITY
          // =========================================================
          if (lockPhase) {
            if (Math.abs(smoothX[0]) < 0.02) vx = 0;
            if (Math.abs(smoothY[0]) < 0.02) vy = 0;
            if (Math.abs(smoothRot[0]) < Math.toRadians(1.5)) omega = 0;
          }

          drive.runVelocity(new ChassisSpeeds(vx, vy, omega));

          // =========================================================
          // LOGGING
          // =========================================================
          Logger.recordOutput("Dock/Dist", dist);
          Logger.recordOutput("Dock/LockPhase", lockPhase);
          Logger.recordOutput("Dock/CreepPhase", creepPhase);
          Logger.recordOutput("Dock/TargetX", target.getX());
          Logger.recordOutput("Dock/TargetY", target.getY());
        },
        () -> {
          drive.stop();
          cachedTarget[0] = null;
        },
        drive);
  }

  // ============================================================
  // Temporary Command to Tune Docking Vision Target (REAL ROBOT ONLY
  // - SIM USES RAW MEASUREMENT AS TARGET)
  // ============================================================

  public static Command logDockingPoseOnce(Vision vision) {
    return Commands.runOnce(
        () -> {
          vision
              .getDockingTarget()
              .ifPresent(
                  transform -> {
                    Logger.recordOutput("Dock/RawForward", transform.getX());
                    Logger.recordOutput("Dock/RawStrafe", transform.getY());
                    Logger.recordOutput("Dock/RawYawDeg", transform.getRotation().getDegrees());
                  });
        });
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
