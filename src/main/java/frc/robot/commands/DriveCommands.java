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

    return Commands.runEnd(
            () -> {

              // =========================================================
              // 1. VISION INPUT (FAST EXIT IF LOST)
              // =========================================================
              Optional<Transform2d> opt = vision.getDockingTarget();

              if (opt.isEmpty()) {
                drive.runVelocity(new ChassisSpeeds(0, 0, 0));
                Logger.recordOutput("Dock/HasTarget", false);
                return;
              }

              Transform2d robotToTag = opt.get();
              Logger.recordOutput("Dock/HasTarget", true);

              // =========================================================
              // 2. DESIRED TARGET (ONLY TUNABLE SURFACE)
              // =========================================================
              double desiredX = Constants.Climb.Vision.targetForward.get();
              double desiredY = Constants.Climb.Vision.targetStrafe.get();
              double desiredTheta =
                  Rotation2d.fromDegrees(Constants.Climb.Vision.targetYawDeg.get()).getRadians();

              Logger.recordOutput("Dock/MeasuredX", robotToTag.getX());
              Logger.recordOutput("Dock/MeasuredY", robotToTag.getY());
              Logger.recordOutput("Dock/MeasuredYawDeg", robotToTag.getRotation().getDegrees());

              Logger.recordOutput("Dock/TargetX", desiredX);
              Logger.recordOutput("Dock/TargetY", desiredY);
              Logger.recordOutput("Dock/TargetYawDeg", Math.toDegrees(desiredTheta));

              // =========================================================
              // 3. ERROR SPACE
              // =========================================================
              double xErr = desiredX - robotToTag.getX();
              double yErr = desiredY - robotToTag.getY();
              double thetaErr =
                  MathUtil.angleModulus(
                      desiredTheta
                          - robotToTag
                              .getRotation()
                              .getRadians()); // angleModulus to wrap to [-pi, pi]

              double dist = Math.hypot(xErr, yErr);

              // =========================================================
              // 4. GAIN STAGING (SIMPLIFIED)
              // =========================================================
              double scale;
              double maxXY;

              if (dist < 0.15) {
                scale = 0.20;
                maxXY = 0.10;
              } else if (dist < 0.60) {
                scale = 0.50;
                maxXY = 0.30;
              } else {
                scale = 1.00;
                maxXY = 0.80;
              }
              double maxOmega = (dist < 0.6) ? 0.25 : 1.2;

              // =========================================================
              // 5. CONTROL OUTPUT
              // =========================================================
              double vx = forward.calculate(robotToTag.getX(), desiredX) * scale;
              double vy = strafe.calculate(robotToTag.getY(), desiredY) * scale;
              double omega = turn.calculate(robotToTag.getRotation().getRadians(), desiredTheta);

              vx = MathUtil.clamp(vx, -maxXY, maxXY) * drive.getMaxLinearSpeedMetersPerSec();
              vy = MathUtil.clamp(vy, -maxXY, maxXY) * drive.getMaxLinearSpeedMetersPerSec();
              omega =
                  MathUtil.clamp(omega, -maxOmega, maxOmega) * drive.getMaxAngularSpeedRadPerSec();

              // ----------------------------------------------------
              // Deadband near target to prevent hunting/jitter
              // ----------------------------------------------------
              if (Math.abs(xErr) < 0.02) vx = 0.0;

              if (Math.abs(yErr) < 0.02) vy = 0.0;

              if (dist < 0.12) {
                vx *= 0.3;
                vy *= 0.3;
              }

              if (Math.abs(thetaErr) < Math.toRadians(2.0)) {
                omega = 0.0;
              }

              drive.runVelocity(new ChassisSpeeds(vx, vy, omega));

              // =========================================================
              // 6. MINIMAL LOGGING (NO THROTTLING NEEDED)
              // =========================================================
              Logger.recordOutput("Dock/xErr", xErr);
              Logger.recordOutput("Dock/yErr", yErr);
              Logger.recordOutput("Dock/thetaErr", thetaErr);
              Logger.recordOutput("Dock/dist", dist);
              Logger.recordOutput("Dock/vx", vx);
              Logger.recordOutput("Dock/vy", vy);
              Logger.recordOutput("Dock/omega", omega);

              Logger.recordOutput("Dock/atGoal", dist < 0.10 && Math.abs(thetaErr) < 0.08);
            },
            drive::stop,
            drive)
        .withName("DockToClimb");
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
