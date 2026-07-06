package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Boolean> robotCentricSupplier) {

    return Commands.run(
        () -> {
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.05);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.05);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), 0.05);

          x *= drive.getMaxLinearSpeedMetersPerSec();
          y *= drive.getMaxLinearSpeedMetersPerSec();
          omega *= drive.getMaxAngularSpeedRadPerSec();

          Rotation2d heading =
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                  ? drive.getRotation().plus(Rotation2d.fromDegrees(180))
                  : drive.getRotation();

          ChassisSpeeds speeds =
              robotCentricSupplier.get()
                  ? new ChassisSpeeds(x, y, omega)
                  : ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, heading);

          drive.runVelocity(speeds);
        },
        drive);
  }

  // ============================================================
  // CLIMB DOCKING (BACK-IN, CLEAN SINGLE-SOURCE VERSION)
  // ============================================================

  public static Command dockToClimb(Drive drive, Vision vision) {

    return Commands.runEnd(
        () -> {
          Pose2d current = drive.getPose();

          Optional<Pose2d> targetOpt =
              vision.getDockTargetPose() == null
                  ? Optional.empty()
                  : Optional.of(vision.getDockTargetPose());

          if (targetOpt.isEmpty()) return;

          Pose2d target = targetOpt.get();

          // =====================================================
          // FIELD ERROR (correct frame usage)
          // =====================================================
          Translation2d error = target.getTranslation().minus(current.getTranslation());
          double distance = error.getNorm();

          double angleError = target.getRotation().minus(current.getRotation()).getRadians();

          // =====================================================
          // LINEAR CONTROL (clean P controller)
          // =====================================================
          double kP = 1.2;

          double vx = MathUtil.clamp(error.getX() * kP, -1.0, 1.0);
          double vy = MathUtil.clamp(error.getY() * kP, -1.0, 1.0);

          // optional slowdown near target
          double slowZone = Constants.Climb.Vision.DOCK_FINAL_DISTANCE;

          if (distance < slowZone) {
            vx *= 0.4;
            vy *= 0.4;
          }

          // =====================================================
          // ANGULAR CONTROL
          // =====================================================
          double omega = MathUtil.clamp(angleError * 2.5, -2.0, 2.0);

          if (Math.abs(angleError) < Math.toRadians(3)) {
            omega = 0;
          }

          // =====================================================
          // CONVERT TO ROBOT SPEEDS
          // =====================================================
          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  vx * drive.getMaxLinearSpeedMetersPerSec(),
                  vy * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  current.getRotation());

          drive.runVelocity(speeds);

          Logger.recordOutput("Dock/Distance", distance);
          Logger.recordOutput("Dock/AngleErrorDeg", Math.toDegrees(angleError));
          Logger.recordOutput("Dock/Target", target);
          Logger.recordOutput("Dock/Robot", current);
        },
        () -> drive.stop(),
        drive);
  }

  public static Command logDockCalibration(Vision vision) {

    return Commands.runOnce(
        () -> {
          var dock = vision.getDockTransform();

          if (dock.isEmpty()) {
            Logger.recordOutput("Dock/Calibration/Status", "NO_DOCK");
            return;
          }

          Translation2d t = dock.get().getTranslation();

          Logger.recordOutput("Dock/Calibration/Forward", t.getX());
          Logger.recordOutput("Dock/Calibration/Strafe", t.getY());
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
