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

  // ============================================================
  // CLIMB DOCKING (REAR LIMELIGHT ROBOT-RELATIVE VERSION)
  // ============================================================

  public static Command dockToClimb(Drive drive, Vision vision) {

    return Commands.runEnd(
        () -> {
          if (!vision.hasFreshDock()) {
            drive.stop();
            return;
          }

          // =====================================================
          // LIMELIGHT ROBOT-RELATIVE ERROR
          //
          // Forward  = distance along robot forward axis
          // Strafe   = distance along robot left/right axis
          // Yaw      = robot rotation relative to tag
          //
          // These are already robot-frame measurements.
          // DO NOT convert to field-relative speeds.
          // =====================================================

          double forwardError = vision.getDockTransform().get().getX();
          double strafeError = vision.getDockTransform().get().getY();
          double yawError =
              MathUtil.angleModulus(vision.getDockTransform().get().getRotation().getRadians());

          // =====================================================
          // POSITION CONTROL
          // =====================================================

          double kPForward = 1.2;
          double kPStrafe = 1.2;
          double kPYaw = 2.5;

          double vx = MathUtil.clamp(forwardError * kPForward, -1.0, 1.0);

          double vy = MathUtil.clamp(strafeError * kPStrafe, -1.0, 1.0);

          double omega = MathUtil.clamp(yawError * kPYaw, -2.0, 2.0);

          // =====================================================
          // STOP ROTATION WHEN ALIGNED
          // =====================================================

          if (Math.abs(Math.toDegrees(yawError)) < 3.0) {
            omega = 0;
          }

          // =====================================================
          // FINAL APPROACH + STOP TOLERANCE
          // =====================================================

          double distance = Math.hypot(forwardError, strafeError);

          double distanceTolerance = 0.015; // 1.5 cm
          double yawToleranceDeg = 1.0; // 1 degree

          boolean atPosition =
              distance < distanceTolerance && Math.abs(Math.toDegrees(yawError)) < yawToleranceDeg;

          Logger.recordOutput("Dock/DistanceError", distance);
          Logger.recordOutput("Dock/AtPosition", atPosition);

          if (atPosition) {
            drive.stop();
            Logger.recordOutput("Dock/Status", "LOCKED");
            return;
          }

          Logger.recordOutput("Dock/Status", "MOVING");

          if (distance < Constants.Climb.Vision.DOCK_FINAL_DISTANCE) {
            vx *= 0.25;
            vy *= 0.25;
          }

          // =====================================================
          // SEND ROBOT-RELATIVE COMMAND
          // =====================================================

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  vx * drive.getMaxLinearSpeedMetersPerSec(),
                  vy * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          drive.runVelocity(speeds);

          // =====================================================
          // CALIBRATION LOGGING
          // =====================================================

          Logger.recordOutput("Dock/ForwardError", forwardError);
          Logger.recordOutput("Dock/StrafeError", strafeError);
          Logger.recordOutput("Dock/YawErrorDeg", Math.toDegrees(yawError));

          Logger.recordOutput("Dock/VxCmd", vx);
          Logger.recordOutput("Dock/VyCmd", vy);
          Logger.recordOutput("Dock/OmegaCmd", omega);
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
