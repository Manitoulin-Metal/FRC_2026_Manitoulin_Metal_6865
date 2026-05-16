package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

public final class DriveCommands {

  private DriveCommands() {}

  // ============================================================
  // TAG SELECTION
  // ============================================================

  static int getClimbTagId() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red
        ? 16
        : 32;
  }

  // ============================================================
  // TELEOP DRIVE (unchanged)
  // ============================================================

  public static Command joystickDrive(
      Drive drive,
      java.util.function.DoubleSupplier xSupplier,
      java.util.function.DoubleSupplier ySupplier,
      java.util.function.DoubleSupplier omegaSupplier,
      java.util.function.Supplier<Boolean> robotCentricSupplier) {

    return Commands.run(
        () -> {
          double x = xSupplier.getAsDouble();
          double y = ySupplier.getAsDouble();
          double omega = omegaSupplier.getAsDouble();

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  x * drive.getMaxLinearSpeedMetersPerSec(),
                  y * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  // ============================================================
  // 🚨 FIXED CLIMB DOCKING (MAIN FIX IS HERE)
  // ============================================================

  @SuppressWarnings("resource")
  public static Command dockToClimb(Drive drive, Vision vision) {

    PIDController forwardController = new PIDController(Constants.Climb.PID.kP_FORWARD, 0.0, 0.0);
    PIDController strafeController = new PIDController(Constants.Climb.PID.kP_STRAFE, 0.0, 0.0);
    PIDController turnController = new PIDController(Constants.Climb.PID.kP_TURN, 0.0, 0.0);

    turnController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Optional<Pose3d> poseOpt = vision.getRearTagRelativePose();

              if (poseOpt.isEmpty()) {
                drive.stop();
                return;
              }

              Pose3d current = poseOpt.get();
              /*
               * ROBOT-RELATIVE TAG SPACE
               * X = left/right
               * Z = forward/back
               * rotation Z = yaw
               */

              // ===============================
              // DESIRED SETPOINT (your tuned offsets)
              // ===============================
              double desiredX = Constants.Climb.Vision.blueTX.get();
              double desiredZ = Constants.Climb.Vision.blueTY.get();

              // ===============================
              // ERROR (what PID should drive to zero)
              // ===============================
              double strafeError = current.getX() - desiredX;
              double forwardError = current.getZ() - desiredZ;
              double yawError = current.getRotation().getZ();
              double forward = -forwardController.calculate(forwardError, 0.0);
              double strafe = -strafeController.calculate(strafeError, 0.0);
              double turn = -turnController.calculate(yawError, 0.0);

              forward = MathUtil.clamp(forward, -1.0, 1.0);
              strafe = MathUtil.clamp(strafe, -1.0, 1.0);
              turn = MathUtil.clamp(turn, -1.5, 1.5);

              forward *= drive.getMaxLinearSpeedMetersPerSec();
              strafe *= drive.getMaxLinearSpeedMetersPerSec();
              turn *= drive.getMaxAngularSpeedRadPerSec();

              drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
            },
            drive)
        .until(
            () -> {
              Optional<Pose3d> poseOpt = vision.getRearTagRelativePose();

              if (poseOpt.isEmpty()) return false;

              Pose3d targetSpace = poseOpt.get();

              double strafeError = Math.abs(targetSpace.getX());

              // ✅ FIXED: must match execute logic exactly
              double forwardError = Math.abs(targetSpace.getZ());

              double yawError = Math.abs(targetSpace.getRotation().getZ());

              return strafeError < 0.03 && forwardError < 0.04 && yawError < Math.toRadians(3);
            })
        .andThen(drive::stop)
        .withName("DockToClimb");
  }

  // ============================================================
  // SIMPLE DRIVE TO POSE (unchanged)
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

  // ============================================================
  // CLIMB TAG DRIVE (unchanged logic, safe)
  // ============================================================

  public static Command driveToClimb(
      Drive drive,
      AprilTagFieldLayout fieldLayout,
      Transform2d offset,
      double kPLinear,
      double kPRotation) {

    return Commands.run(
            () -> {
              int tagId = getClimbTagId();

              Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
              if (tagOpt.isEmpty()) return;

              Pose2d targetPose = tagOpt.get().toPose2d().transformBy(offset);
              Pose2d current = drive.getPose();

              double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
              double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

              double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

              double rotSpeed = rotError * kPRotation;

              drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            },
            drive)
        .andThen(drive::stop);
  }
}
