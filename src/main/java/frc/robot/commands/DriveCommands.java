package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {

  private DriveCommands() {
  }

  // -----------------------------
  // Tag selection
  // -----------------------------
  private static int getClimbTagId() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? 16
            : 32;
  }

  // -----------------------------
  // TELEOP JOYSTICK DRIVE
  // -----------------------------

  public static Command joystickDrive(
      Drive drive,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier,
      Supplier<Boolean> robotCentricSupplier) {

    return Commands.run(
        () -> {
          ChassisSpeeds speeds;

          double xSpeed = xSupplier.get() * drive.getMaxLinearSpeedMetersPerSec();

          double ySpeed = ySupplier.get() * drive.getMaxLinearSpeedMetersPerSec();

          double omegaSpeed = omegaSupplier.get() * drive.getMaxAngularSpeedRadPerSec();

          if (robotCentricSupplier.get()) {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
          } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                omegaSpeed,
                drive.getRotation());
          }

          drive.runVelocity(speeds);

          Logger.recordOutput(
              "Drive/RobotCentric",
              robotCentricSupplier.get());
          SmartDashboard.putBoolean("Drive/RobotCentric", robotCentricSupplier.get());
        },
        drive);
  }

  public static Command slowJoystickDrive(
      Drive drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
    return joystickDrive(
        drive,
        () -> forward.getAsDouble() * 0.5,
        () -> strafe.getAsDouble() * 0.5,
        () -> rotation.getAsDouble() * 0.5,
        () -> false);
  }

  // -----------------------------
  // Generic Drive to Pose
  // -----------------------------
  public static Command driveToPose(
      Drive drive, Pose2d targetPose, double kPLinear, double kPRotation) {

    return Commands.run(
        () -> {
          Pose2d current = drive.getPose();

          double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
          double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

          double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

          double rotSpeed = rotError * kPRotation;

          // Clamp
          double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
          double maxAngular = drive.getMaxAngularSpeedRadPerSec();

          xSpeed = MathUtil.clamp(xSpeed, -maxLinear, maxLinear);
          ySpeed = MathUtil.clamp(ySpeed, -maxLinear, maxLinear);
          rotSpeed = MathUtil.clamp(rotSpeed, -maxAngular, maxAngular);

          drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
        },
        drive)
        .until(
            () -> {
              Pose2d current = drive.getPose();

              double dist = current.getTranslation().getDistance(targetPose.getTranslation());

              double angle = Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());

              return dist < 0.05 && angle < 0.05;
            })
        .andThen(drive::stop);
  }

  // -----------------------------
  // CLIMB COMMAND (uses limelight0)
  // -----------------------------
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
          if (tagOpt.isEmpty())
            return;

          Pose2d tagPose = tagOpt.get().toPose2d();

          Pose2d targetPose = tagPose.transformBy(offset);

          Pose2d current = drive.getPose();

          double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
          double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

          double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

          double rotSpeed = rotError * kPRotation;

          double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
          double maxAngular = drive.getMaxAngularSpeedRadPerSec();

          xSpeed = MathUtil.clamp(xSpeed, -maxLinear, maxLinear);
          ySpeed = MathUtil.clamp(ySpeed, -maxLinear, maxLinear);
          rotSpeed = MathUtil.clamp(rotSpeed, -maxAngular, maxAngular);

          drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
        },
        drive)
        .until(
            () -> {
              int tagId = getClimbTagId();

              Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
              if (tagOpt.isEmpty())
                return false;

              Pose2d target = tagOpt.get().toPose2d().transformBy(offset);

              Pose2d current = drive.getPose();

              double dist = current.getTranslation().getDistance(target.getTranslation());

              double angle = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());

              return dist < 0.05 && angle < 0.05;
            })
        .andThen(drive::stop);
  }

  // -----------------------------
  // SHOOT COMMAND (no vision)
  // -----------------------------
  public static Command driveToShoot(
      Drive drive,
      AprilTagFieldLayout fieldLayout,
      int tagId,
      double distance,
      double kPLinear,
      double kPRotation) {

    return Commands.run(
        () -> {
          Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
          if (tagOpt.isEmpty())
            return;

          Pose2d tagPose = tagOpt.get().toPose2d();

          Pose2d current = drive.getPose();

          Translation2d tagToRobot = current.getTranslation().minus(tagPose.getTranslation());

          double currentDist = tagToRobot.getNorm();

          Translation2d direction = tagToRobot.div(currentDist);

          Translation2d targetTranslation = tagPose.getTranslation().plus(direction.times(distance));

          Rotation2d targetRotation = tagPose.getTranslation().minus(targetTranslation).getAngle();

          Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

          double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
          double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

          double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

          double rotSpeed = rotError * kPRotation;

          double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
          double maxAngular = drive.getMaxAngularSpeedRadPerSec();

          xSpeed = MathUtil.clamp(xSpeed, -maxLinear, maxLinear);
          ySpeed = MathUtil.clamp(ySpeed, -maxLinear, maxLinear);
          rotSpeed = MathUtil.clamp(rotSpeed, -maxAngular, maxAngular);

          drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
        },
        drive)
        .until(
            () -> {
              Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
              if (tagOpt.isEmpty())
                return false;

              Pose2d target = tagOpt
                  .get()
                  .toPose2d()
                  .transformBy(
                      new Transform2d(new Translation2d(distance, 0), new Rotation2d()));

              Pose2d current = drive.getPose();

              double dist = current.getTranslation().getDistance(target.getTranslation());

              return dist < 0.05;
            })
        .andThen(drive::stop);
  }

  // ============================================================
  // Vision-based shooting alignment using Limelight + distance)
  // ============================================================
  public static Command driveToShootVision(
      Drive drive,
      VisionSubsystem vision,
      frc.robot.subsystems.shooter.ShooterSubsystem shooter,
      AprilTagFieldLayout fieldLayout,
      Supplier<Boolean> visionEnabled,
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRot,
      double kPLinear,
      double kPRotation) {

    return Commands.run(
        () -> {

          // =========================
          // VISION OVERRIDE CHECK
          // =========================
          if (!visionEnabled.get()) {
            drive.runVelocity(new ChassisSpeeds(driverX.get(), driverY.get(), driverRot.get()));
            return;
          }

          // =========================
          // AUTO TAG SELECTION
          // =========================
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          int targetTag = (alliance == Alliance.Blue) ? 25 : 9;

          if (!vision.hasTag(targetTag)) {
            // fallback to driver control
            drive.runVelocity(new ChassisSpeeds(driverX.get(), driverY.get(), driverRot.get()));
            return;
          }

          // =========================
          // REAL TAG POSE MATH
          // =========================
          Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(targetTag);
          if (tagPose3d.isEmpty())
            return;

          Pose2d tagPose = tagPose3d.get().toPose2d();
          Pose2d robotPose = drive.getPose();

          // 2m shooting offset (arc target)
          Transform2d offset = new Transform2d(new Translation2d(-2.0, 0.0), Rotation2d.fromDegrees(180));

          Pose2d targetPose = tagPose.transformBy(offset);

          Transform2d error = targetPose.minus(robotPose);

          double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());

          // RPM from your table
          double targetRPM = Constants.getRPMForDistance(distance);

          // Convert to RPS for shooter
          double targetRps = targetRPM / 60.0;

          // Send to shooter
          shooter.runShooter(targetRps);

          // Dashboard display
          SmartDashboard.putString(
              "Shooter Status",
              String.format("Shooting to %.2f m at %.0f RPM", distance, targetRPM));

          // =========================
          // ARC + BLENDING
          // =========================
          double visionWeight = 0.7;
          double driverWeight = 1.0 - visionWeight;

          double forwardVision = error.getX() * kPLinear;
          double strafeVision = error.getY() * kPLinear;
          double rotVision = error.getRotation().getRadians() * kPRotation;

          double vx = driverX.get() * driverWeight + forwardVision * visionWeight;
          double vy = driverY.get() * driverWeight + strafeVision * visionWeight;
          double vr = driverRot.get() * driverWeight + rotVision * visionWeight;

          // Clamp
          double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
          double maxAngular = drive.getMaxAngularSpeedRadPerSec();

          vx = MathUtil.clamp(vx, -maxLinear, maxLinear);
          vy = MathUtil.clamp(vy, -maxLinear, maxLinear);
          vr = MathUtil.clamp(vr, -maxAngular, maxAngular);

          // Drive
          drive.runVelocity(new ChassisSpeeds(vx, vy, vr));
        },
        drive)
        .until(() -> Math.abs(vision.getTX()) < 1.0 && Math.abs(vision.getTY()) < 1.0)
        .andThen(drive::stop);
  }
}
