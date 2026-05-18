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

  private DriveCommands() {
  }

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
          ChassisSpeeds speeds = new ChassisSpeeds(
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
  // CLIMB DOCKING (CLEAN ROBOT-RELATIVE VERSION)
  // ============================================================

  public static Command dockToClimb(Drive drive, Vision vision) {

  PIDController forwardController =
      new PIDController(Constants.Climb.PID.kP_FORWARD, 0.0, 0.0);

  PIDController strafeController =
      new PIDController(Constants.Climb.PID.kP_STRAFE, 0.0, 0.0);

  PIDController turnController =
      new PIDController(Constants.Climb.PID.kP_TURN, 0.0, 0.0);

  turnController.enableContinuousInput(-Math.PI, Math.PI);

  return Commands.startRun(
      () -> {
        System.out.println("DockToClimb RUNNING");
        forwardController.reset();
        strafeController.reset();
        turnController.reset();
      },
      () -> {

        Optional<Transform2d> robotToTagOpt = vision.getDockingTarget();

        if (robotToTagOpt.isEmpty()) {
          System.out.println("DockToClimb: NO TAG");
          drive.stop();
          return;
        }

        Transform2d robotToTag = robotToTagOpt.get();

        // IMPORTANT: we want error = tag relative to robot inverse
        Transform2d error = robotToTag.inverse();

        double forwardError = error.getX();
        double strafeError = error.getY();
        double rotError = error.getRotation().getRadians();

        double vx = forwardController.calculate(forwardError, 0.0);
        double vy = strafeController.calculate(strafeError, 0.0);
        double omega = turnController.calculate(rotError, 0.0);

        vx = MathUtil.clamp(vx, -0.8, 0.8);
        vy = MathUtil.clamp(vy, -0.8, 0.8);
        omega = MathUtil.clamp(omega, -1.2, 1.2);

        vx *= drive.getMaxLinearSpeedMetersPerSec();
        vy *= drive.getMaxLinearSpeedMetersPerSec();
        omega *= drive.getMaxAngularSpeedRadPerSec();

        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
      },
      drive
  ).finallyDo(drive::stop)
   .withName("DockToClimb");
}

  // ============================================================
  // SIMPLE DRIVE TO POSE (UNCHANGED)
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
  // FIELD-BASED CLIMB DRIVE (OPTIONAL LEGACY SAFE VERSION)
  // ============================================================

  public static Command driveToClimbPose(
      Drive drive,
      AprilTagFieldLayout fieldLayout,
      Transform2d offset,
      double kPLinear,
      double kPRotation) {

    return Commands.run(
        () -> {
          int tagId = getClimbTagId();

          var tagOpt = fieldLayout.getTagPose(tagId);
          if (tagOpt.isEmpty())
            return;

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
