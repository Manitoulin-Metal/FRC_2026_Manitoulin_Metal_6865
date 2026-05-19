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
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

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
  // CLIMB DOCKING (ROBOT-RELATIVE VERSION)
  // ============================================================

@SuppressWarnings("resource")
public static Command dockToClimb(Drive drive, Vision vision) {

    PIDController forwardController =
            new PIDController(Constants.Climb.PID.kP_FORWARD, 0.0, 0.0);

    PIDController strafeController =
            new PIDController(Constants.Climb.PID.kP_STRAFE, 0.0, 0.0);

    PIDController turnController =
            new PIDController(Constants.Climb.PID.kP_TURN, 0.0, 0.0);

    turnController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.runEnd(
        new Runnable() {

            int logCounter = 0;

            @Override
            public void run() {

                Optional<Transform2d> robotToTagOpt = vision.getDockingTarget();

                boolean hasTarget = robotToTagOpt.isPresent();

                if (!hasTarget) {
                    drive.stop();

                    // lightweight log (always safe)
                    Logger.recordOutput("DockToClimb/hasTarget", false);
                    return;
                }

                Transform2d robotToTag = robotToTagOpt.get();
                Transform2d error = robotToTag.inverse();

                double xError = error.getX();
                double yError = error.getY();
                double thetaError = error.getRotation().getRadians();

                double distance = Math.hypot(xError, yError);

                // ============================================================
                // STAGE CONTROL
                // ============================================================
                boolean slowMode = distance < 0.6;

                double gainScale;
                double maxXY;
                double omegaScale;

                if (!slowMode) {
                    gainScale = 1.0;
                    maxXY = 0.8;
                    omegaScale = 1.0;
                } else {
                    gainScale = 0.5;
                    maxXY = 0.3;
                    omegaScale = 0.7;
                }

                // ============================================================
                // PID CONTROL
                // ============================================================
                double vx = forwardController.calculate(xError, 0.0);
                double vy = strafeController.calculate(yError, 0.0);
                double omega = turnController.calculate(thetaError, 0.0);

                vx *= gainScale;
                vy *= gainScale;
                omega *= omegaScale;

                vx = MathUtil.clamp(vx, -maxXY, maxXY);
                vy = MathUtil.clamp(vy, -maxXY, maxXY);
                omega = MathUtil.clamp(omega, -1.2, 1.2);

                vx *= drive.getMaxLinearSpeedMetersPerSec();
                vy *= drive.getMaxLinearSpeedMetersPerSec();
                omega *= drive.getMaxAngularSpeedRadPerSec();

                drive.runVelocity(new ChassisSpeeds(vx, vy, omega));

                // ============================================================
                // THROTTLED LOGGING (EVERY 5 CYCLES)
                // ============================================================
                logCounter++;

                if (logCounter % 5 == 0) {

                    Logger.recordOutput("DockToClimb/hasTarget", true);

                    Logger.recordOutput("DockToClimb/xError", xError);
                    Logger.recordOutput("DockToClimb/yError", yError);
                    Logger.recordOutput("DockToClimb/thetaError", thetaError);
                    Logger.recordOutput("DockToClimb/distance", distance);

                    Logger.recordOutput("DockToClimb/vx", vx);
                    Logger.recordOutput("DockToClimb/vy", vy);
                    Logger.recordOutput("DockToClimb/omega", omega);

                    boolean atGoal =
                            distance < 0.10 && Math.abs(thetaError) < 0.08;

                    Logger.recordOutput("DockToClimb/atGoal", atGoal);
                }
            }
        },
        drive::stop,
        drive
    ).withName("DockToClimb");
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
}
