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

  public static Command dockToClimb(Drive drive, Vision vision) {

    return Commands.runEnd(
            () -> {
              Pose2d target = vision.getDockTargetPose();
              Pose2d current = drive.getPose();

              Translation2d delta = target.getTranslation().minus(current.getTranslation());

              double distance = delta.getNorm();

              double angleError = target.getRotation().minus(current.getRotation()).getRadians();

              double vx = delta.getX();
              double vy = delta.getY();

              Rotation2d direction = new Rotation2d(vx, vy);

              double speedScale;

              if (distance > Constants.Climb.Vision.DOCK_SLOW_DISTANCE) {
                speedScale = Constants.Climb.Vision.DOCK_MAX_SPEED;
              } else if (distance > Constants.Climb.Vision.DOCK_FINAL_DISTANCE) {
                speedScale = 0.35;
              } else {
                speedScale = Constants.Climb.Vision.DOCK_MIN_SPEED;
              }

              double vxCmd = direction.getCos() * speedScale;
              double vyCmd = direction.getSin() * speedScale;

              double omegaScale;

              if (Math.abs(angleError) > Math.toRadians(8)) {
                omegaScale = Constants.Climb.Vision.DOCK_MAX_OMEGA;
              } else if (Math.abs(angleError) > Math.toRadians(3)) {
                omegaScale = 0.4;
              } else {
                omegaScale = Constants.Climb.Vision.DOCK_MIN_OMEGA;
              }

              double omegaCmd = MathUtil.clamp(angleError * 2.5, -omegaScale, omegaScale);

              if (distance < Constants.Climb.Vision.DOCK_POSITION_DEADBAND) {
                vxCmd = 0;
                vyCmd = 0;
              }

              if (Math.abs(angleError)
                  < Math.toRadians(Constants.Climb.Vision.DOCK_ANGLE_DEADBAND)) {
                omegaCmd = 0;
              }

              drive.runVelocity(
                  new ChassisSpeeds(
                      vxCmd * drive.getMaxLinearSpeedMetersPerSec(),
                      vyCmd * drive.getMaxLinearSpeedMetersPerSec(),
                      omegaCmd * drive.getMaxAngularSpeedRadPerSec()));

              Logger.recordOutput("Dock/Distance", distance);
              Logger.recordOutput("Dock/AngleErrorDeg", Math.toDegrees(angleError));
              Logger.recordOutput("Dock/Target", target);
              Logger.recordOutput("Dock/Robot", current);
            },
            () -> drive.stop(),
            drive)
        .withName("DockToClimb");
  }

  public static Command logDockingPoseOnce(Vision vision) {
    return Commands.runOnce(
        () -> Logger.recordOutput("Dock/TargetPose", vision.getDockTargetPose()));
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
