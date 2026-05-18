package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public final class ClimbCommands {

  private ClimbCommands() {}

  // ============================================================
  // PID CONTROLLERS
  // ============================================================

  private static final PIDController forwardController =
      new PIDController(Constants.Climb.PID.kP_FORWARD, 0.0, 0.0);

  private static final PIDController strafeController =
      new PIDController(Constants.Climb.PID.kP_STRAFE, 0.0, 0.0);

  private static final PIDController turnController =
      new PIDController(Constants.Climb.PID.kP_TURN, 0.0, 0.0);

  static {
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ============================================================
  // ALLIANCE HELPERS
  // ============================================================

  private static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  private static boolean isRed() {
    return alliance() == Alliance.Red;
  }

  // ONE unified yaw target (no contradictions)
  private static Rotation2d targetYaw() {
    // Flip if your physical mounting requires it
    return new Rotation2d(Math.PI); // sets to 180deg
  }

  // ============================================================
  // FILTER STATE
  // ============================================================

  private static double filteredTX = 0.0;
  private static double filteredTY = 0.0;

  private static void updateFilter(double tx, double ty) {
    filteredTX = 0.8 * filteredTX + 0.2 * tx;
    filteredTY = 0.8 * filteredTY + 0.2 * ty;
  }

  // ============================================================
  // SIMPLE ALIGN (FIXED)
  // ============================================================

  public static Command autoClimbDrive(Drive drive, Vision vision) {

    return Commands.run(
            () -> {
              int targetTag = isRed() ? 16 : 32;

              if (!vision.hasTag(targetTag)) {
                drive.stop();
                return;
              }

              double tx = vision.getTX();
              double ty = vision.getTY();

              updateFilter(tx, ty);

              double kP_X = 0.05;
              double kP_Y = 0.05;
              double kP_ROT = 0.03;

              double vx = filteredTY * kP_Y;
              double vy = filteredTX * kP_X;

              double yawError = drive.getRotation().minus(targetYaw()).getRadians();

              double omega = yawError * kP_ROT;

              drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
            },
            drive)
        .finallyDo(drive::stop)
        .withName("ClimbAutoAlignDrive");
  }

  // ============================================================
  // MAIN DOCKING COMMAND (CLEAN + CONSISTENT)
  // ============================================================

  // ============================================================
  // MAIN DOCKING COMMAND
  // ============================================================
  //
  // Uses Limelight TARGET SPACE pose data.
  //
  // TARGET SPACE AXES:
  // pose[0] = X = left/right offset from tag (strafe)
  // pose[2] = Z = forward/back distance from tag
  // pose[4] = Yaw relative to tag
  //
  // ------------------------------------------------------------
  // CALIBRATION / TESTING NOTES
  // ------------------------------------------------------------
  //
  // 1. Disable robot and physically place robot where you WANT
  //    the final docking location to be.
  //
  // 2. Observe logged values:
  //
  //    ClimbDock/TX
  //    ClimbDock/TZ
  //    ClimbDock/Yaw
  //
  // 3. Copy those values into:
  //
  //    Constants.Climb.Vision.blueTX
  //    Constants.Climb.Vision.blueTY
  //
  // 4. Re-enable robot and test.
  //
  // 5. IMPORTANT SIGN TEST:
  //
  //    Move robot FARTHER from tag:
  //
  //    If TZ increases:
  //       forward PID probably needs NEGATIVE sign.
  //
  //    If TZ decreases:
  //       remove negative sign from vx.
  //
  // 6. STRAFE TEST:
  //
  //    Move robot LEFT:
  //
  //    If TX increases:
  //       strafe sign is correct.
  //
  //    If TX decreases:
  //       invert vy.
  //
  // ------------------------------------------------------------

  public static Command dockToClimb(Drive drive) {

    return Commands.run(
            () -> {

              // ------------------------------------------------
              // Ensure Limelight sees target
              // ------------------------------------------------

              if (!LimelightHelpers.getTV(Constants.Climb.Vision.REAR_LIMELIGHT)) {
                drive.stop();
                return;
              }

              // ------------------------------------------------
              // Read TARGET SPACE pose directly from Limelight
              // ------------------------------------------------

              double[] pose =
                  LimelightHelpers.getBotPose_TargetSpace(Constants.Climb.Vision.REAR_LIMELIGHT);

              // TARGET SPACE VALUES
              double tx = pose[0]; // left/right
              double tz = pose[2]; // forward/back
              double yawDeg = pose[4]; // yaw in degrees

              // ------------------------------------------------
              // Apply smoothing filter
              // ------------------------------------------------

              updateFilter(tx, tz);

              // ------------------------------------------------
              // Desired docking pose
              // ------------------------------------------------

              double desiredTX = Constants.Climb.Vision.targetTX;
              double desiredTZ = Constants.Climb.Vision.targetTY;

              // ------------------------------------------------
              // PID calculations
              // ------------------------------------------------
              //
              // IMPORTANT:
              // Forward is NEGATED because Limelight target-space
              // Z axis is inverted relative to robot forward
              // on many rear-camera setups.
              //
              // If robot drives AWAY from tag:
              // remove the negative sign from vx.
              //
              // ------------------------------------------------

              double vx = -forwardController.calculate(filteredTY, desiredTZ);

              double vy = strafeController.calculate(filteredTX, desiredTX);

              double omega = turnController.calculate(yawDeg, 0.0);

              // ------------------------------------------------
              // Clamp outputs
              // ------------------------------------------------

              vx = MathUtil.clamp(vx, -0.8, 0.8);
              vy = MathUtil.clamp(vy, -0.8, 0.8);
              omega = MathUtil.clamp(omega, -1.0, 1.0);

              // ------------------------------------------------
              // Deadbands
              // ------------------------------------------------

              if (Math.abs(vx) < 0.03) vx = 0.0;
              if (Math.abs(vy) < 0.03) vy = 0.0;
              if (Math.abs(omega) < 0.03) omega = 0.0;

              // ------------------------------------------------
              // Logging for calibration
              // ------------------------------------------------

              Logger.recordOutput("ClimbDock/TX", filteredTX);
              Logger.recordOutput("ClimbDock/TZ", filteredTY);
              Logger.recordOutput("ClimbDock/YawDeg", yawDeg);

              Logger.recordOutput("ClimbDock/DesiredTX", desiredTX);
              Logger.recordOutput("ClimbDock/DesiredTZ", desiredTZ);

              Logger.recordOutput("ClimbDock/TXError", desiredTX - filteredTX);

              Logger.recordOutput("ClimbDock/TZError", desiredTZ - filteredTY);

              Logger.recordOutput("ClimbDock/YawErrorDeg", yawDeg);

              Logger.recordOutput("ClimbDock/VX", vx);
              Logger.recordOutput("ClimbDock/VY", vy);
              Logger.recordOutput("ClimbDock/Omega", omega);

              // ------------------------------------------------
              // Final drive command
              // ------------------------------------------------

              drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
            },
            drive)

        // --------------------------------------------------------
        // Finish condition
        // --------------------------------------------------------

        .until(
            () -> {
              if (!LimelightHelpers.getTV(Constants.Climb.Vision.REAR_LIMELIGHT)) {
                return false;
              }

              double txError = Math.abs(filteredTX - Constants.Climb.Vision.targetTX);

              double tzError = Math.abs(filteredTY - Constants.Climb.Vision.targetTY);

              return txError < 0.03 && tzError < 0.04;
            })
        .finallyDo(drive::stop)
        .withName("DockToClimb");
  }

  // ============================================================
  // CLIMBER COMMANDS (UNCHANGED LOGICALLY)
  // ============================================================

  public static Command waitForHome(ClimbSubsystem climb) {
    return Commands.waitUntil(climb::isHomed).withTimeout(2.0).withName("WaitForClimbHome");
  }

  public static Command hookUp(ClimbSubsystem climb) {
    return Commands.startEnd(climb::moveUp, climb::stop, climb).withName("HookUp");
  }

  public static Command climbDown(ClimbSubsystem climb) {
    return Commands.startEnd(climb::moveDown, climb::stop, climb).withName("ClimbDown");
  }

  public static Command stop(ClimbSubsystem climb) {
    return Commands.runOnce(climb::stop, climb).withName("ClimbStop");
  }

  public static Command home(ClimbSubsystem climb) {
    return climb.homeCommand().withName("ClimbHome");
  }

  public static Command autoClimberUp(ClimbSubsystem climb) {
    return climb.upCommand().withName("ClimbAutoUp");
  }

  // ============================================================
  // FULL SEQUENCE
  // ============================================================

  public static Command climbSequence(Drive drive, Vision vision, ClimbSubsystem climb) {

    return Commands.sequence(
            dockToClimb(drive).withTimeout(4.0),
            hookUp(climb).withTimeout(1.8),
            Commands.waitSeconds(0.4),
            climbDown(climb).withTimeout(1.0),
            stop(climb))
        .withName("FullClimbSequence");
  }
}
