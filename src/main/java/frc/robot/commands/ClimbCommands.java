package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;

public final class ClimbCommands {

    private ClimbCommands() {
    }

    // -----------------------------
    // Teleop: align to cage tag then climb
    // -----------------------------

    /**
     * Aligns to the alliance-appropriate cage AprilTag (with a 5-second timeout),
     * then runs the
     * climber upward until interrupted.
     */
    public static Command alignAndClimb(Drive drive, VisionSubsystem vision, ClimbSubsystem climb) {
        return DriveCommands.alignToTag(DriveCommands.getClimbTagId(), drive, vision)
                .withTimeout(5.0)
                .andThen(climb.climbCommand(0.75));
    }

    // -----------------------------
    // Auto: drive to climb position (field-relative)
    // -----------------------------

    /**
     * Drives to the cage AprilTag using field-relative odometry (used in
     * PathPlanner named
     * commands).
     */
    public static Command autoClimbDrive(Drive drive, AprilTagFieldLayout fieldLayout) {
        return DriveCommands.driveToClimb(
                drive,
                fieldLayout,
                new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
                1.5,
                3.0);
    }

    /** Runs the climber upward for autonomous sequences. */
    public static Command autoClimbUp(ClimbSubsystem climb) {
        return climb.climbCommand(0.5).withTimeout(4);
    }

    /** Runs the climber downward for autonomous sequences. */
    public static Command autoClimbDown(ClimbSubsystem climb) {
        return climb.climbCommand(-0.5).withTimeout(6);
    }

    /**
     * Field-relative climb alignment using camera-reported robot-relative error.
     */
    public static Command driveToClimbVision(Drive drive, VisionSubsystem vision) {
        return Commands.run(
                () -> {
                    double forward = 0;
                    double strafe = 0;
                    double turn = 0;

                    if (!vision.shouldUseVisionForClimb()) {
                        // Search mode
                        turn = 0.5;
                    } else {
                        var errorOpt = vision.getRobotRelativeError();

                        if (errorOpt.isEmpty()) {
                            drive.stop();
                            return;
                        }

                        Transform2d error = errorOpt.get();

                        forward = error.getX() * Constants.CLIMB_kP_FORWARD;
                        strafe = error.getY() * Constants.CLIMB_kP_STRAFE;
                        turn = error.getRotation().getRadians() * Constants.CLIMB_kP_TURN;

                        if (Math.abs(error.getX()) < 0.5) {
                            forward = 0;
                        }
                        if (Math.abs(error.getY()) < 0.5) {
                            strafe = 0;
                        }
                        if (Math.abs(error.getRotation().getDegrees()) < 1.0) {
                            turn = 0;
                        }
                    }

                    forward = MathUtil.clamp(forward, -1.0, 1.0);
                    strafe = MathUtil.clamp(strafe, -1.0, 1.0);
                    turn = MathUtil.clamp(turn, -1.0, 1.0);

                    drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
                },
                drive);
    }

    /** Limelight-only climb alignment fallback command. */
    public static Command limelightClimbFull(Drive drive, VisionSubsystem vision) {
        return Commands.run(
                () -> {
                    boolean seesTag = vision.hasTag(Constants.CLIMB_TAG_ID);

                    double forward = 0;
                    double strafe = 0;
                    double turn = 0;

                    if (!seesTag) {
                        turn = 0.5;
                    } else {
                        double tx = vision.getTX();
                        double ty = vision.getTY();

                        double targetTX = 0.0;
                        double targetTY = 9.15;

                        double kTurn = 0.035;
                        double kForward = 0.08;
                        double kStrafe = 0.025;

                        double errorX = targetTX - tx;
                        double errorY = targetTY - ty;

                        turn = errorX * kTurn;
                        forward = errorY * kForward;
                        strafe = errorX * kStrafe;

                        if (Math.abs(errorX) < 1.0) {
                            turn = 0;
                            strafe = 0;
                        }

                        if (Math.abs(errorY) < 0.5) {
                            forward = 0;
                        }
                    }

                    turn = MathUtil.clamp(turn, -1.0, 1.0);
                    forward = MathUtil.clamp(forward, -1.0, 1.0);
                    strafe = MathUtil.clamp(strafe, -1.0, 1.0);

                    drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
                },
                drive);
    }

    /** Logs current offset from a requested tag pose to SmartDashboard. */
    public static Command logClimbOffset(Drive drive, AprilTagFieldLayout fieldLayout, int tagId) {
        return Commands.runOnce(
                () -> {
                    var tagPoseOpt = fieldLayout.getTagPose(tagId);
                    if (tagPoseOpt.isEmpty()) {
                        SmartDashboard.putString("ClimbOffset/Status", "Tag not found: " + tagId);
                        return;
                    }

                    Pose2d tagPose = tagPoseOpt.get().toPose2d();
                    Pose2d robotPose = drive.getPose();
                    Transform2d offset = new Transform2d(tagPose, robotPose);

                    SmartDashboard.putNumber("ClimbOffset/X", offset.getX());
                    SmartDashboard.putNumber("ClimbOffset/Y", offset.getY());
                    SmartDashboard.putNumber("ClimbOffset/RotDeg", offset.getRotation().getDegrees());
                    SmartDashboard.putString("ClimbOffset/Status", "OK");
                });
    }
}
