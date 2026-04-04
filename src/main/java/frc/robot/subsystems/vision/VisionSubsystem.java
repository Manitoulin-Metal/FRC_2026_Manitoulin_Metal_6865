package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final Drive drive;

  public VisionSubsystem(VisionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  // ========================= TAG DETECTION =========================

  public boolean hasTag(int tagId) {
    for (int id : inputs.tagIds) {
      if (id == tagId) return true;
    }
    return false;
  }

  public boolean hasAnyTag() {
    return inputs.tagIds.length > 0;
  }

  public int[] getVisibleTagIDs() {
    return inputs.tagIds;
  }

  // ========================= BEST OBSERVATION =========================

  public PoseObservation getBestObservation() {
    if (inputs.poseObservations.length == 0) return null;

    PoseObservation best = null;
    double bestScore = -1;

    for (PoseObservation obs : inputs.poseObservations) {

      if (obs.tagCount() == 0) continue;
      if (obs.ambiguity() > 0.3) continue;

      double score = 0;

      if (obs.type() == VisionIO.PoseObservationType.MEGATAG_2) {
        score += 2.0;
      }

      score += obs.tagCount();
      score += 1.0 / (obs.averageTagDistance() + 0.001);

      if (best == null || score > bestScore) {
        best = obs;
        bestScore = score;
      }
    }

    return best;
  }

  // ========================= POSE =========================

  public Pose2d getEstimatedPose() {
    PoseObservation obs = getBestObservation();
    if (obs == null) return null;
    return obs.pose().toPose2d();
  }

  public Pose2d getBestRobotPose() {
    Pose2d visionPose = getEstimatedPose();
    if (visionPose != null) {
      return visionPose;
    }
    return drive.getPose();
  }

  // ========================= DISTANCE =========================

  public double getAverageDistance() {
    PoseObservation obs = getBestObservation();
    if (obs == null) return 999;
    return obs.averageTagDistance();
  }

  // ========================= TX / TY =========================

  public double getTX() {
    return inputs.latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    return inputs.latestTargetObservation.ty().getDegrees();
  }

  // ========================= CLIMB TARGET =========================

  public Optional<Pose2d> getClimbTargetPose(AprilTagFieldLayout layout) {

    if (!hasTag(Constants.CLIMB_TAG_ID)) return Optional.empty();

    Optional<Pose3d> tagPose3d = layout.getTagPose(Constants.CLIMB_TAG_ID);
    if (tagPose3d.isEmpty()) return Optional.empty();

    Pose2d tagPose = tagPose3d.get().toPose2d();
    Pose2d targetPose = tagPose.transformBy(Constants.CLIMB_OFFSET);

    return Optional.of(targetPose);
  }

  public Optional<Transform2d> getClimbError(AprilTagFieldLayout layout) {

    Optional<Pose2d> targetOpt = getClimbTargetPose(layout);
    if (targetOpt.isEmpty()) return Optional.empty();

    Pose2d robotPose = getBestRobotPose();
    Pose2d targetPose = targetOpt.get();

    return Optional.of(targetPose.minus(robotPose));
  }

  public boolean isClimbAligned(AprilTagFieldLayout layout) {
    Optional<Transform2d> errorOpt = getClimbError(layout);
    if (errorOpt.isEmpty()) return false;

    Transform2d error = errorOpt.get();

    boolean posGood =
        Math.abs(error.getX()) < Constants.CLIMB_POS_TOLERANCE
            && Math.abs(error.getY()) < Constants.CLIMB_POS_TOLERANCE;

    boolean rotGood = Math.abs(error.getRotation().getRadians()) < Constants.CLIMB_ROT_TOLERANCE;

    return posGood && rotGood;
  }

  // ========================= STATUS =========================

  public boolean isReadyToClimb() {
    return hasTag(Constants.CLIMB_TAG_ID) && getBestObservation() != null;
  }

  // ========================= PERIODIC =========================

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    SmartDashboard.putBoolean("Vision/Connected", inputs.connected);
    SmartDashboard.putNumber("Vision/TagCount", inputs.tagIds.length);
    SmartDashboard.putNumber("Vision/Distance", getAverageDistance());
    SmartDashboard.putNumber("Vision/TX", getTX());
    SmartDashboard.putNumber("Vision/TY", getTY());

    PoseObservation best = getBestObservation();
    if (best != null) {
      SmartDashboard.putNumber("Vision/Ambiguity", best.ambiguity());
    }
  }
}
