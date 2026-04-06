package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

import java.util.List;
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
      if (id == tagId)
        return true;
    }
    return false;
  }

  public boolean hasAnyTag() {
    return inputs.tagIds.length > 0;
  }

  // ========================= STABILITY =========================

  public boolean hasStableTarget() {
    PoseObservation obs = getBestObservation();
    return obs != null && obs.tagCount() > 0 && obs.ambiguity() < 0.2;
  }

  public boolean shouldUseVisionForClimb() {
    return hasTag(Constants.CLIMB_TAG_ID) && hasStableTarget();
  }

  // ========================= OBSERVATION =========================

  public PoseObservation getBestObservation() {
    if (inputs.poseObservations.length == 0)
      return null;

    PoseObservation best = null;
    double bestScore = -1;

    for (PoseObservation obs : inputs.poseObservations) {

      if (obs.tagCount() == 0)
        continue;
      if (obs.ambiguity() > 0.3)
        continue;

      double score = obs.tagCount() + (1.0 / (obs.averageTagDistance() + 0.001));

      if (obs.type() == VisionIO.PoseObservationType.MEGATAG_2) {
        score += 2.0;
      }

      if (best == null || score > bestScore) {
        best = obs;
        bestScore = score;
      }
    }

    return best;
  }

  // ========================= FIELD SPACE =========================

  public Pose2d getEstimatedPose() {
    PoseObservation obs = getBestObservation();
    return (obs == null) ? null : obs.pose().toPose2d();
  }

  public Pose2d getBestRobotPose() {
    Pose2d visionPose = getEstimatedPose();
    return (visionPose != null) ? visionPose : drive.getPose();
  }

  // ========================= ROBOT SPACE (KEY) =========================

  public double getTX() {
    if (!hasAnyTag())
      return 0.0;
    return inputs.latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    if (!hasAnyTag())
      return 0.0;
    return inputs.latestTargetObservation.ty().getDegrees();
  }

  public Optional<Transform2d> getRobotRelativeError() {
    if (!hasAnyTag())
      return Optional.empty();

    double tx = getTX();
    double ty = getTY();

    double forwardError = Constants.CLIMB_TARGET_TY - ty;
    double strafeError = tx * Constants.CLIMB_STRAFE_FACTOR;
    Rotation2d rotationError = Rotation2d.fromDegrees(tx);

    return Optional.of(
        new Transform2d(new Translation2d(forwardError, strafeError), rotationError));
  }

  // ========================= STATUS =========================

  public boolean isReadyToClimb() {
    return shouldUseVisionForClimb();
  }

  // ========================= PERIODIC =========================

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // -------- Debug table of all detected tags (for tuning) --------
    // Example: get detected tags
    List<Integer> tagIDs = getDetectedTagIDs();
    List<Double> txs = getTXs(); // horizontal offset
    List<Double> tys = getTYs(); // vertical offset
    List<Double> dists = getDistances(); // distance to tag
    List<Double> angles = new ArrayList<>(); // angle relative to robot

    // Compute angles from pose if you have tag poses
    for (int id : tagIDs) {
      Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(id);
      if (tagPoseOpt.isPresent()) {
        Pose2d tagPose = tagPoseOpt.get().toPose2d();
        Pose2d robotPose = drive.getPose();
        Rotation2d angleToTag = tagPose.getTranslation().minus(robotPose.getTranslation()).getAngle();
        angles.add(Math.toDegrees(angleToTag.getRadians()));
      } else {
        angles.add(Double.NaN);
      }
    }

    // Build table
    StringBuilder table = new StringBuilder();
    table.append(String.format("| ID | TX | TY | Dist | Angle |\n"));
    table.append("|---|---|---|---|---|\n");

    for (int i = 0; i < tagIDs.size(); i++) {
      table.append(String.format(
          "| %d | %.2f | %.2f | %.2f | %.1f |\n",
          tagIDs.get(i), txs.get(i), tys.get(i), dists.get(i), angles.get(i)));
    }

    // Publish to SmartDashboard / Elastic
    SmartDashboard.putString("Test/AprilTagTable", table.toString());

    // ------- End of debug table -------

    SmartDashboard.putBoolean("Vision/HasTag",

        hasAnyTag());
    SmartDashboard.putBoolean("Vision/Stable", hasStableTarget());
    SmartDashboard.putBoolean("Vision/UseVision", shouldUseVisionForClimb());

    SmartDashboard.putNumber("Vision/TX", getTX());
    SmartDashboard.putNumber("Vision/TY", getTY());
  }
}
