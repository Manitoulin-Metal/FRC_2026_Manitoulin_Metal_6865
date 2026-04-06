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
    return obs != null; // no ambiguity check needed with current Limelight v3
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

  // Function to convert angular tx and ty to a 2D translation in robot space
  // (forward, strafe)
  public double[] getCameraToTagOffset(
      AprilTagFieldLayout fieldLayout, int tagId, double tx, double ty) {
    // Get tag pose from the field layout
    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
    if (tagPoseOpt.isEmpty()) {
      // Tag not found in layout, return NaN array
      return new double[] { Double.NaN, Double.NaN, Double.NaN };
    }

    Pose3d tagPose = tagPoseOpt.get();

    // Convert angles from degrees to radians
    double txRad = Math.toRadians(tx);
    double tyRad = Math.toRadians(ty);

    // Forward offset (x) using vertical angle
    double deltaZ = tagPose.getZ() - VisionConstants.CAMERA_UP; // Vertical difference between camera and tag
    double forwardDistance = deltaZ / Math.tan(tyRad);

    // Lateral offset (y) using horizontal angle
    double lateralOffset = forwardDistance * Math.tan(txRad);

    // Straight-line distance
    double distance = Math.sqrt(
        forwardDistance * forwardDistance + lateralOffset * lateralOffset + deltaZ * deltaZ);

    return new double[] { forwardDistance, lateralOffset, distance };
  }

  // ========================= STATUS =========================

  public boolean isReadyToClimb() {
    return shouldUseVisionForClimb();
  }

  // ========================= PERIODIC =========================

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // // -------- Debug table of all detected tags (for tuning) --------
    // StringBuilder table = new StringBuilder();
    // table.append("| ID | TX | TY | Angle |\n");
    // table.append("|---|---|---|---|\n");

    // // Loop through all detected tag IDs
    // for (int i = 0; i < inputs.tagIds.length; i++) {
    // int tagId = inputs.tagIds[i];
    // double tx = 0.0;
    // double ty = 0.0;
    // double angle = Double.NaN;

    // if (inputs.latestTargetObservation != null &&
    // inputs.tagIds().contains(tagId)) {
    // // Use latest observation for that tag
    // tx = inputs.latestTargetObservation.tx().getDegrees();
    // ty = inputs.latestTargetObservation.ty().getDegrees();

    // // Compute angle relative to robot
    // Optional<Pose3d> tagPoseOpt =
    // VisionConstants.aprilTagLayout.getTagPose(tagId);
    // if (tagPoseOpt.isPresent()) {
    // Pose2d tagPose = tagPoseOpt.get().toPose2d();
    // Pose2d robotPose = drive.getPose();
    // angle =
    // Math.toDegrees(
    // tagPose
    // .getTranslation()
    // .minus(robotPose.getTranslation())
    // .getAngle()
    // .getRadians());
    // }
    // }

    // table.append(String.format("| %d | %.2f | %.2f | %.1f |\n", tagId, tx, ty,
    // angle));
    // }

    // SmartDashboard.putString("Test/AprilTagTable", table.toString());

    // ------- End of debug table -------
    SmartDashboard.putBoolean("Vision/HasTag", hasAnyTag());
    SmartDashboard.putBoolean("Vision/Stable", hasStableTarget());
    SmartDashboard.putBoolean("Vision/UseVision", shouldUseVisionForClimb());

    SmartDashboard.putNumber("Vision/TX", getTX());
    SmartDashboard.putNumber("Vision/TY", getTY());
  }
}
