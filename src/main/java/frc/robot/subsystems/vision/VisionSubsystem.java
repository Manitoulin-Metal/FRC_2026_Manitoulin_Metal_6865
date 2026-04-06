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

  // Function to convert angular tx and ty to a 2D translation in robot space
  // (forward, strafe)
  public static double[] calculateCameraToTagOffsets(
      VisionSubsystem visionClimb,
      AprilTagFieldLayout fieldLayout,
      int tagId) {

    // Camera position relative to robot center
    final double camForward = 0.184;
    final double camRight = -0.1651;
    final double camUp = 0.441425;

    // Get tag height from field layout
    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);

    if (tagPoseOpt.isEmpty()) {
      return new double[] { 0.0, 0.0, 0.0 };
    }

    double tagHeight = tagPoseOpt.get().getZ();

    // Get Limelight angles
    double txRad = Math.toRadians(visionClimb.getTX());
    double tyRad = Math.toRadians(visionClimb.getTY());

    // Vertical height difference
    double dz = tagHeight - camUp;

    // Forward distance from camera to tag
    double xOffset = dz / Math.tan(tyRad);

    // Side-to-side offset
    double yOffset = xOffset * Math.tan(txRad);

    // Apply camera position offset
    xOffset += camForward;
    yOffset += camRight;

    // True 3D straight-line distance
    double distance3d = Math.sqrt(xOffset * xOffset + yOffset * yOffset + dz * dz);

    return new double[] { xOffset, yOffset, distance3d };
  }

  public double getDistanceToTag(
      AprilTagFieldLayout fieldLayout,
      int tagId) {

    Optional<Translation3d> offset = getCameraToTagOffset(fieldLayout, tagId);

    if (offset.isEmpty()) {
      return Double.NaN;
    }

    return offset.get().getNorm();
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
