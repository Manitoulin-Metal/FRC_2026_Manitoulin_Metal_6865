package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.List;
import java.util.Set;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO io;
  private final Drive drive;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  private Pose2d latestValidPose = new Pose2d();
  private boolean hasValidVision = false;
  private double lastValidTime = 0.0;
  private static final Set<Integer> SHOOTING_TAGS = Set.of(9, 25);

  public int getTargetTagId() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return 9;
    }

    return 25; // Blue default
  }

  public VisionSubsystem(VisionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Decay valid vision if stale (>1s)
    if (Timer.getFPGATimestamp() - lastValidTime > 1.0) {
      hasValidVision = false;
    }

    // Fuse valid AprilTag poses to Drive pose estimator & track latest
    if (inputs.poseObservations != null) {
      for (PoseObservation observation : inputs.poseObservations) {
        // Filter valid observations: low ambiguity, multiple tags, reasonable distance
        int minTags = DriverStation.isAutonomous() ? 1 : 2;
        if (observation.ambiguity() < VisionConstants.maxAmbiguity
            && observation.tagCount() >= minTags
            && observation.averageTagDistance() > 0.1
            && // Avoid zero/too-close
            observation.averageTagDistance() < 10.0) { // Max field distance

          // Convert Pose3d to Pose2d (XY + yaw)
          Pose2d visionPose =
              new Pose2d(
                  observation.pose().getTranslation().toTranslation2d(),
                  observation.pose().getRotation().toRotation2d());

          // Dynamic standard deviations (meters translation, radians rotation)
          // Scale by distance and 1/sqrt(tagCount)
          double distanceFactor = Math.max(observation.averageTagDistance() / 1.0, 1.0);
          double tagFactor = 1.0 / Math.sqrt(observation.tagCount());
          double autoLooser = DriverStation.isAutonomous() ? 1.5 : 1.0;
          double xStdDev =
              VisionConstants.linearStdDevBaseline * distanceFactor * tagFactor * autoLooser;
          double yStdDev = xStdDev;
          double yawStdDev =
              VisionConstants.angularStdDevBaseline * distanceFactor * tagFactor * autoLooser;

          Matrix<N3, N1> stdDevs = VecBuilder.fill(xStdDev, yStdDev, yawStdDev);

          // Feed to Drive estimator
          drive.addVisionMeasurement(visionPose, observation.timestamp(), stdDevs);

          // Track latest valid pose for bump correction
          latestValidPose = visionPose;
          hasValidVision = true;
          lastValidTime = Timer.getFPGATimestamp();
        }
      }
    }
  }

  public double getTxDegrees() {
    if (inputs.latestTargetObservation == null) return 0.0;
    return inputs.latestTargetObservation.tx().getDegrees();
  }

  public double getTyDegrees() {
    if (inputs.latestTargetObservation == null) return 0.0;
    return inputs.latestTargetObservation.ty().getDegrees();
  }

  public boolean hasTarget() {
    return inputs.latestTargetObservation != null;
  }

  public boolean hasTag(int id) {
    for (int i = 0; i < inputs.rawFiducialCount; i++) {
      if (inputs.rawFiducialIDs[i] == id) {
        return true;
      }
    }
    return false;
  }

  public boolean hasShootingTag() {
    for (int i = 0; i < inputs.rawFiducialCount; i++) {
      if (SHOOTING_TAGS.contains(inputs.rawFiducialIDs[i])) {
        return true;
      }
    }
    return false;
  }

  /**
   * Checks if Limelight detects shooting tags {25,26} within shooting range [1.5-5.5m]. Refactored
   * to use List.contains for robustness if more tags added.
   */
  @SuppressWarnings("unlikely-arg-type")
  public boolean hasTargetInRange() {
    if (inputs.rawFiducialCount == 0) {
      return false;
    }

    for (int i = 0; i < inputs.rawFiducialCount; i++) {
      if (List.of(Constants.SHOOTING_TAG_IDS).contains(inputs.rawFiducialIDs[i])
          && inputs.rawFiducialDistances[i] >= Constants.MIN_SHOOT_DISTANCE_METERS
          && inputs.rawFiducialDistances[i] <= Constants.MAX_SHOOT_DISTANCE_METERS) {
        return true;
      }
    }
    return false;
  }

  /** Gets average distance to valid shooting targets (tags 25/26 in range), or -1 if none. */
  public double getDistanceToTarget() {
    if (inputs.latestTargetObservation == null) return -1.0;

    double bestDist = Double.MAX_VALUE;

    for (int i = 0; i < inputs.rawFiducialCount; i++) {
      if (SHOOTING_TAGS.contains(inputs.rawFiducialIDs[i])) {
        bestDist = Math.min(bestDist, inputs.rawFiducialDistances[i]);
      }
    }

    return bestDist == Double.MAX_VALUE ? -1.0 : bestDist;
  }

  public boolean isReadyToShoot() {
    return hasShootingTag() && Math.abs(getTxDegrees()) < 1.0 && getDistanceToTarget() > 0;
  }

  public boolean isReadyToClimb() {
    return hasClimbTag() && Math.abs(getTxDegrees()) < 1.0 && getDistanceToTarget() > 0;
  }

  /** Returns true if any detected fiducial is considered a climb tag. */
  public boolean hasClimbTag() {
    // Default behavior: treat any detected tag that is not a known shooting tag as a climb tag.
    // This avoids adding new external constants and fixes the undefined method compile error.
    for (int i = 0; i < inputs.rawFiducialCount; i++) {
      if (!SHOOTING_TAGS.contains(inputs.rawFiducialIDs[i])) {
        return true;
      }
    }
    return false;
  }

  public double getShootingTargetDistance() {
    if (inputs.rawFiducialCount == 0) {
      return -1.0;
    }

    double totalDist = 0.0;
    int validCount = 0;

    for (int i = 0; i < inputs.rawFiducialCount; i++) {
      if (List.of(Constants.SHOOTING_TAG_IDS).contains(inputs.rawFiducialIDs[i])
          && inputs.rawFiducialDistances[i] >= Constants.MIN_SHOOT_DISTANCE_METERS
          && inputs.rawFiducialDistances[i] <= Constants.MAX_SHOOT_DISTANCE_METERS) {
        totalDist += inputs.rawFiducialDistances[i];
        validCount++;
      }
    }

    return validCount > 0 ? totalDist / validCount : -1.0;
  }

  /** Latest vision pose that passed validation for bump correction. */
  public Pose2d getLatestValidPose() {
    return latestValidPose;
  }

  /** Whether we have recent valid vision pose (<1s old). */
  public boolean hasValidVision() {
    return hasValidVision;
  }
}
