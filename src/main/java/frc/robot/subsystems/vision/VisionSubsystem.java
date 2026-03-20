package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO io;
  private final Drive drive;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public VisionSubsystem(VisionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Fuse valid AprilTag poses to Drive pose estimator
    if (inputs.poseObservations != null) {
      for (PoseObservation observation : inputs.poseObservations) {
        // Filter valid observations: low ambiguity, multiple tags, reasonable distance
        if (observation.ambiguity() < VisionConstants.maxAmbiguity
            && observation.tagCount() >= 2
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
          double xStdDev = VisionConstants.linearStdDevBaseline * distanceFactor * tagFactor;
          double yStdDev = xStdDev;
          double yawStdDev = VisionConstants.angularStdDevBaseline * distanceFactor * tagFactor;

          Matrix<N3, N1> stdDevs = VecBuilder.fill(xStdDev, yStdDev, yawStdDev);

          // Feed to Drive estimator
          drive.addVisionMeasurement(visionPose, observation.timestamp(), stdDevs);
        }
      }
    }
  }

  public boolean hasTag(int id) {
    return inputs.latestTargetObservation != null;
  }

  public double getTX() {
    return inputs.latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    return inputs.latestTargetObservation.ty().getDegrees();
  }

  /** Checks if Limelight detects shooting tags 25/26 within shooting range */
  public boolean hasTargetInRange() {
    if (inputs.rawFiducials == null || inputs.rawFiducials.length == 0) {
      return false;
    }

    for (frc.robot.LimelightHelpers.RawFiducial fiducial : inputs.rawFiducials) {
      int id = fiducial.id;
      double dist = fiducial.distToRobot;

      // Check if it's a shooting tag and within range
      if ((id == frc.robot.Constants.SHOOTING_TAG_IDS[0]
              || id == frc.robot.Constants.SHOOTING_TAG_IDS[1])
          && dist >= frc.robot.Constants.MIN_SHOOT_DISTANCE_METERS
          && dist <= frc.robot.Constants.MAX_SHOOT_DISTANCE_METERS) {
        return true;
      }
    }
    return false;
  }
}
