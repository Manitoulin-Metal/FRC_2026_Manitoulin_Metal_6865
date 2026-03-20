// Copyright (c) 2026 FRC 6865 Manitoulin Metal
// Vision measurement processing for Limelight data using VisionUtil logic

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionUtil.VisionMode;
import java.util.ArrayList;
import java.util.List;

/**
 * Processes Limelight vision data into validated VisionMeasurements for fusion. Uses VisionUtil
 * POOF/MA modes with Limelight-specific handling.
 */
public class VisionMeasurement {
  private static VisionMode mode = VisionMode.POOF;

  /** Processes VisionIOInputs (Limelight) into List of validated VisionMeasurements. */
  public static List<VisionUtil.VisionMeasurement> processLimelightData(VisionIOInputs inputs) {
    List<VisionUtil.VisionMeasurement> measurements = new ArrayList<>();

    if (!inputs.connected || inputs.poseObservations == null) {
      SmartDashboard.putNumber("Vision/NumMeasurements", 0);
      return measurements;
    }

    // Primary: botpose_wpiblue PoseEstimate (recommended for fusion)
    PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (botPoseEstimate.tagCount > 0) {
      VisionUtil.VisionMeasurement measurement = mode.getVisionMeasurement(botPoseEstimate);
      if (mode.acceptVisionMeasurement(botPoseEstimate)) { // Fixed: direct PoseEstimate
        measurements.add(measurement);
      }
    }

    // Fallback/secondary: Process individual PoseObservations
    for (PoseObservation obs : inputs.poseObservations) {
      if (obs.ambiguity() < 0.3 && obs.tagCount() >= 1) {
        // Create synthetic PoseEstimate from observation
        PoseEstimate syntheticEst =
            new PoseEstimate(
                new Pose2d(
                    obs.pose().getX(), obs.pose().getY(), obs.pose().getRotation().toRotation2d()),
                obs.timestamp(),
                0, // latency from obs if available
                obs.tagCount(),
                0, // span
                obs.averageTagDistance(),
                0, // area (use rawFiducials avg if needed)
                new LimelightHelpers.RawFiducial[0],
                obs.type() == VisionIO.PoseObservationType.MEGATAG_2);
        VisionUtil.VisionMeasurement measurement = mode.getVisionMeasurement(syntheticEst);
        if (mode.acceptVisionMeasurement(syntheticEst)) {
          measurements.add(measurement);
        }
      }
    }

    // Log to SmartDashboard/NT
    SmartDashboard.putNumber("Vision/NumMeasurements", measurements.size());
    if (!measurements.isEmpty()) {
      VisionUtil.VisionMeasurement best = getBestMeasurement(measurements);
      SmartDashboard.putNumber("Vision/BestX", best.poseEstimate().pose.getX());
      SmartDashboard.putNumber("Vision/BestY", best.poseEstimate().pose.getY());
      SmartDashboard.putNumber("Vision/BestStdDevXY", best.visionMeasurementStdDevs().get(0, 0));
    }

    return measurements;
  }

  /** Selects best measurement (lowest stdDev, prefer multi-tag). */
  public static VisionUtil.VisionMeasurement getBestMeasurement(
      List<VisionUtil.VisionMeasurement> measurements) {
    if (measurements.isEmpty()) return null;

    VisionUtil.VisionMeasurement best = measurements.get(0);
    double bestScore = scoreMeasurement(best);

    for (VisionUtil.VisionMeasurement m : measurements) {
      double score = scoreMeasurement(m);
      if (score < bestScore) {
        bestScore = score;
        best = m;
      }
    }
    return best;
  }

  private static double scoreMeasurement(VisionUtil.VisionMeasurement m) {
    PoseEstimate est = m.poseEstimate();
    double xyDev = m.visionMeasurementStdDevs().get(0, 0);
    double score = xyDev * xyDev; // Squared for emphasis
    if (est.tagCount >= 2) score *= 0.5; // Prefer multi-tag
    if (est.isMegaTag2()) score *= 0.8; // Slight MT2 preference
    return score;
  }

  public static void setMode(VisionMode newMode) {
    mode = newMode;
  }

  public static VisionMode getMode() {
    return mode;
  }
}
