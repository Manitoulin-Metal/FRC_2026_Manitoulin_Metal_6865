// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Generic camera IO interface used by AdvantageKit logging. */
public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    /** True if camera is actively publishing data. */
    public boolean connected = false;

    public Pose3d targetSpacePose = new Pose3d();
    public boolean hasTargets = false;

    /** Latest simple target observation (tx/ty). */
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    /** Pose observations from camera pipeline. */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    /** Visible AprilTag IDs. */
    public int[] tagIds = new int[0];
  }

  /** Simple tx / ty target angles. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Full robot pose sample from vision. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
