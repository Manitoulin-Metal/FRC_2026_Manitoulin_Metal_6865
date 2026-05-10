package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {

  private VisionConstants() {}

  /** 2026 field */
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // =========================================================
  // Camera Names
  // =========================================================

  public static final String rearCameraName = "limelight-back";

  public static final String frontCameraName = "limelight-forward";

  // =========================================================
  // Camera Indexes
  // =========================================================

  public static final int REAR_CAMERA = 0;
  public static final int FRONT_CAMERA = 1;

  // =========================================================
  // Camera Mounting Positions
  // Robot center -> camera
  // =========================================================

  /** Rear camera */
  public static final Transform3d robotToRearCamera =
      new Transform3d(-0.20, 0.00, 0.44, new Rotation3d(0.0, -0.35, Math.PI));

  /** Front camera */
  public static final Transform3d robotToFrontCamera =
      new Transform3d(0.20, 0.00, 0.44, new Rotation3d(0.0, -0.35, 0.0));

  // =========================================================
  // Pose Rejection Thresholds
  // =========================================================

  public static final double maxAmbiguity = 0.30;

  public static final double maxZError = 0.75;

  public static final double maxAngularVelocityDegPerSec = 360.0;

  // =========================================================
  // Vision Std Dev Baselines
  // =========================================================

  public static final double linearStdDevBaseline = 0.02;

  public static final double angularStdDevBaseline = 0.06;

  public static final double[] cameraStdDevFactors = {
    1.4, // rear less trusted globally
    1.0 // front primary odometry correction
  };

  public static final double linearStdDevMegatag2Factor = 0.50;

  public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
