package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> robotPoseSupplier;

  private boolean enabled = true;

  public Vision(
      VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, Drive drive, VisionIO... io) {

    this.consumer = consumer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.io = io;

    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " disconnected.", AlertType.kWarning);
    }
  }

  // ==========================================================
  // BASIC HELPERS
  // ==========================================================

  public double getTX() {
    return inputs[REAR_CAMERA].latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    return inputs[REAR_CAMERA].latestTargetObservation.ty().getDegrees();
  }

  public boolean hasTag(int id) {
    for (int c = 0; c < inputs.length; c++) {
      for (int tag : inputs[c].tagIds) {
        if (tag == id) return true;
      }
    }
    return false;
  }

  // ==========================================================
  // FRONT CAMERA POSE
  // ==========================================================

  public Optional<Pose2d> getEstimatedPoseFromCamera1() {
    return getLatestPose(FRONT_CAMERA);
  }

  private Optional<Pose2d> getLatestPose(int camera) {
    Pose2d best = null;
    double newest = -999;

    for (var obs : inputs[camera].poseObservations) {
      if (obs.timestamp() > newest) {
        newest = obs.timestamp();
        best = obs.pose().toPose2d();
      }
    }

    return Optional.ofNullable(best);
  }

  // ==========================================================
  // REAR POSE
  // ==========================================================

  public Optional<Pose3d> getRearTargetSpacePoseForClimb() {

    if (!inputs[REAR_CAMERA].connected) return Optional.empty();
    if (!inputs[REAR_CAMERA].hasTargets) return Optional.empty();

    return Optional.of(inputs[REAR_CAMERA].targetSpacePose);
  }

  // ==========================================================
  // BEST POSE (SAFE FALLBACK)
  // ==========================================================

  public Optional<Pose2d> getBestEstimatedPose() {

    Optional<Pose2d> front = getEstimatedPoseFromCamera1();
    if (front.isPresent()) return front;

    Optional<Pose3d> rear = getRearTargetSpacePoseForClimb();
    if (rear.isPresent()) {
      Pose3d r = rear.get();
      return Optional.of(new Pose2d(r.getX(), r.getY(), r.getRotation().toRotation2d()));
    }

    return Optional.empty();
  }

  // ==========================================================
  // DOCKING TARGET FOR CLIMB (FROM REAR CAMERA)
  // ==========================================================

  public Optional<Transform2d> getDockingTarget() {

    // ==========================================================
    // SIM OVERRIDE
    // ==========================================================
    if (Constants.currentMode == Constants.Mode.SIM) {

      Pose2d robotPose = robotPoseSupplier.get();

      if (robotPose == null) return Optional.empty();

      int tagId = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 16 : 32;

      var tagOpt = VisionConstants.aprilTagLayout.getTagPose(tagId);

      if (tagOpt.isEmpty()) return Optional.empty();

      Pose2d tagPose = tagOpt.get().toPose2d();

      // ==========================================
      // OFFSET TARGET POSE
      // ==========================================
      Pose2d targetPose =
          tagPose.transformBy(
              new Transform2d(
                  1.15, // 1.15m in front of tag
                  0.3, //   0.3m to the right of tag (looking from above)
                  Rotation2d.kZero));

      // robot -> tag transform
      return Optional.of(new Transform2d(robotPose, targetPose));
    }
    // ==========================================================
    // REAL LIMELIGHT PATH (CLEAN ROBOT → TAG)
    // ==========================================================

    int desiredTag = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 16 : 32;

    double seenTag = LimelightHelpers.getFiducialID(Constants.Climb.Vision.REAR_LIMELIGHT);

    // Only trust correct tag
    if ((int) seenTag != desiredTag) {
      return Optional.empty();
    }

    double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.Climb.Vision.REAR_LIMELIGHT);

    // Safety check
    if (pose == null || pose.length < 6) {
      return Optional.empty();
    }

    // ==========================================================
    // RAW LIMELIGHT MEASUREMENT (robot → tag in tag frame)
    // ==========================================================

    double x = pose[0]; // forward (tag frame)
    double y = pose[1]; // right/left (tag frame)
    double yawDeg = pose[5];

    // Logging (safe, direct sensor view)
    Logger.recordOutput("Climb/RawForward", x);
    Logger.recordOutput("Climb/RawRight", y);
    Logger.recordOutput("Climb/RawYaw", yawDeg);

    // ==========================================================
    // RETURN PURE TRANSFORM
    // robot → tag (NO TARGET SUBTRACTION HERE)
    // ==========================================================

    return Optional.of(new Transform2d(new Translation2d(x, y), Rotation2d.fromDegrees(yawDeg)));
  }

  // ==========================================================
  // PERIODIC
  // ==========================================================

  @Override
  public void periodic() {

    if (!enabled) return;

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
    }

    List<Pose3d> accepted = new ArrayList<>();
    List<Pose3d> rejected = new ArrayList<>();

    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {

      for (var obs : inputs[cameraIndex].poseObservations) {

        boolean reject =
            obs.tagCount() == 0 || obs.ambiguity() > 0.3 || Math.abs(obs.pose().getZ()) > 1.0;

        if (reject) {
          rejected.add(obs.pose());
          continue;
        }

        accepted.add(obs.pose());

        consumer.accept(obs.pose().toPose2d(), obs.timestamp(), VecBuilder.fill(0.05, 0.05, 0.05));
      }
    }

    Logger.recordOutput("Vision/Accepted", accepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Rejected", rejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
