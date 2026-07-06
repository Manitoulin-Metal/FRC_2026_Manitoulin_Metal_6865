package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  // ==========================================================
  // DOCK CACHE
  // ==========================================================

  private Optional<Transform2d> cachedDockTransform = Optional.empty();
  private Optional<Pose2d> cachedDockPose = Optional.empty();

  private double lastDockTime = -1.0;

  private static final double DOCK_TIMEOUT_SEC = 0.35;
  private static final int DOCK_HISTORY_SIZE = 6;

  private final ArrayDeque<Transform2d> dockHistory = new ArrayDeque<>();

  public Vision(VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, VisionIO... io) {

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

  // ==========================================================
  // BEST POSE
  // ==========================================================

  public Optional<Pose2d> getBestEstimatedPose() {
    Optional<Pose2d> front = getLatestPose(FRONT_CAMERA);
    if (front.isPresent()) return front;
    return getLatestPose(REAR_CAMERA);
  }

  private Optional<Pose2d> getLatestPose(int camera) {
    Pose2d best = null;
    double newest = -1;

    for (var obs : inputs[camera].poseObservations) {
      if (obs.timestamp() > newest) {
        newest = obs.timestamp();
        best = obs.pose().toPose2d();
      }
    }
    return Optional.ofNullable(best);
  }

  // ==========================================================
  // PUBLIC API (USED BY DRIVE)
  // ==========================================================

  public Pose2d getDockTargetPose() {
    return cachedDockPose.orElseGet(
        () -> getDockingTransform().map(t -> robotPoseSupplier.get().plus(t)).orElse(new Pose2d()));
  }

  public Optional<Pose2d> getDockingPose() {
    return cachedDockPose;
  }

  public boolean hasFreshDockTarget() {
    return Timer.getFPGATimestamp() - lastDockTime < DOCK_TIMEOUT_SEC;
  }

  // ==========================================================
  // CORE DOCK LOGIC
  // ==========================================================

  private Optional<Transform2d> getDockingTransform() {

    // =========================
    // SIMULATION (perfect clone)
    // =========================
    if (Constants.currentMode == Constants.Mode.SIM) {

      Pose2d robot = robotPoseSupplier.get();
      if (robot == null) return Optional.empty();

      int tagId =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
              ? Constants.Climb.Hardware.CLIMB_TAG_IDS[1]
              : Constants.Climb.Hardware.CLIMB_TAG_IDS[0];

      var tagOpt = VisionConstants.aprilTagLayout.getTagPose(tagId);
      if (tagOpt.isEmpty()) return Optional.empty();

      Pose2d tagPose = tagOpt.get().toPose2d();

      Transform2d robotToTag = new Transform2d(robot, tagPose);

      updateCache(robot, robotToTag);

      return Optional.of(robotToTag);
    }

    // =========================
    // REAL LIMELIGHT
    // =========================
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    int desiredTag =
        alliance == Alliance.Red
            ? Constants.Climb.Hardware.CLIMB_TAG_IDS[1]
            : Constants.Climb.Hardware.CLIMB_TAG_IDS[0];

    double seenTag = LimelightHelpers.getFiducialID(Constants.Climb.Vision.REAR_LIMELIGHT);

    if ((int) seenTag != desiredTag) return getCachedIfFresh();

    double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.Climb.Vision.REAR_LIMELIGHT);

    if (pose == null || pose.length < 6) return getCachedIfFresh();

    Transform2d robotToTag =
        new Transform2d(new Translation2d(-pose[2], pose[0]), Rotation2d.fromDegrees(pose[5]));

    updateCache(robotPoseSupplier.get(), robotToTag);

    return Optional.of(robotToTag);
  }

  // ==========================================================
  // CACHE SYSTEM
  // ==========================================================

  private void updateCache(Pose2d robot, Transform2d transform) {

    dockHistory.addLast(transform);

    if (dockHistory.size() > DOCK_HISTORY_SIZE) {
      dockHistory.removeFirst();
    }

    Transform2d filtered = averageTransform(dockHistory);

    cachedDockTransform = Optional.of(filtered);
    cachedDockPose = Optional.of(robot.plus(filtered));
    lastDockTime = Timer.getFPGATimestamp();
  }

  private Optional<Transform2d> getCachedIfFresh() {
    return hasFreshDockTarget() ? cachedDockTransform : Optional.empty();
  }

  private static Transform2d averageTransform(Collection<Transform2d> history) {

    if (history.isEmpty()) return new Transform2d();

    double x = 0;
    double y = 0;
    double sin = 0;
    double cos = 0;

    for (Transform2d t : history) {
      x += t.getX();
      y += t.getY();
      sin += Math.sin(t.getRotation().getRadians());
      cos += Math.cos(t.getRotation().getRadians());
    }

    int n = history.size();

    return new Transform2d(
        new Translation2d(x / n, y / n), new Rotation2d(Math.atan2(sin / n, cos / n)));
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

    for (int cam = 0; cam < inputs.length; cam++) {
      for (var obs : inputs[cam].poseObservations) {

        boolean reject =
            obs.tagCount() == 0 || obs.ambiguity() > 0.3 || Math.abs(obs.pose().getZ()) > 1.0;

        if (reject) continue;

        // consumer.accept(obs.pose().toPose2d(), obs.timestamp(), VecBuilder.fill(0.05, 0.05,
        // 0.05));
      }
    }

    Logger.recordOutput("Dock/FreshTarget", hasFreshDockTarget());
    Logger.recordOutput("Dock/HistorySize", dockHistory.size());

    cachedDockTransform.ifPresent(t -> Logger.recordOutput("Dock/FilteredTransform", t));

    cachedDockPose.ifPresent(p -> Logger.recordOutput("Dock/FilteredPose", p));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
