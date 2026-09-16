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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  // =========================================================
  // CORE DEPENDENCIES
  // =========================================================
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnected;

  private final Supplier<Pose2d> robotPoseSupplier;

  private boolean enabled = true;

  // =========================================================
  // DOCKING STATE
  // =========================================================
  private final ArrayDeque<Transform2d> dockHistory = new ArrayDeque<>();
  private static final int DOCK_HISTORY_SIZE = 6;
  private static final double DOCK_TIMEOUT = 0.35;

  private Optional<Transform2d> cachedDock = Optional.empty();
  private double lastDockTimestamp = 0;

  // =========================================================
  // DOCK MEASUREMENT DATA
  // =========================================================
  public record DockMeasurement(double forward, double strafe, double yawDeg) {}

  // =========================================================
  // CONSTRUCTOR
  // =========================================================
  public Vision(VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, VisionIO... io) {

    this.consumer = consumer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.io = io;

    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnected = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnected[i] = new Alert("Vision camera " + i + " disconnected.", AlertType.kWarning);
    }
  }

  // =========================================================
  // SIMPLE CAMERA HELPERS
  // =========================================================
  public double getTX() {
    return inputs[REAR_CAMERA].latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    return inputs[REAR_CAMERA].latestTargetObservation.ty().getDegrees();
  }

  // =========================================================
  // POSE ESTIMATION (FRONT PRIORITY)
  // =========================================================
  public Optional<Pose2d> getBestEstimatedPose() {
    Optional<Pose2d> front = getLatestPose(FRONT_CAMERA);
    if (front.isPresent()) return front;
    return getLatestPose(REAR_CAMERA);
  }

  private Optional<Pose2d> getLatestPose(int cam) {
    Pose2d best = null;
    double latest = -1;

    for (var obs : inputs[cam].poseObservations) {
      if (obs.timestamp() > latest) {
        latest = obs.timestamp();
        best = obs.pose().toPose2d();
      }
    }

    return Optional.ofNullable(best);
  }

  // =========================================================
  // DOCK API (FOR DRIVE COMMANDS)
  // =========================================================

  public Optional<Transform2d> getDockTransform() {
    return hasFreshDock() ? cachedDock : Optional.empty();
  }

  public Pose2d getDockTargetPose() {
    Pose2d robot = robotPoseSupplier.get();
    return getDockTransform().map(robot::plus).orElse(robot);
  }

  public boolean hasFreshDock() {
    return (Timer.getFPGATimestamp() - lastDockTimestamp) < DOCK_TIMEOUT;
  }

  // =========================================================
  // DOCK FILTERING
  // =========================================================
  private Transform2d average(Collection<Transform2d> list) {
    if (list.isEmpty()) return new Transform2d();

    double x = 0, y = 0;
    double sin = 0, cos = 0;

    for (Transform2d t : list) {
      x += t.getX();
      y += t.getY();
      sin += Math.sin(t.getRotation().getRadians());
      cos += Math.cos(t.getRotation().getRadians());
    }

    int n = list.size();

    return new Transform2d(
        new Translation2d(x / n, y / n), new Rotation2d(Math.atan2(sin / n, cos / n)));
  }

  // =========================================================
  // DOCK COMPUTE CORE
  // =========================================================
  private void updateDock(Transform2d raw) {

    dockHistory.addLast(raw);
    if (dockHistory.size() > DOCK_HISTORY_SIZE) {
      dockHistory.removeFirst();
    }

    Transform2d filtered = average(dockHistory);

    cachedDock = Optional.of(filtered);
    lastDockTimestamp = Timer.getFPGATimestamp();

    Logger.recordOutput("Dock/Raw", raw);
    Logger.recordOutput("Dock/Filtered", filtered);
    Logger.recordOutput("Dock/HistorySize", dockHistory.size());

    Logger.recordOutput("Dock/TargetForward", filtered.getX());
    Logger.recordOutput("Dock/TargetStrafe", filtered.getY());
    Logger.recordOutput("Dock/TargetYawDeg", filtered.getRotation().getDegrees());
  }

  // =========================================================
  // PERIODIC
  // =========================================================
  @Override
  public void periodic() {

    if (!enabled) return;

    // =====================================================
    // CAMERA UPDATES
    // =====================================================
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      disconnected[i].set(!inputs[i].connected);
      Logger.processInputs("Vision/Cam" + i, inputs[i]);
    }

    // =====================================================
    // POSE ESTIMATION (ONLY REAL FIELD POSES)
    // =====================================================
    for (int cam = 0; cam < inputs.length; cam++) {
      for (var obs : inputs[cam].poseObservations) {

        if (obs.tagCount() == 0 || obs.ambiguity() > 0.3) continue;

        consumer.accept(obs.pose().toPose2d(), obs.timestamp(), VecBuilder.fill(0.05, 0.05, 0.05));
      }
    }

    // =====================================================
    // DOCKING (REAR CAMERA ONLY)
    // =====================================================
    computeDocking();
    Logger.recordOutput("Dock/HasFreshDock", hasFreshDock());
  }

  // =========================================================
  // DOCKING LOGIC (SIM + REAL UNIFIED)
  // =========================================================
  private void computeDocking() {

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    int tagId =
        (alliance == Alliance.Red)
            ? Constants.Climb.Hardware.CLIMB_TAG_IDS[1]
            : Constants.Climb.Hardware.CLIMB_TAG_IDS[0];

    // =========================================================
    // SIM
    // =========================================================
    if (Constants.currentMode == Constants.Mode.SIM) {

      Pose2d robot = robotPoseSupplier.get();
      if (robot == null) return;

      var tagOpt = VisionConstants.aprilTagLayout.getTagPose(tagId);
      if (tagOpt.isEmpty()) return;

      Pose2d tag = tagOpt.get().toPose2d();

      // THIS is the dock point in field space

      Transform2d tagToDock =
          new Transform2d(
              new Translation2d(
                  Constants.Climb.Vision.DOCK_OFFSET_X.get(),
                  Constants.Climb.Vision.DOCK_OFFSET_Y.get()),
              new Rotation2d(0));

      Pose2d dockPose = tag.plus(tagToDock);

      // FINAL measurement: robot → dock
      Transform2d raw = new Transform2d(robot, dockPose);

      updateDock(raw);
      return;
    }

    // =====================================================
    // REAL LIMELIGHT MODE
    // =====================================================
    double seen = LimelightHelpers.getFiducialID(Constants.Climb.Vision.REAR_LIMELIGHT);

    if ((int) seen != tagId) return;

    double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.Climb.Vision.REAR_LIMELIGHT);

    if (pose == null || pose.length < 6) {
      Logger.recordOutput("Dock/LimelightPoseValid", false);
      return;
    }

    Logger.recordOutput("Dock/LL/X", pose[0]);
    Logger.recordOutput("Dock/LL/Z", pose[2]);
    Logger.recordOutput("Dock/LimelightPoseValid", true);
    Logger.recordOutput("Dock/LimelightX", pose[0]);
    Logger.recordOutput("Dock/LimelightY", pose[1]);
    Logger.recordOutput("Dock/LimelightZ", pose[2]);
    Logger.recordOutput("Dock/LimelightYaw", pose[5]);

    // Camera to AprilTag measurement
    Transform2d cameraToTag =
        new Transform2d(
            new Translation2d(pose[2], -pose[0]),
            Rotation2d.fromDegrees(pose[5] + 180 + Constants.Climb.Vision.DOCK_OFFSET_YAW.get()));

    // Your desired robot position relative to the tag
    Transform2d tagToDock =
        new Transform2d(
            new Translation2d(
                Constants.Climb.Vision.DOCK_OFFSET_X.get(),
                Constants.Climb.Vision.DOCK_OFFSET_Y.get()),
            Rotation2d.fromDegrees(Constants.Climb.Vision.DOCK_OFFSET_YAW.get()));

    // Camera -> tag -> dock
    Transform2d raw = cameraToTag.plus(tagToDock);

    updateDock(raw);
  }

  // =========================================================
  // CONSUMER INTERFACE
  // =========================================================
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
