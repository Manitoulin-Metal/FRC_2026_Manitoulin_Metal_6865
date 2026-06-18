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
  // DOCK TARGET (UNIFIED)
  // ==========================================================
  public Optional<Transform2d> getDockingTarget() {

    // =========================
    // SIMULATION
    // =========================
    if (Constants.currentMode == Constants.Mode.SIM) {

      Pose2d robotPose = robotPoseSupplier.get();
      if (robotPose == null) return Optional.empty();

      int tagId =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
              ? Constants.Climb.Hardware.CLIMB_TAG_IDS[1]
              : Constants.Climb.Hardware.CLIMB_TAG_IDS[0];

      var tagOpt = VisionConstants.aprilTagLayout.getTagPose(tagId);
      if (tagOpt.isEmpty()) return Optional.empty();

      Pose2d tagPose = tagOpt.get().toPose2d();

      // =========================================================
      // CORRECT SIMULATION OF LIMELIGHT TARGET SPACE
      // =========================================================

      Transform2d robotToTag = new Transform2d(robotPose, tagPose);

      Logger.recordOutput("Dock/RawForward", robotToTag.getX());
      Logger.recordOutput("Dock/RawStrafe", robotToTag.getY());
      Logger.recordOutput("Dock/RawYawDeg", robotToTag.getRotation().getDegrees());

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

    if ((int) seenTag != desiredTag) return Optional.empty();

    double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.Climb.Vision.REAR_LIMELIGHT);

    if (pose == null || pose.length < 6) return Optional.empty();

    Transform2d robotToTag =
        new Transform2d(new Translation2d(pose[0], pose[1]), Rotation2d.fromDegrees(-pose[5]));

    // log real too (important for consistency)
    Logger.recordOutput("Dock/RawForward", pose[0]);
    Logger.recordOutput("Dock/RawStrafe", pose[1]);
    Logger.recordOutput("Dock/RawYawDeg", pose[5]);

    return Optional.of(robotToTag);
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

    for (int camera = 0; camera < inputs.length; camera++) {
      for (var obs : inputs[camera].poseObservations) {

        boolean reject =
            obs.tagCount() == 0 || obs.ambiguity() > 0.3 || Math.abs(obs.pose().getZ()) > 1.0;

        if (reject) continue;

        consumer.accept(obs.pose().toPose2d(), obs.timestamp(), VecBuilder.fill(0.05, 0.05, 0.05));
      }
    }
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
