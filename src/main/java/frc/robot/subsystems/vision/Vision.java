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
  // BASIC ANGLE HELPERS (rear camera)
  // ==========================================================

  public double getTX() {
    return inputs[REAR_CAMERA].latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    return inputs[REAR_CAMERA].latestTargetObservation.ty().getDegrees();
  }

  // ==========================================================
  // BEST POSE ESTIMATE (FIXED + RESTORED)
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
  // CLIMB TAG POSE (FIELD FIXED)
  // ==========================================================

  public Optional<Pose2d> getClimbTagPose() {

    int tagId =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Constants.Climb.Hardware.CLIMB_TAG_IDS[1]
            : Constants.Climb.Hardware.CLIMB_TAG_IDS[0];

    return VisionConstants.aprilTagLayout.getTagPose(tagId).map(Pose3d::toPose2d);
  }

  // ==========================================================
  // DOCKING TARGET (RAW LIMELIGHT SPACE ONLY)
  // ==========================================================

  public Optional<Transform2d> getDockingTarget() {

    double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.Climb.Vision.REAR_LIMELIGHT);

    if (pose == null || pose.length < 6) return Optional.empty();

    double strafe = pose[0];
    double forward = -pose[2];
    double yawDeg = pose[5];

    return Optional.of(
        new Transform2d(new Translation2d(forward, strafe), Rotation2d.fromDegrees(yawDeg)));
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
