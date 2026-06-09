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
  // DOCKING TARGET FOR CLIMB
  //
  // Returns:
  //   SIM  -> robot → desired dock pose
  //   REAL -> robot → tag measurement from Limelight
  //
  // The climb command is responsible for converting
  // measurements into docking errors.
  // ==========================================================

  public Optional<Transform2d> getDockingTarget() {

    // ==========================================================
    // SIMULATION
    // ==========================================================
    if (Constants.currentMode == Constants.Mode.SIM) {

      Pose2d robotPose = robotPoseSupplier.get();

      if (robotPose == null) {
        return Optional.empty();
      }

      int tagId = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 16 : 32;

      var tagOpt = VisionConstants.aprilTagLayout.getTagPose(tagId);

      if (tagOpt.isEmpty()) {
        return Optional.empty();
      }

      Pose2d tagPose = tagOpt.get().toPose2d();

      // ========================================================
      // SIM TARGET POSE
      //
      // This is the desired climb position used only in SIM.
      // The dock command drives robot → targetPose until
      // the transform becomes (0,0,0).
      // ========================================================

      Pose2d targetPose =
          tagPose.transformBy(
              new Transform2d(
                  1.15, // forward from tag
                  0.30, // sideways offset
                  Rotation2d.kZero));

      Logger.recordOutput("Dock/SimTargetPose", targetPose);

      return Optional.of(new Transform2d(robotPose, targetPose));
    }

    // ==========================================================
    // REAL LIMELIGHT PATH
    // ==========================================================

    int desiredTag = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 16 : 32;

    double seenTag = LimelightHelpers.getFiducialID(Constants.Climb.Vision.REAR_LIMELIGHT);

    // only accept the climb tag
    if ((int) seenTag != desiredTag) {
      return Optional.empty();
    }

    double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.Climb.Vision.REAR_LIMELIGHT);

    if (pose == null || pose.length < 6) {
      return Optional.empty();
    }

    // ==========================================================
    // LIMELIGHT TARGET SPACE
    //
    // These are the raw values you will tune from.
    // Read these when the robot is physically docked.
    // ==========================================================

    double rawForward = pose[0];
    double rawStrafe = pose[1];
    double rawYawDeg = pose[5];

    // ==========================================================
    // TUNING LOGS
    // ==========================================================

    Logger.recordOutput("Dock/RawForward", rawForward);
    Logger.recordOutput("Dock/RawStrafe", rawStrafe);
    Logger.recordOutput("Dock/RawYawDeg", rawYawDeg);

    Logger.recordOutput("Dock/TargetForward", Constants.Climb.Vision.targetForward.get());

    Logger.recordOutput("Dock/TargetStrafe", Constants.Climb.Vision.targetStrafe.get());

    Logger.recordOutput("Dock/TargetYawDeg", Constants.Climb.Vision.targetYawDeg.get());

    // ==========================================================
    // RETURN PURE MEASUREMENT
    //
    // robot → tag
    // ==========================================================

    return Optional.of(
        new Transform2d(
            new Translation2d(rawForward, rawStrafe), Rotation2d.fromDegrees(rawYawDeg)));
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
