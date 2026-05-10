package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " disconnected.", AlertType.kWarning);
    }
  }
  // ==================================================
  // Implicit enable/disable functionality for the entire subsystem. This is
  // useful for
  // preventing vision updates during certain phases of the match (e.g. endgame
  // climb).
  // ==

  private boolean enabled = true;

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  // ==================================================
  // Rear Camera Helpers (camera0)
  // ==================================================

  public double getTX() {
    return inputs[REAR_CAMERA].latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    return inputs[REAR_CAMERA].latestTargetObservation.ty().getDegrees();
  }

  public boolean hasRearTag(int[] climbTagIds) {
    for (int tag : inputs[REAR_CAMERA].tagIds) {
      for (int climbTag : climbTagIds) {
        if (tag == climbTag) return true;
      }
    }
    return false;
  }

  public boolean shouldUseVisionForClimb() {
    return hasRearTag(Constants.Climb.Hardware.CLIMB_TAG_IDS);
  }

  public Optional<Transform2d> getRobotRelativeError() {
    if (!inputs[REAR_CAMERA].connected) {
      return Optional.empty();
    }

    double tx = getTX();
    double ty = getTY();

    return Optional.of(
        new Transform2d(-ty, tx, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(tx)));
  }

  public Optional<Pose3d> getRearTargetSpacePose() {

    if (!inputs[VisionConstants.REAR_CAMERA].connected) {
      return Optional.empty();
    }

    if (!inputs[VisionConstants.REAR_CAMERA].hasTargets) {
      return Optional.empty();
    }

    return Optional.of(inputs[VisionConstants.REAR_CAMERA].targetSpacePose);
  }
  // ==================================================
  // Front Pose Helpers
  // ==================================================

  public Optional<Pose2d> getEstimatedPoseFromCamera1() {
    return getLatestPose(FRONT_CAMERA);
  }

  public Pose2d getEstimatedPose() {
    return getLatestPose(FRONT_CAMERA).orElse(null);
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

  public boolean hasTag(int id) {
    for (int c = 0; c < inputs.length; c++) {
      for (int tag : inputs[c].tagIds) {
        if (tag == id) return true;
      }
    }
    return false;
  }

  public Optional<Pose3d> getRearTargetSpacePoseForClimb() {

    if (!inputs[REAR_CAMERA].connected) {
      return Optional.empty();
    }

    if (!inputs[REAR_CAMERA].hasTargets) {
      return Optional.empty();
    }

    boolean validTag = false;

    for (int tag : inputs[REAR_CAMERA].tagIds) {
      if (tag == 32 || tag == 16) {
        validTag = true;
        break;
      }
    }

    if (!validTag) {
      return Optional.empty();
    }

    return Optional.of(inputs[REAR_CAMERA].targetSpacePose);
  }

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

    // ONLY front camera contributes pose
    int cameraIndex = FRONT_CAMERA;

    for (var observation : inputs[cameraIndex].poseObservations) {

      boolean reject =
          observation.tagCount() == 0
              || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
              || (observation.tagCount() == 1 && observation.averageTagDistance() > 3.5)
              || Math.abs(observation.pose().getZ()) > maxZError
              || observation.pose().getX() < 0
              || observation.pose().getX() > aprilTagLayout.getFieldLength()
              || observation.pose().getY() < 0
              || observation.pose().getY() > aprilTagLayout.getFieldWidth();

      if (reject) {
        rejected.add(observation.pose());
        continue;
      }

      accepted.add(observation.pose());

      Logger.recordOutput("Vision/Camera" + cameraIndex + "/Pose", observation.pose().toPose2d());
      Logger.recordOutput("Vision/LatestPose", observation.pose().toPose2d());

      double factor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
      // replacing with...
      // double linear = linearStdDevBaseline * factor;
      // double angular = angularStdDevBaseline * factor;

      // if (observation.type() == PoseObservationType.MEGATAG_2) {
      // linear *= linearStdDevMegatag2Factor;
      // angular *= angularStdDevMegatag2Factor;
      // }

      double linear = linearStdDevBaseline * factor;
      double angular = angularStdDevBaseline * factor;

      if (observation.type() == PoseObservationType.MEGATAG_2) {

        linear *= linearStdDevMegatag2Factor;

        // Let gyro dominate heading
        angular = 9999999.0;

      } else {

        angular *= angularStdDevMegatag2Factor;
      }

      linear *= cameraStdDevFactors[cameraIndex];
      angular *= cameraStdDevFactors[cameraIndex];

      consumer.accept(
          observation.pose().toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(linear, linear, angular));
    }

    Logger.recordOutput("Vision/AcceptedPoses", accepted.toArray(new Pose3d[0]));

    Logger.recordOutput("Vision/RejectedPoses", rejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
